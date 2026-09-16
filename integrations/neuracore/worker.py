"""Isolated Neuracore SDK process; IPC socket is inherited from the parent."""
from multiprocessing.connection import Connection
from pathlib import Path
import sys
import time
import traceback
import xml.etree.ElementTree as ET
from unittest.mock import patch

import neuracore as nc
import numpy as np


def setup(config):
    urdf = Path(config['urdf'])
    for mesh in ET.parse(urdf).getroot().iter('mesh'):
        path = urdf.parent / mesh.attrib['filename']
        if not path.is_file():
            raise FileNotFoundError(f'Missing URDF mesh: {path}')
    print('Logging in to Neuracore before robot initialization...', flush=True)
    nc.login()
    # Scoped model-upload timeout from record_synthetic.py; also applies to 17.0.1.
    from neuracore.core.utils.http_session import thread_local_session
    session = thread_local_session()
    original_put = session.put

    def upload(url, **kwargs):
        if 'robot_package' in kwargs.get('files', {}):
            kwargs['timeout'] = (config['upload_timeout'], config['upload_timeout'])
        return original_put(url, **kwargs)

    with patch.object(session, 'put', side_effect=upload):
        nc.connect_robot(robot_name='unitree_g1_29dof_inspire_ftp', urdf_path=str(urdf))
    nc.create_dataset(name=config['dataset'], description=config['description'])


def log_sample(sample, last_joint_timestamp=float('-inf')):
    # Repeated DDS snapshots must not produce duplicate joint timestamps;
    # samples received before the episode's start are excluded as well.
    if sample['timestamp'] > last_joint_timestamp:
        nc.log_joint_positions(sample['positions'], timestamp=sample['timestamp'])
        nc.log_joint_velocities(sample['velocities'], timestamp=sample['timestamp'])
    nc.log_joint_target_positions(sample['targets'], timestamp=sample['target_timestamp'])
    for name, values in sample['hands'].items():
        nc.log_custom_1d(name, np.asarray(values), timestamp=sample['target_timestamp'])
    for name, (timestamp, shape, pixels) in sample['cameras'].items():
        nc.log_rgb(name, np.frombuffer(pixels, dtype=np.uint8).reshape(shape), timestamp=timestamp)
    return max(last_joint_timestamp, sample['timestamp'])


def run(connection):
    active = False
    try:
        config = connection.recv()
        setup(config)
        connection.send({'ok': True})
        while True:
            command = connection.recv()
            op = command['op']
            if op == 'start':
                nc.start_recording(timestamp=command['timestamp'])
                active = True
                last_joint_timestamp = command['timestamp']
                nc.log_language('instruction', config['instruction'], timestamp=command['timestamp'])
            elif op == 'sample':
                if not active:
                    raise RuntimeError('Sample received outside recording')
                last_joint_timestamp = log_sample(command['sample'], last_joint_timestamp)
            elif op == 'stop':
                if active:
                    nc.stop_recording(wait=True, timestamp=command['timestamp'])
                    active = False
            elif op == 'close':
                if active:
                    nc.stop_recording(wait=True, timestamp=time.time())
                    active = False
                connection.send({'ok': True})
                return
            else:
                raise ValueError(f'Unknown recorder command: {op}')
            connection.send({'ok': True})
    except EOFError:
        pass
    except Exception as exc:
        traceback.print_exc()
        try:
            connection.send({'error': str(exc)})
        except (OSError, EOFError):
            pass
    finally:
        try:
            if active:
                nc.stop_recording(wait=True, timestamp=time.time())
        finally:
            connection.close()


if __name__ == '__main__':
    run(Connection(int(sys.argv[1])))

"""Nonblocking Neuracore recording bridge; SDK runs in its own environment."""
import logging
from multiprocessing.connection import Connection
from pathlib import Path
import queue
import socket
import subprocess
import threading
import time

import numpy as np

logger = logging.getLogger(__name__)
ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / 'assets/g1_neuracore/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf'
JOINT_NAMES = tuple(
    f'{side}_{joint}_joint'
    for side in ('left', 'right')
    for joint in ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle_pitch', 'ankle_roll')
) + ('waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint') + tuple(
    f'{side}_{joint}_joint'
    for side in ('left', 'right')
    for joint in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
                  'wrist_roll', 'wrist_pitch', 'wrist_yaw')
)


def make_sample(arm, target, hands, cameras, camera_config):
    """Use one coherent DDS snapshot; timestamps are host Unix seconds.

    TeleImager supplies no capture timestamp, so RGB is timestamped when read
    here. No claim of hardware synchronization between camera and DDS clocks.
    """
    state = arm.get_recording_state()
    q, dq = np.asarray(state['q'], dtype=float), np.asarray(state['dq'], dtype=float)
    target = np.asarray(target, dtype=float)
    if q.shape != (29,) or dq.shape != (29,) or target.shape != (14,):
        raise ValueError('Neuracore requires 29 G1 joints and 14 arm targets')
    if not all(np.isfinite(v).all() for v in (q, dq, target)):
        raise ValueError('Non-finite joint data; stopping recording')
    now = time.time()
    result = dict(timestamp=state['timestamp'],
                  positions=dict(zip(JOINT_NAMES, q.tolist())),
                  velocities=dict(zip(JOINT_NAMES, dq.tolist())),
                  target_timestamp=now,
                  targets=dict(zip(JOINT_NAMES[15:], target.tolist())),
                  hands={f'{side}_hand_{kind}': list(getattr(hands, f'{side}_{attr}'))
                         for side in ('left', 'right')
                         for kind, attr in (('state', 'state'), ('target', 'action'))},
                  cameras={})
    for name, frame in cameras.items():
        if not camera_config[name]['enable_zmq']:
            continue
        if frame is None or frame.bgr is None:
            raise ValueError(f'{name}: RGB frame unavailable; stopping recording')
        bgr = frame.bgr
        views = {name: bgr}
        if name == 'head_camera' and camera_config[name]['binocular']:
            mid = bgr.shape[1] // 2
            views = {'head_camera_left': bgr[:, :mid], 'head_camera_right': bgr[:, mid:]}
        for view, pixels in views.items():
            rgb = np.ascontiguousarray(pixels[:, :, ::-1])
            result['cameras'][view] = (now, rgb.shape, rgb.tobytes())
    return result


class NeuracoreRecorder:
    """Match episode controls, but keep cloud calls off the robot loop.

    A short bounded sample backlog fails recording visibly on overload. Stop
    commands are always queued, including after pause/tracking loss/damping.
    """
    def __init__(self, args):
        python = Path(args.neuracore_python).expanduser()
        if not python.is_file():
            raise RuntimeError('Install Neuracore first: uv sync --project integrations/neuracore; '
                               'or pass --neuracore-python /path/to/python')
        self.episode_id = 0
        self.active = False
        self.ready = threading.Event()
        self.error = None
        self.commands = queue.Queue()
        parent, child = socket.socketpair()
        self.connection = Connection(parent.detach())
        try:
            self.process = subprocess.Popen(
                [str(python), str(ROOT / 'integrations/neuracore/worker.py'), str(child.fileno())],
                pass_fds=(child.fileno(),))
        except BaseException:
            self.connection.close()
            raise
        finally:
            child.close()
        try:
            logger.info('Neuracore login/model setup before robot initialization...')
            self.connection.send(dict(urdf=str(URDF), dataset=args.neuracore_dataset or args.task_name,
                                      description=args.task_desc, instruction=args.task_goal,
                                      upload_timeout=args.neuracore_upload_timeout))
            self._reply()  # interactive login is deliberately before robot setup
        except BaseException:
            self.connection.close()
            self.process.terminate()
            self.process.wait()
            raise
        self.ready.set()
        self.thread = threading.Thread(target=self._run, name='neuracore-recorder', daemon=True)
        self.thread.start()

    def _reply(self):
        reply = self.connection.recv()
        if reply.get('error'):
            raise RuntimeError(reply['error'])

    def _run(self):
        try:
            while True:
                command = self.commands.get()
                self.connection.send(command)
                self._reply()
                if command['op'] == 'stop':
                    logger.info('[Neuracore] Episode finalized and upload completed')
                    self.ready.set()
                if command['op'] == 'close':
                    return
        except Exception as exc:
            self.error = str(exc)
            logger.exception('[Neuracore] Recording worker failed')
        finally:
            self.connection.close()

    def is_ready(self):
        return self.ready.is_set() and self.error is None and self.process.poll() is None

    def create_episode(self):
        if not self.is_ready():
            return False
        self.ready.clear()
        self.active = True
        self.episode_id += 1
        self.commands.put(dict(op='start', timestamp=time.time()))
        return True

    def add_sample(self, sample):
        if self.error or self.process.poll() is not None:
            raise RuntimeError(self.error or 'Neuracore worker exited')
        if not self.active:
            return
        if self.commands.qsize() >= 8:
            raise RuntimeError('Neuracore backlog full; recording stopped to protect control latency')
        self.commands.put(dict(op='sample', sample=sample))

    def save_episode(self):
        if self.active:
            self.active = False
            self.commands.put(dict(op='stop', timestamp=time.time()))

    def close(self):
        self.save_episode()
        self.commands.put(dict(op='close'))
        self.thread.join()
        self.process.wait()
        if self.error:
            raise RuntimeError(self.error)

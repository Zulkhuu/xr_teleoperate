"""A device-selected ADB tunnel for Quest Browser video and tracking."""
import logging
import subprocess
import threading

logger = logging.getLogger(__name__)
QUEST_URL = 'https://localhost:8012/?ws=wss://localhost:8012&grid=False'


class QuestUSBConnection:
    def __init__(self, adb_path='adb', serial=None):
        self.adb_path = adb_path
        self.serial = serial
        self.owns_reverse = False
        self.stop_event = threading.Event()
        self.failure = None
        self.monitor = None

    def _adb(self, *args, targeted=True):
        command = [self.adb_path]
        if targeted:
            command += ['-s', self.serial]
        try:
            return subprocess.run(command + list(args), capture_output=True, text=True,
                                  timeout=5, check=True).stdout.strip()
        except (OSError, subprocess.SubprocessError) as exc:
            raise RuntimeError(f'Quest USB ADB failed: {exc}. Check cable and USB debugging.') from exc

    def start(self):
        rows = [line.split() for line in self._adb('devices', '-l', targeted=False).splitlines()]
        devices = [row for row in rows if len(row) >= 2
                   and any(item.startswith('usb:') for item in row[2:])
                   and (self.serial is None or row[0] == self.serial)]
        if len(devices) != 1:
            raise RuntimeError('Connect one USB Quest or select --quest-serial. Check adb devices -l.')
        self.serial, state = devices[0][:2]
        if state != 'device':
            raise RuntimeError(f'Quest is {state}; accept USB debugging in the headset.')
        mappings = self._adb('reverse', '--list').splitlines()
        existing = [row.split()[-1] for row in mappings
                    if len(row.split()) >= 3 and row.split()[-2] == 'tcp:8012']
        if existing and existing != ['tcp:8012']:
            raise RuntimeError('Quest port 8012 already forwards elsewhere.')
        if not existing:
            self._adb('reverse', '--no-rebind', 'tcp:8012', 'tcp:8012')
            self.owns_reverse = True
        self.monitor = threading.Thread(target=self._monitor, daemon=True)
        self.monitor.start()

    def _monitor(self):
        while not self.stop_event.wait(1):
            try:
                if self._adb('get-state') != 'device':
                    raise RuntimeError('Quest USB disconnected')
                rows = self._adb('reverse', '--list').splitlines()
                if not any(row.split()[-2:] == ['tcp:8012', 'tcp:8012'] for row in rows):
                    raise RuntimeError('Quest USB tunnel lost')
            except RuntimeError as exc:
                self.failure = exc
                return

    def check(self):
        if self.failure is not None:
            raise RuntimeError(f'Quest USB connection lost: {self.failure}')

    def close(self):
        self.stop_event.set()
        if self.monitor is not None:
            self.monitor.join(timeout=11)
        if self.owns_reverse:
            self.owns_reverse = False
            try:
                rows = self._adb('reverse', '--list').splitlines()
                if any(row.split()[-2:] == ['tcp:8012', 'tcp:8012'] for row in rows):
                    self._adb('reverse', '--remove', 'tcp:8012')
            except RuntimeError as exc:
                logger.warning('Could not clean up Quest USB tunnel: %s', exc)

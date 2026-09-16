"""Serialize damping inhibition with the background DDS publisher."""
import threading
import time


class ArmCommandGate:
    def _init_command_gate(self):
        self._command_gate = threading.Lock()
        self.commands_suspended = False
        self._command_epoch = 0
        self._publish_count = 0
        self.state_received_at = time.monotonic()

    def suspend_commands(self):
        # Once this returns, no normal arm DDS write can race with Damp().
        with self._command_gate:
            self.commands_suspended = True
            self._command_epoch += 1

    def resume_commands(self):
        # Caller must install a fresh measured-pose target first.
        with self._command_gate:
            self.commands_suspended = False
            self._command_epoch += 1

    def _publish_command(self, epoch):
        with self._command_gate:
            if not self.commands_suspended and epoch == self._command_epoch:
                self.lowcmd_publisher.Write(self.msg)
                self._publish_count += 1
                return True
            return False

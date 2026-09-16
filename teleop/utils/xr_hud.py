"""Compact OpenCV HUD, repeated independently in side-by-side eye images."""
import cv2

from teleop.utils.quest_control import TeleopState


def draw_hud(image, state, tracking, recording=False, elapsed=0, episode=0,
             binocular=False, reason='', recording_enabled=True,
             preparation_required=False):
    white, green, yellow, red = (255, 255, 255), (80, 240, 80), (0, 220, 255), (40, 40, 255)
    instructions = {
        TeleopState.WAITING_FOR_XR: ['Waiting for XR tracking...'],
        TeleopState.WAITING_FOR_PREPARATION: ['ARM: NOT PREPARED', 'Hold X: initial safety pose', 'Then B: set anchor'],
        TeleopState.WAITING_FOR_ANCHOR: ['ARM: NOT ALIGNED', 'B: set anchor'],
        TeleopState.ALIGNED: ['ARM: ALIGNED', 'Hold X to start', 'B: re-align'],
        TeleopState.PREPARING: ['ARM: PREPARING', 'Safety preparation motion', 'A: pause'],
        TeleopState.ACTIVE: ['ARM: ACTIVE', 'Y: record' if recording_enabled else 'Recording disabled', 'A: pause'],
        TeleopState.PAUSED: ['ARM: PAUSED', 'B: re-align, then hold X', 'Hold A: exit'],
        TeleopState.DAMPED: ['DAMPING / ESTOP', 'Robot control disabled', 'Check robot', 'B: re-align after recovery'],
        TeleopState.EXITING: ['Exiting...'],
    }
    if preparation_required:
        instructions[TeleopState.PAUSED] = ['ARM: PAUSED', 'Hold X: retry preparation', 'Hold A: exit']
    eyes = [image]
    if binocular:
        mid = image.shape[1] // 2
        eyes = [image[:, :mid], image[:, mid:]]
    for eye in eyes:
        scale = min(0.55, eye.shape[1] / 850)
        step = max(12, round(26 * scale / 0.55))
        color = red if state == TeleopState.DAMPED else green if state == TeleopState.ACTIVE else yellow
        lines = instructions[state][:]
        lines.insert(1, 'TRACKING: OK' if tracking else 'TRACKING: LOST')
        if reason:
            lines.append(reason)
        def text(value, x, y, ink):
            cv2.putText(eye, value, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(eye, value, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, ink, 1, cv2.LINE_AA)
        for i, line in enumerate(lines):
            text(line, 10, step * (i + 1), color if i == 0 else white)
        if recording:
            seconds = int(max(0, elapsed))
            x = max(10, eye.shape[1] - round(155 * scale / .55))
            cv2.circle(eye, (x - 7, step - 4), 3, red, -1, cv2.LINE_AA)
            text(f'REC {seconds // 60:02d}:{seconds % 60:02d}', x, step, red)
            text(f'EP {episode:03d}', x, step * 2, red)
    return image

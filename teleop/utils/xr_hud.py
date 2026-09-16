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
        # Quest Browser crops/overscans the extreme edges of each eye.  Keep a
        # generous per-eye safe margin and use a slightly smaller type size so
        # the HUD remains readable without being clipped.
        scale = min(0.45, eye.shape[1] / 1000)
        step = max(12, round(24 * scale / 0.45))
        margin_x = max(24, round(eye.shape[1] * 0.04))
        margin_y = step
        color = red if state == TeleopState.DAMPED else green if state == TeleopState.ACTIVE else yellow
        lines = instructions[state][:]
        lines.insert(1, 'TRACKING: OK' if tracking else 'TRACKING: LOST')
        if reason:
            lines.append(reason)
        def text(value, x, y, ink):
            cv2.putText(eye, value, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(eye, value, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, ink, 1, cv2.LINE_AA)
        for i, line in enumerate(lines):
            text(line, margin_x, margin_y + step * i, color if i == 0 else white)
        if recording:
            seconds = int(max(0, elapsed))
            rec = f'REC {seconds // 60:02d}:{seconds % 60:02d}'
            ep = f'EP {episode:03d}'
            rec_width = cv2.getTextSize(rec, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0]
            ep_width = cv2.getTextSize(ep, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0]
            x = eye.shape[1] - margin_x - max(rec_width, ep_width)
            cv2.circle(eye, (x - 7, margin_y - 4), 3, red, -1, cv2.LINE_AA)
            text(rec, x, margin_y, red)
            text(ep, eye.shape[1] - margin_x - ep_width, margin_y + step, red)
    return image

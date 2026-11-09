"""Compact OpenCV HUD, repeated independently in side-by-side eye images."""
import cv2
import numpy as np

from teleop.utils.quest_control import TeleopState


def prepare_xr_image(raw_image, binocular=False, scale=0.8, offset_y=0.0):
    """Inset the complete camera view per eye; positive offset moves it down.

    Offset is a fraction of eye height. This changes display pixels, not the
    Vuer plane or tracking coordinates. The source frame is never modified.
    """
    if scale == 1.0:
        return raw_image.copy()
    image = np.zeros_like(raw_image)
    width = raw_image.shape[1]
    bounds = (0, width // 2, width) if binocular else (0, width)
    for left, right in zip(bounds, bounds[1:]):
        source = raw_image[:, left:right]
        height, eye_width = source.shape[:2]
        new_width = max(1, round(eye_width * scale))
        new_height = max(1, round(height * scale))
        x = (eye_width - new_width) // 2
        y = round((height - new_height) / 2 + height * offset_y)
        y = max(0, min(height - new_height, y))
        image[y:y + new_height, left + x:left + x + new_width] = cv2.resize(
            source, (new_width, new_height), interpolation=cv2.INTER_AREA)
    return image


def draw_hud(image, state, tracking, recording=False, elapsed=0, episode=0,
             binocular=False, reason='', recording_enabled=True,
             preparation_required=False, input_mode='controller'):
    white, green, yellow, red = (255, 255, 255), (80, 240, 80), (0, 220, 255), (40, 40, 255)
    hand = input_mode == 'hand'
    prepare = 'R: prepare arms' if hand else 'Hold X (0.5s): prepare arms'
    follow = 'R: follow' if hand else 'Hold LEFT GRIP: follow'
    pause = 'P: pause' if hand else 'Release LEFT GRIP: pause'
    exit_hint = 'Q: exit' if hand else 'Hold A (1.5s): exit'
    record = ('S' if hand else 'Y') + (': stop recording' if recording else ': start recording')
    instructions = {
        TeleopState.WAITING_FOR_XR: ['WAITING FOR TRACKING / ROBOT DATA'],
        TeleopState.WAITING_FOR_PREPARATION: ['ARMS NOT PREPARED', prepare],
        TeleopState.WAITING_FOR_ANCHOR: ['ARMS PREPARED', 'B: set anchor' if hand else follow],
        TeleopState.ALIGNED: ['ANCHOR SET / READY', follow],
        TeleopState.PREPARING: ['PREPARING ARMS', 'Moving to initial pose',
                                'P: interrupt' if hand else 'Press A: interrupt'],
        TeleopState.ACTIVE: ['TELEOPERATING / RECORDING' if recording else 'TELEOPERATING',
                             pause, record if recording_enabled else 'Recording unavailable'],
        TeleopState.PAUSED: ['PAUSED / HOLDING POSE',
                             'B: set anchor, then R: follow' if hand else follow, exit_hint],
        TeleopState.DAMPED: ['ARM COMMANDS SUSPENDED', 'Check robot before recovery',
                             'B: acknowledge recovery' if hand else 'Hold X (0.5s): acknowledge recovery',
                             exit_hint],
        TeleopState.EXITING: ['Exiting...'],
    }
    if preparation_required:
        instructions[TeleopState.PAUSED] = ['PAUSED / PREPARATION REQUIRED', prepare, exit_hint]
    eyes = [image]
    if binocular:
        mid = image.shape[1] // 2
        eyes = [image[:, :mid], image[:, mid:]]
    for eye in eyes:
        # Keep text inside the central visible area of each eye. Headset
        # optics/view settings can leave image corners outside the field of view.
        scale = min(0.45, eye.shape[1] / 1000)
        step = max(12, round(24 * scale / 0.45))
        color = red if not tracking or state == TeleopState.DAMPED else green if state == TeleopState.ACTIVE else yellow
        lines = instructions[state][:]
        lines.insert(1, 'TRACKING: OK' if tracking else 'TRACKING: LOST')
        if reason:
            lines.append(reason)
        inks = [color] + [white] * (len(lines) - 1)
        if recording:
            seconds = int(max(0, elapsed))
            lines.append(f'REC {seconds // 60:02d}:{seconds % 60:02d}   EP {episode:03d}')
            inks.append(red)
        # Bottom-center per eye, with clearance from the headset's lower edge.
        bottom = eye.shape[0] - max(step, round(eye.shape[0] * 0.12))
        top = max(step, bottom - step * (len(lines) - 1))
        def text(value, x, y, ink):
            cv2.putText(eye, value, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(eye, value, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, ink, 1, cv2.LINE_AA)
        for i, line in enumerate(lines):
            width = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0]
            x = max(4, (eye.shape[1] - width) // 2)
            y = top + step * i
            text(line, x, y, inks[i])
            if recording and i == len(lines) - 1:
                cv2.circle(eye, (x - 7, y - 4), 3, red, -1, cv2.LINE_AA)
    return image

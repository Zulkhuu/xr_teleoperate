# xr_teleoperate Current Configuration and Run Procedure

This document records the currently working configuration for this Host and this Unitree G1. Instructions in older documentation concerning `pixi`, Dex3, the DFX Inspire bridge, and `teleimager-server --rs` no longer apply to the current setup.

## 1. Current Hardware and Network

### Hardware

- Robot: Unitree G1 EDU, `G1_29`, locked waist
- Dexterous hands: Inspire FTP / RH56DFTP, using `--ee=inspire_ftp`
- Head camera: Intel RealSense D435i, currently used by teleimager through the OpenCV camera path
- XR: Meta Quest 3S, currently using controller input rather than hand tracking

### Network

| Device | IP / Interface | Purpose |
| --- | --- | --- |
| Host PC | `eno1 = 192.168.123.222` | Runs the main teleop program; connects to DDS and the hand ModbusTCP interface |
| G1 PC2 / image server | `192.168.123.164` | Runs teleimager and provides RGB images |
| G1 robot controller | `192.168.123.161` | Unitree motion / mode service |
| Inspire FTP left hand | `192.168.123.210:6000` | ModbusTCP |
| Inspire FTP right hand | `192.168.123.211:6000` | ModbusTCP |
| Quest 3S | Same Wi-Fi network as the Host | Opens the Vuer page |

Robot SSH:

```bash
ssh unitree@192.168.123.164
# Password: 123
# If prompted after login with ros: foxy(1) noetic(2), choose 1
```

## 2. Host Environment

The current setup uses `uv` and the repository's `.venv`, not `pixi`.

Common verification commands:

```bash
cd /home/zc1525/xr_teleoperate
uv run python -c "import pinocchio, casadi, pymodbus, teleimager, televuer, unitree_sdk2py; print('env ok')"
uv pip list | grep -E 'inspire-sdkpy|teleimager|televuer|unitree-sdk2py|pymodbus'
```

The current key packages should include:

- `teleimager` editable: `/home/zc1525/xr_teleoperate/teleop/teleimager`
- `televuer` editable: `/home/zc1525/xr_teleoperate/teleop/televuer`
- `inspire-sdkpy` editable: `/home/zc1525/inspire_hand_ws/inspire_hand_sdk`
- `unitree-sdk2py`
- `pymodbus`

If DDS initialization fails, first confirm the Host's robot-facing network interface name:

```bash
ip addr show eno1
```

Normally, you should see `192.168.123.222/24`.

When starting teleop, use:

```bash
--network-interface=eno1
```

## 3. Start the RGB Image Service

On PC2, the currently stable path is OpenCV mode; do not use `--rs`. Previously, `--rs` failed because `librealsense2.so.2.50` was missing.

Run from the Host:

```bash
ssh unitree@192.168.123.164
echo 123 | sudo -S modprobe uvcvideo
pkill -f teleimager-server || true
nohup ~/.local/bin/teleimager-server > /tmp/teleimager.log 2>&1 &
tail -f /tmp/teleimager.log
```

The normal log should contain something similar to:

```text
[OpenCVCamera: head_camera] initialized with 480x640 @ 30 FPS
head_camera is ready
```

If the RGB feed drops out, first reset the RealSense USB device on PC2, then restart teleimager:

```bash
ssh unitree@192.168.123.164
echo 123 | sudo -S sh -c 'echo 0 > /sys/bus/usb/devices/2-3/authorized; sleep 2; echo 1 > /sys/bus/usb/devices/2-3/authorized'
pkill -f teleimager-server || true
nohup ~/.local/bin/teleimager-server > /tmp/teleimager.log 2>&1 &
tail -f /tmp/teleimager.log
```

Quickly verify the image from the Host side:

```bash
cd /home/zc1525/xr_teleoperate/teleop
uv run python - <<'PY'
import time
from teleimager.image_client import ImageClient
c = ImageClient(host="192.168.123.164", request_bgr=True)
print(c.get_cam_config()["head_camera"])
time.sleep(0.2)
img = c.get_head_frame()
print(None if img is None or img.bgr is None else img.bgr.shape)
c.close()
PY
```

Normal output should include `(480, 640, 3)`.

## 4. Teleop Startup Commands

### 4.1 Recommended: Upper-body teleop while the lower body remains under Unitree's built-in balancing control

This mode is suitable for tabletop manipulation: the robot's lower body remains standing and balanced in Regular/AI mode, and the Quest does not control locomotion.

Before starting, first confirm that the robot is in Regular/AI mode, meaning Unitree's low-level motion control is still responsible for lower-body balance. Do not use Damping mode, and do not use Debug lowcmd mode by starting without `--motion`.

Key points:

- Use `--motion --upper-body-only`: the upper body is controlled through the arm SDK, while the lower body remains standing under Unitree's built-in controller.
- Use `--motion` without `--upper-body-only`: the lower body is still balanced by Unitree's built-in controller, but velocity commands come from the Quest controller.
- Do not use `--motion`: the program calls `Enter_Debug_Mode()`, entering the Debug/SDK lowcmd path. This is not the desired "lower body automatically remains standing" mode.

Then run on the Host:

```bash
cd /home/zc1525/xr_teleoperate/teleop
uv run python teleop_hand_and_arm.py \
  --arm=G1_29 \
  --ee=inspire_ftp \
  --input-mode=controller \
  --img-server-ip=192.168.123.164 \
  --network-interface=enx207bd2c81d8d \
  --motion \
  --upper-body-only
```

Explanation:

- `--motion`: controls the upper limbs through the Unitree arm SDK channel, avoiding whole-body Debug lowcmd mode.
- `--upper-body-only`: does not send lower-body velocity commands, allowing Unitree's built-in motion/balance module to keep the robot standing.
- After exit, the program attempts to switch back to AI/remote mode so the physical remote controller can be used again.

### 4.2 Quest controller locomotion + upper-body teleop

If you want to control robot locomotion using the Quest controller, remove `--upper-body-only`:

```bash
cd /home/zc1525/xr_teleoperate/teleop
uv run python teleop_hand_and_arm.py \
  --arm=G1_29 \
  --ee=inspire_ftp \
  --input-mode=controller \
  --img-server-ip=192.168.123.164 \
  --network-interface=enx207bd2c81d8d \
  --motion
```

Locomotion mapping:

- Left joystick forward/backward: `vx`
- Left joystick left/right: `vy`
- Right joystick left/right: `yaw`
- Maximum speed scale: `0.3`
- Press both thumbsticks simultaneously: send `Damp()`

Note: this locomotion is not controlled with the physical Unitree remote. Instead, the Quest controller sends commands to the robot through `LocoClientWrapper.Move(vx, vy, vyaw)`.

## 5. Quest 3S Connection Sequence

Recommended sequence:

1. Start `teleop_hand_and_arm.py` first.
2. Open the Vuer page in the Quest browser:

   ```text
   https://vuer.ai?grid=False
   ```

   Alternatively, use the `Visit: ...` address printed in the terminal.

3. Wait until the page shows `WebSocket connected`.
4. Select **Virtual Reality** and allow VR permissions.
5. The operator and the controllers should face toward the robot's forward direction, aligned as closely as possible with the robot's initial orientation.
6. Return to the Host terminal and press `r` to start.
7. Press `q` to exit.

Why enter VR before pressing `r`:

- After `r` is pressed, the program samples the controllers' current poses and anchors them to the robot's current wrist poses.
- If the VR session's world-coordinate orientation is incorrect, the robot's hands may twist toward the operator's real-world standing position.
- The current code uses relative controller wrist poses by default. You can return to the old absolute wrist-pose behavior with `--absolute-wrist-pose`, but this is not recommended.

After pressing `r`, the expected terminal log sequence is:

```text
Pre-teleop safety sequence: raise arms to safety pose before opening hands.
Arm safety pose reached.
Starting hand controller after arms reached the safety pose.
[Inspire safety] open both hands.
---------------------start Tracking-------------------------
```

If you have only started the program and entered VR but have not yet pressed `r`, the hands will not open. This is the current safety logic: during power-on and the waiting stage, the hands remain closed by default so the fingers do not open before the robot has raised its arms.

## 6. Inspire FTP Hand Open/Close Logic

The current hands are not Dex3 and do not use the DFX bridge. They are Inspire FTP hands controlled directly over ModbusTCP:

- Left hand: `192.168.123.210:6000`
- Right hand: `192.168.123.211:6000`
- Register `1486`: angle command
- Register `1522`: speed
- Register `1004`: clear error

Binary actions:

- Controller trigger released: open
- Controller trigger pressed: close
- Left trigger controls the left hand
- Right trigger controls the right hand

Current target values:

```text
open  = [1, 1, 1, 1, 1, 1]  -> [1000, 1000, 1000, 1000, 1000, 1000]
close = [0, 0, 0, 0, 0, 1]  -> [0, 0, 0, 0, 0, 1000]
```

These are position state / angle commands, not acceleration values. After contacting an object, the hand will continue trying to reach the close target. There is currently no tactile- or force-feedback-based automatic slowdown or automatic hold logic.

Hand safety sequence:

- After power-on: the hands should remain closed to protect the fingers.
- While the program is waiting for `r`: the hand controller is not started by default, so the hands will not open early.
- After pressing `r`: first raise both arms to a safe pose, then start the hand controller and open the hands.
- After pressing `q`: first hold the current pose, return to the safe pose, close the hands, slowly return to the startup pose, and finally release the arm SDK.

Do not add the following option to the normal teleop startup command:

```bash
--disable-hand-safety-sequence
```

This option skips the current safety procedure of "raise the arms before opening the hands, and close the hands first during shutdown."

To manually close both hands:

```bash
cd /home/zc1525/xr_teleoperate
uv run python teleop/utils/inspire_ftp_close_hands.py
```

If the hands do not respond, check in this order:

```bash
ping 192.168.123.210
ping 192.168.123.211
nc -vz 192.168.123.210 6000
nc -vz 192.168.123.211 6000
cd /home/zc1525/xr_teleoperate
uv run python teleop/utils/inspire_ftp_close_hands.py
```

## 7. Exit Safety Logic

The current `q` shutdown sequence has been changed to prevent the arms from suddenly dropping:

1. Hold the current teleop arm pose, default `0.8s`.
2. Return slowly to the safety pose, using default velocity `--arm-safety-velocity=0.8`.
3. Hold the safety pose, default `1.0s`.
4. Close both hands.
5. With `--motion`, use the SDK to slowly return the arms to the standing arm pose recorded when the program started, using default `--exit-lower-velocity=0.35`.
6. Hold the final pose, default `1.0s`.
7. Slowly release the arm SDK over `--exit-release-duration=12.0`.
8. Switch back to AI/remote mode.

If lowering is still too fast, add this when starting:

```bash
--exit-lower-velocity=0.2 --exit-release-duration=18
```

If you only want to debug and do not want the arms to return to the startup pose when exiting:

```bash
--disable-exit-lower
```

## 8. Data Recording

Add `--record` and task information when starting:

```bash
cd /home/zc1525/xr_teleoperate/teleop
uv run python teleop_hand_and_arm.py \
  --arm=G1_29 \
  --ee=inspire_ftp \
  --input-mode=controller \
  --img-server-ip=192.168.123.164 \
  --network-interface=eno1 \
  --motion \
  --upper-body-only \
  --record \
  --task-dir=/home/zc1525/xr_teleoperate/records \
  --task-name=g1_inspire_teleop_test \
  --task-goal="teleoperate G1 with Inspire FTP hand" \
  --task-desc="Quest controller teleoperation with RGB observation" \
  --task-steps="start teleop; manipulate object; save episode"
```

Recording workflow:

- Press `r` to start teleop.
- Press `s` to start recording.
- Press `s` again to save the current episode.
- Press `q` to exit.

By default, data is saved to:

```text
/home/zc1525/xr_teleoperate/records/<task-name>/episode_0000/
```

Currently recorded data:

- `colors/color_0`: head RGB, 30 Hz, `640x480`
- `states.left_arm/right_arm.qpos`: current left/right arm joint states
- `actions.left_arm/right_arm.qpos`: IK output target joints
- `states.left_ee/right_ee.qpos`: hand state arrays
- `actions.left_ee/right_ee.qpos`: hand open/close actions
- In full locomotion mode: `actions.body.qpos = [vx, vy, vyaw]`
- In full locomotion mode: `states.body.qpos` is the 35-dimensional whole-body motor qpos

Currently not recorded:

- depth
- wrist camera
- torque
- qvel
- raw Quest pose
- tactile data

Recording frequency:

```text
--frequency defaults to 30.0 Hz
1 step = 1 / 30 = 0.033333 s
```

## 9. Common Issues

### 9.1 CycloneDDS interface error at startup

Confirm the network interface name:

```bash
ip addr
```

`eno` is incorrect. The current Host should use:

```bash
--network-interface=eno1
```

If you see:

```text
/tmp/cdds.LOG: cannot open for writing
```

this is generally not the main cause. The real problem is usually an incorrect DDS interface selection or a permissions/environment issue.

### 9.2 Black screen in VR after starting RGB

First confirm that PC2 teleimager is outputting the OpenCV head camera:

```bash
ssh unitree@192.168.123.164
tail -50 /tmp/teleimager.log
```

Then use the Host-side Python verification from Section 3 and confirm that it can retrieve an image with shape `(480, 640, 3)`.

### 9.3 Arms twist toward where the operator is standing immediately after pressing `r`

The usual cause is that the Quest/OpenXR world-coordinate orientation and the robot coordinate orientation are not aligned. When entering Virtual Reality, the operator and controllers should face the robot's forward direction, and only then should `r` be pressed.

The current code anchors the initial controller poses to the robot's current wrist poses, which is already more stable than the old absolute-pose method.

### 9.4 Hands do not move

The current hands are FTP hands, so there is no need to run a DFX bridge on PC2. First check the network and ports:

```bash
ping 192.168.123.210
ping 192.168.123.211
nc -vz 192.168.123.210 6000
nc -vz 192.168.123.211 6000
```

Then test manual closing:

```bash
cd /home/zc1525/xr_teleoperate
uv run python teleop/utils/inspire_ftp_close_hands.py
```

If the hands do not open after pressing `r`, first check whether the terminal contains these logs:

```text
Starting hand controller after arms reached the safety pose.
[Inspire_Controller_FTP] Using direct ModbusTCP control ...
[Inspire_Controller_FTP] Connected left FTP hand at 192.168.123.210:6000.
[Inspire_Controller_FTP] Connected right FTP hand at 192.168.123.211:6000.
[Inspire safety] open both hands.
```

If there is no `Connected ... FTP hand` message, the problem is with the hands' network, power, or port.

If `open both hands` appears but the hands do not move, use the manual close/open test script or check whether the hands are already in the open state.

### 9.5 Physical remote controller does not recover after exit

After `q` is pressed, the program releases the arm SDK and calls `Exit_Debug_Mode()` to switch back to AI/remote mode. Wait for these logs:

```text
release arm sdk mode OK
Switch to AI/remote mode: Success
```

If the physical remote still cannot be used, manually switch the robot back to Regular/AI mode using the Unitree app or remote controller.

## 10. Current Recommended Startup Templates

Daily upper-body teleop:

```bash
cd /home/zc1525/xr_teleoperate/teleop
uv run python teleop_hand_and_arm.py \
  --arm=G1_29 \
  --ee=inspire_ftp \
  --input-mode=controller \
  --img-server-ip=192.168.123.164 \
  --network-interface=eno1 \
  --motion \
  --upper-body-only \
  --exit-lower-velocity=0.2 \
  --exit-release-duration=18
```

When Quest-controller locomotion is required:

```bash
cd /home/zc1525/xr_teleoperate/teleop
uv run python teleop_hand_and_arm.py \
  --arm=G1_29 \
  --ee=inspire_ftp \
  --input-mode=controller \
  --img-server-ip=192.168.123.164 \
  --network-interface=eno1 \
  --motion \
  --exit-lower-velocity=0.2 \
  --exit-release-duration=18
```

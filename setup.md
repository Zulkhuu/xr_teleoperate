# G1 setup and daily operation

Use this procedure for G1-29 upper-body teleoperation with Inspire FTP hands
and Quest controllers. Run host commands from the repository root unless a
step gives a different directory. Use the paths, interface names, camera
identifiers, and addresses for your installation.

## 1. Prepare the host environment

In **each new host terminal**, do these steps before you run a command:

1. Go to the repository directory.
2. Load the G1 environment script with the `source` command.

```bash
cd /path/to/xr_teleoperate
source /path/to/g1_env.sh
```

Use the `g1_env.sh` script for your host. The repository does not supply this
script. The program requires the repository's Python environment and the
Pinocchio/CasADi and DDS libraries.

The `.envrc` file contains robotpkg environment settings. Keep the additional
settings from your `g1_env.sh` script.

To install or update dependencies, run:

```bash
git submodule update --init --recursive
uv sync --locked
```

Before you run `uv sync`, put the Unitree Python SDK in
`../unitree_sdk2_python`. The `pyproject.toml` file uses this location.
The workspace includes `teleimager`, `televuer`, and dex-retargeting.

After you load the environment, check the imports and network interfaces:

```bash
uv run python -c "import pinocchio, pinocchio.casadi, casadi, pymodbus, teleimager, televuer, unitree_sdk2py; print('environment OK')"
ip -br addr
```

Find the Ethernet interface connected to the robot.
Set `ROBOT_INTERFACE` to its name. The interface name can differ between hosts.

```bash
export ROBOT_INTERFACE=eth0  # Replace with your robot-facing interface.
```

## 2. Network and hardware

This configuration uses these robot network addresses:

| Device | Address | Purpose |
| --- | --- | --- |
| G1 PC2 / image server | `192.168.123.164` | TeleImager camera streams |
| G1 robot controller | `192.168.123.161` | Motion and mode services |
| Left Inspire FTP hand | `192.168.123.210:6000` | Direct ModbusTCP |
| Right Inspire FTP hand | `192.168.123.211:6000` | Direct ModbusTCP |

1. Set the host address to an unused address on the robot subnet, for example `192.168.123.222/24`.
2. Check the network connections:

```bash
ping -c 3 192.168.123.164
nc -vz 192.168.123.210 6000
nc -vz 192.168.123.211 6000
```

Use `--arm G1_29 --ee inspire_ftp`. The FTP hands use direct ModbusTCP.
They do not require the Inspire DFX bridge.

## 3. Start and check the image server

1. Connect to PC2.
2. Check if the TeleImager server is running.
3. If the server is stopped, start it.
4. Read the server log.

```bash
ssh unitree@192.168.123.164
pgrep -af teleimager-server
# If no server is running:
nohup ~/.local/bin/teleimager-server > /tmp/teleimager.log 2>&1 &
tail -f /tmp/teleimager.log
```

Check the camera configuration on the server.
The repository file `teleop/teleimager/cam_config_server.yaml` specifies:

- UVC binocular head video: `[480, 1280]`, 30 FPS.
- UVC left and right wrist video: `[480, 640]`, 30 FPS.
- ZMQ enabled for all three cameras; head ZMQ is required for the camera HUD.

Set the camera serial numbers or device IDs for your hardware.
The server configuration can differ from the repository file.
After you change the server configuration, restart the server.

In a host terminal, load `g1_env.sh`. Then check the camera stream:

```bash
uv run python - <<'PY'
import time
from teleimager.image_client import ImageClient
client = ImageClient(host="192.168.123.164", request_bgr=True)
try:
    print(client.get_cam_config()["head_camera"])
    time.sleep(1)
    frame = client.get_head_frame()
    print(None if frame is None or frame.bgr is None else frame.bgr.shape)
finally:
    client.close()
PY
```

The repository stereo configuration produces head frames with shape
`(480, 1280, 3)`. Check the configuration that the server returns.

## 4. Connect Quest Browser over Wi-Fi

Regular Vuer over Wi-Fi is the main connection method.

1. Connect the Quest and host to the same Wi-Fi network.
2. Configure the Vuer TLS certificate and key for the host address.
   Use `XR_TELEOP_CERT` and `XR_TELEOP_KEY`, or
   `~/.config/xr_teleoperate/cert.pem` and `~/.config/xr_teleoperate/key.pem`.
3. Start teleoperation with `--xr vuer`. Use the command in section 5.
4. Open the visit address shown in the terminal in Quest Browser.
5. Accept the certificate if necessary.
6. Enter VR.
7. Give permission for tracking.

The host uses the robot network for DDS and camera streams.

USB is an optional connection method under test. To test it, use
`--xr vuer_usb` with an authorized USB headset and a certificate for `localhost`.
See [XR connection details](docs/xr_connections.md) for the USB procedure.

## 5. Launch upper-body teleoperation

1. Set the robot to **Regular/AI mode**. The robot controller must keep the robot balanced.
2. Clear the workspace. At startup, the program automatically moves the arms to the safety pose and opens the hands.
3. Open a host terminal.
4. Load the environment and start the program:

```bash
cd /path/to/xr_teleoperate
source /path/to/g1_env.sh
export ROBOT_INTERFACE=eth0  # Replace with your robot-facing interface.
uv run --directory teleop python teleop_hand_and_arm.py \
  --xr vuer \
  --arm G1_29 \
  --ee inspire_ftp \
  --input-mode controller \
  --img-server-ip 192.168.123.164 \
  --network-interface "$ROBOT_INTERFACE" \
  --motion \
  --upper-body-only
```

The options `--motion --upper-body-only` select arm SDK control.
The program does not send locomotion velocity commands in this mode.
Without `--motion`, the program selects Debug/lowcmd control.

To control locomotion with Quest, remove `--upper-body-only`.
The left joystick controls forward and lateral velocity.
The right joystick controls yaw. These commands operate only in the ACTIVE state.

5. Wait for arm preparation to finish.
6. Make sure tracking is valid.
7. Face the forward direction of the robot.
8. Hold the controllers in a comfortable position.
9. Follow the instructions on the head-up display (HUD).

| Controller action | Result |
| --- | --- |
| Hold left side grip | Set a new reference and follow controller movement |
| Release left grip | Pause and freeze arms and fingers |
| Hold X for 0.5 seconds | Prepare the arms if necessary. In the DAMPED state, acknowledge recovery only |
| B | Unused in controller mode |
| Tap Y, then release within 0.7 seconds | Start or stop recording in the ACTIVE state with `--record` enabled |
| Press A | Pause immediately |
| Hold A for 1.5 seconds while paused | Start the normal shutdown sequence |
| Press L3 + R3 | Latch application damping |

Release the left grip before you press it again.
Each new press sets the current robot pose as the reference for controller movement.
Tracking loss or an invalid inverse kinematics (IK) result pauses motion.
To continue, release the left grip and press it again.

After application damping, do these steps:

1. Check the robot.
2. Restore the robot operating mode with the external robot controls.
3. Hold X for 0.5 seconds to acknowledge recovery.
4. If preparation is incomplete, release X. Hold X again for 0.5 seconds to prepare the arms.
5. Release the left grip.
6. Press the left grip to follow controller movement.

Application damping does not replace the physical emergency stop.

Use these keyboard commands:

| Key | Function |
| --- | --- |
| `p` | Pause |
| `s` | Start or stop recording |
| `q` | Start shutdown |
| `r` | Request preparation in controller mode |
| `b` | No function in controller mode |

For `--input-mode hand`, do these steps:

1. Press `r` to prepare the arms.
2. Hold your hands still.
3. Press `b` to set the reference.
4. Press `r` again to follow hand movement.

In hand mode, following stays active until a pause or interruption occurs.

## 6. Camera display and hand behavior

The default head view is 80% of full size and is centered in each eye.
For a full-size image, use `--xr-image-scale 1 --xr-image-offset-y 0`.

The right wrist camera image appears automatically inside the head view.
Its width is 28% of the head view width.
It moves and changes size with the head image.
The program uses the server's right wrist ZMQ stream if it is enabled.
Otherwise, the program can use a local RealSense camera.
To select a local camera, use `--right-wrist-d405-serial SERIAL`.

The HUD and camera overlay do not change recorded camera pixels.
Pass-through mode has no camera HUD.

During following, each controller trigger opens or closes the related FTP hand.
Release the left grip to hold both arms and fingers in position.
At startup, the program raises the arms before it starts hand control and opens the hands.
Do not use `--disable-hand-safety-sequence` for normal operation.

To close the hands manually:

1. Go to the repository root.
2. Load `g1_env.sh`.
3. Run this command:

```bash
uv run python teleop/utils/inspire_ftp_close_hands.py
```

## 7. Recording

To record local episodes, add these options to the start command:

```bash
--record --recording-backend episode \
--task-dir ../records \
--task-name g1_inspire_teleop \
--task-goal "Manipulate an object with Inspire FTP hands"
```

With `--directory teleop`, the path `../records` selects the repository's
`records` directory.

1. Hold the left grip to follow controller movement.
2. Tap Y to start recording.
3. Keep the left grip pressed during the episode.
4. Tap Y again to save the episode.

A pause, grip release, tracking loss, invalid IK result, damping, or exit ends
and saves the active episode.

The episode recorder saves these data:

- Raw head images, with a separate image for each eye in stereo mode.
- Wrist images from enabled ZMQ streams.
- Arm states and targets.
- Hand states and actions.

Body data depends on the locomotion mode. The default sample rate is 30 Hz.
The episode recorder does not save depth, torque, raw Quest poses, or tactile data.

To use Neuracore:

1. Go to the repository root.
2. Load `g1_env.sh`.
3. Install the separate recording environment:

```bash
uv sync --project integrations/neuracore --python 3.10
```

4. Add these options to the teleoperation command:

```bash
--record --recording-backend neuracore --neuracore-dataset "G1 Teleoperation"
```

Neuracore login runs in the terminal before robot initialization and arm preparation.
Neuracore requires head ZMQ. Enable wrist ZMQ to record wrist images.
Neuracore records all 29 measured G1 joint positions and velocities.
It also records the 14 arm position targets.
These data are also recorded in upper-body-only mode.
See the [recording documentation](README.md#recording-backends) for more information.

## 8. Shutdown and troubleshooting

To start normal shutdown, hold A for 1.5 seconds while paused, or press keyboard `q`.
Wait for the shutdown sequence to finish.

During normal shutdown, the program does these steps:

1. Holds the current pose.
2. Moves the arms to the safety pose.
3. Closes the hands.
4. Lowers the arms to the startup pose in motion mode.
5. Releases arm SDK control.
6. Tries to restore AI/remote mode.

After damping, shutdown does not move the arms home, release SDK control, or switch the robot mode.

To decrease the lowering speed and increase the release time, add:

```bash
--exit-lower-velocity 0.2 --exit-release-duration 18
```

| Problem | Action |
| --- | --- |
| Import or shared-library error | Load `g1_env.sh` in the same terminal. Repeat the import check in section 1. |
| DDS initialization error | Run `ip -br addr`. Check the robot connection and `--network-interface`. |
| Black headset video | Read `/tmp/teleimager.log` on PC2. Check head ZMQ. Run the image check in section 3. |
| USB video connection error | Check ADB authorization, the localhost URL, and certificate trust. |
| Incorrect controller alignment | Release the grip. Face forward. Position the controllers. Press the grip to set a new reference. |
| Hands do not move | Check both ModbusTCP connections and the preparation logs. Hand control starts after the arms reach the safety pose. |
| Physical remote does not operate after exit | Wait for SDK release and the AI/remote mode change. Use external robot controls to restore Regular/AI mode if necessary. |

For the optional USB test, check tracking and video without robot commands:

1. Go to the repository root.
2. Load `g1_env.sh`.
3. Run this command:

```bash
uv run python -m teleop.check_xr --stereo
```

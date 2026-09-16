# Quest Browser over USB

Use `--xr vuer_usb` for both tracking and robot-camera video through a USB
cable. `--xr meta_quest` is a compatibility alias for the same browser mode;
it no longer installs or starts a native APK. `--xr vuer` retains the existing
network mode.

## Data flow

```text
Robot camera -> host ImageClient (ZMQ) -> Vuer image stream -> USB -> Quest Browser
Robot control <- host teleop loop     <- Vuer tracking     <- USB <- Quest Browser
```

The host still communicates with the robot/image server over its normal robot
network (preferably Ethernet). The headset-to-host tracking and camera-image
connection uses an ADB reverse tunnel for TCP port 8012. USB mode explicitly
disables direct WebRTC to the image server, even if its configuration enables
WebRTC, so the displayed video does not silently travel over Wi-Fi.

## Setup and launch

1. Enable Quest developer mode, connect a USB data cable, and accept USB
   debugging in the headset. Install Android platform-tools (`adb`) on the host.
2. Check `adb devices -l`. The headset must have state `device` and a `usb:`
   transport entry. Use `--quest-serial SERIAL` with multiple USB devices.
3. Set `head_camera.enable_zmq: true` on the image server and restart it if
   necessary. Immersive and ego USB display modes require this; a WebRTC-only
   configuration is rejected.
4. Keep the existing Vuer TLS setup: `XR_TELEOP_CERT` and `XR_TELEOP_KEY`, or
   `~/.config/xr_teleoperate/cert.pem` and `key.pem`. The certificate should cover
   `localhost` and be accepted by Quest Browser.
5. From the repository root, load the robotpkg environment and launch:

```bash
source .envrc
uv run --directory teleop python teleop_hand_and_arm.py \
  --xr vuer_usb --input-mode controller --arm G1_29 --ee inspire_ftp \
  --motion --upper-body-only
```

6. In **Quest Browser**, open:

```text
https://localhost:8012/?ws=wss://localhost:8012&grid=False
```

Accept/trust the certificate, enter VR, and grant tracking permissions. Confirm
camera video and active tracking before pressing `r` to start robot following.
Press `q` to stop. No separate headset APK is needed.

USB defaults to controller input and immersive camera display. Use
`--input-mode hand` for hand tracking, `--display-mode ego` for the smaller
camera view, or `--display-mode pass-through` for no robot-camera display.
Monocular/binocular rendering follows the head-camera configuration.
`--record` preserves normal host-side recording. `--adb-path` selects a custom
ADB executable. Network Vuer retains its hand/immersive defaults.

Vuer's frontend may need internet access to load its assets initially; this
change does not bundle an offline frontend. That is separate from the USB
tracking/video stream.

## Reliability and latency

The application selects the USB device explicitly, preserves existing matching
tunnels, refuses conflicting mappings, and removes only its owned mapping on
cleanup. Background ADB checks detect tunnel/device loss without blocking the
robot loop. Fresh head and hand/controller events are required before startup
and during tracking (`--xr-tracking-timeout`, default 0.5 seconds). Loss triggers
the existing robot shutdown sequence. These checks are not a hardware interlock.

USB avoids dependence on Wi-Fi for headset streaming, but is not latency-free:
camera capture, host image processing, compression, USB/TCP buffering and browser
rendering still contribute. Start with moderate resolution and 30 fps; measure
latency on the actual setup before increasing them. The existing Vuer image
writer keeps its latest frame rather than a growing image queue, but this does
not eliminate buffering elsewhere. Freshness timestamps measure host receipt,
not device capture time.

## Validation

```bash
uv run python -m unittest discover -s tests -v
```

Hardware-free tests cover forced USB image routing, network WebRTC preservation,
hand/controller configuration, tunnel ownership/conflicts, disconnect checks,
tracking freshness, and a mocked entry-point run rendering camera frames while
reaching arm IK. Physical Quest video, certificate acceptance, cable loss and
end-to-end latency still need testing on the headset. No robot movement or
headset installation was performed by these checks.

The local televuer submodule includes event timestamps used by the watchdog;
preserve that submodule change when committing/deploying the refactor.

## Check USB without a G1 or simulator

From the repository root:

```bash
uv run python -m teleop.check_xr --stereo
```

This diagnostic uses the same XRSession and USB tunnel as teleoperation but
never imports robot controllers or initializes DDS/IK. It generates an animated
image locally, so no robot or image server is needed. Keep the TLS certificate
setup described above, open the same localhost URL in Quest Browser, and enter
VR. Look for a moving green dot, an advancing host timer, and the correct
LEFT/RIGHT EYE labels. The terminal reports TRACKING OK, wrist positions,
triggers, and thumbsticks when fresh tracking arrives. Exit with Ctrl+C.

Use `--input-mode hand` to check hand tracking, `--quest-serial SERIAL` to select
a headset, or `--duration 60` for a timed check. The exit status is 1 if no valid
tracking was observed; displayed-video success must be checked visually.
`--img-server-ip ADDRESS` uses a running image server instead of generated frames.
This checks USB, browser rendering and tracking, not robot IK or actuation.
`--sim` on the full teleop entry point still needs a simulator publishing robot
state; it is not a standalone headset test.

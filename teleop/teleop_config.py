"""Command-line options for the teleoperation entry point."""

import argparse


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--xr', choices=['vuer', 'vuer_usb', 'meta_quest'], default='vuer',
                        help='XR connection: vuer (network), vuer_usb (Quest Browser over USB); meta_quest is an alias for vuer_usb.')
    parser.add_argument('--quest-serial', default=None, help='USB Quest serial from adb devices -l; required if multiple USB devices are attached.')
    parser.add_argument('--adb-path', default='adb', help='ADB executable for USB mode.')
    parser.add_argument('--xr-tracking-timeout', type=float, default=0.5,
                        help='Maximum tracking age in seconds before stopping teleoperation.')
    parser.add_argument('--xr-image-scale', type=float, default=1.0,
                        help='Per-eye camera image scale inside the XR display (0 < scale <= 1; default: 1, full size).')
    parser.add_argument('--xr-image-offset-y', type=float, default=0.0,
                        help='Vertical camera-image offset as a fraction of eye height; positive moves down (default: 0). Clamped to keep the full image visible.')
    # basic control parameters
    parser.add_argument('--frequency', type = float, default = 30.0, help = 'control and record \'s frequency')
    parser.add_argument('--input-mode', type=str, choices=['hand', 'controller'], default=None, help='XR input (default: hand for network, controller for USB)')
    parser.add_argument('--display-mode', type=str, choices=['immersive', 'ego', 'pass-through'], default=None, help='XR display (default: immersive)')
    parser.add_argument('--arm', type=str, choices=['G1_29', 'G1_23', 'H1_2', 'H1', 'H2'], default='G1_29', help='Select arm controller')
    parser.add_argument('--ee', type=str, choices=['dex1', 'dex3', 'inspire_ftp', 'inspire_dfx', 'brainco'], help='Select end effector controller')
    parser.add_argument('--img-server-ip', type=str, default='192.168.123.164', help='IP address of image server, used by teleimager and televuer')
    parser.add_argument('--network-interface', type=str, default=None, help='Network interface for dds communication, e.g., eth0, wlan0. If None, use default interface.')
    # mode flags
    parser.add_argument('--motion', action = 'store_true', help = 'Enable motion control mode')
    parser.add_argument('--upper-body-only', action='store_true',
                        help='In motion mode, do not send loco commands; keep lower body under the robot motion controller.')
    parser.add_argument('--absolute-wrist-pose', action='store_true',
                        help='Use raw absolute XR wrist poses instead of anchoring teleop to the robot pose at start.')
    parser.add_argument('--disable-hand-safety-sequence', action='store_true',
                        help='Disable the hand protection sequence: defer hand control until pre-teleop open, then exit close.')
    parser.add_argument('--hand-safety-settle', type=float, default=0.8,
                        help='Seconds to wait after hand open/close safety commands.')
    parser.add_argument('--arm-safety-velocity', type=float, default=0.8,
                        help='Arm joint velocity limit used by the pre-teleop and exit safety pose.')
    parser.add_argument('--arm-safety-timeout', type=float, default=15.0,
                        help='Seconds to wait for the arm safety pose.')
    parser.add_argument('--exit-initial-hold', type=float, default=0.8,
                        help='Seconds to hold the current arm pose immediately after exit is requested.')
    parser.add_argument('--exit-safety-hold', type=float, default=1.0,
                        help='Seconds to hold the arm safety pose before closing hands.')
    parser.add_argument('--exit-lower-velocity', type=float, default=0.35,
                        help='Arm joint velocity limit used when lowering arms before releasing SDK control.')
    parser.add_argument('--exit-lower-timeout', type=float, default=40.0,
                        help='Seconds to wait when lowering arms before releasing SDK control.')
    parser.add_argument('--exit-hold-before-release', type=float, default=1.0,
                        help='Seconds to hold the final exit arm pose before releasing SDK control.')
    parser.add_argument('--exit-release-duration', type=float, default=12.0,
                        help='Seconds used to release arm SDK control back to the robot motion controller.')
    parser.add_argument('--disable-exit-lower', action='store_true',
                        help='Disable controlled arm lowering before releasing SDK control in motion mode.')
    parser.add_argument('--headless', action='store_true', help='Enable headless mode (no display)')
    parser.add_argument('--sim', action = 'store_true', help = 'Enable isaac simulation mode')
    parser.add_argument('--ipc', action = 'store_true', help = 'Enable IPC server to handle input; otherwise enable sshkeyboard')
    parser.add_argument('--affinity', action = 'store_true', help = 'Enable high priority and set CPU affinity mode')
    # record mode and task info
    parser.add_argument('--record', action = 'store_true', help = 'Enable data recording mode')
    parser.add_argument('--recording-backend', choices=['episode', 'neuracore'], default='episode',
                        help='Recorder used with --record: episode (existing local format) or neuracore.')
    from pathlib import Path
    parser.add_argument('--neuracore-python', default=str(Path(__file__).resolve().parents[1] /
                        'integrations/neuracore/.venv/bin/python'),
                        help='Python executable in the isolated Neuracore environment.')
    parser.add_argument('--neuracore-dataset', default=None, help='Neuracore dataset name (defaults to --task-name).')
    parser.add_argument('--neuracore-upload-timeout', type=float, default=300.,
                        help='Neuracore robot model upload timeout in seconds.')
    parser.add_argument('--task-dir', type = str, default = './utils/data/', help = 'path to save data')
    parser.add_argument('--task-name', type = str, default = 'pick cube', help = 'task file name for recording')
    parser.add_argument('--task-goal', type = str, default = 'pick up cube.', help = 'task goal for recording at json file')
    parser.add_argument('--task-desc', type = str, default = 'task description', help = 'task description for recording at json file')
    parser.add_argument('--task-steps', type = str, default = 'step1: do this; step2: do that;', help = 'task steps for recording at json file')

    args = parser.parse_args(argv)
    if args.recording_backend == 'neuracore' and (args.arm != 'G1_29' or args.ee != 'inspire_ftp'):
        parser.error('Neuracore robot model requires --arm G1_29 --ee inspire_ftp')
    if args.recording_backend == 'neuracore' and not args.record:
        parser.error('--recording-backend neuracore requires --record')
    if args.input_mode is None:
        args.input_mode = 'hand' if args.xr == 'vuer' else 'controller'
    if args.display_mode is None:
        args.display_mode = 'immersive'
    import math
    if not math.isfinite(args.xr_image_scale) or not 0 < args.xr_image_scale <= 1:
        parser.error('--xr-image-scale must be finite and in (0, 1]')
    if not math.isfinite(args.xr_image_offset_y) or not -0.5 <= args.xr_image_offset_y <= 0.5:
        parser.error('--xr-image-offset-y must be finite and in [-0.5, 0.5]')
    for name in ('frequency', 'xr_tracking_timeout', 'neuracore_upload_timeout'):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f'--{name.replace("_", "-")} must be finite and positive')
    return args

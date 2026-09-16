#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mode=shell
gpu=0
gui=0
usb=0

usage() {
    cat <<'HELP'
Usage:
  docker/run.sh [--gpu] [--gui] [--usb] --shell
  docker/run.sh --test
  docker/run.sh [--gpu] [--gui] [--usb] -- [teleop arguments]

No arguments opens a shell; teleoperation never starts by default.
--test runs the hardware-free unit tests without host networking or mounts.
--gpu requires NVIDIA Container Toolkit on the host.
--gui shares the host X11 display and its existing Xauthority cookie.
--usb exposes USB devices for Quest/ADB; host udev permissions still apply.

Environment:
  XR_DOCKER_IMAGE  Image tag (default: xr-teleoperate:latest)
  XR_RECORDS_DIR   Host recordings directory (default: ~/xr_teleoperate/records)
  XR_NEURACORE_DIR Host Neuracore config/cache (default: ~/.neuracore)
  XR_CERT_DIR      Directory with cert.pem/key.pem (default: ~/.config/xr_teleoperate,
                   falling back to this checkout's teleop/televuer directory)

Example:
  docker/run.sh -- --arm=G1_29 --ee=inspire_ftp --input-mode=controller \
    --img-server-ip=192.168.123.164 --network-interface=enx207bd2c81d8d \
    --xr=vuer --motion --upper-body-only --headless --record \
    --recording-backend=neuracore --neuracore-dataset=g1_inspire_teleop_test

For local recordings use --record --recording-backend=episode --task-dir=/records.
For an interactive Python command inside the shell, the teleop venv is on PATH;
Neuracore uses its own interpreter automatically. Source is baked into the image:
rebuild after changing code. Exit teleoperation with A-hold/q before stopping the
container; Docker's SIGINT is an interruption, not the normal arm-home sequence.
HELP
}

while (($#)); do
    case "$1" in
        --help|-h) usage; exit 0 ;;
        --gpu) gpu=1; shift ;;
        --gui) gui=1; shift ;;
        --usb) usb=1; shift ;;
        --shell) mode=shell; shift ;;
        --test) mode=test; shift ;;
        --) mode=teleop; shift; break ;;
        *) printf 'Unknown wrapper option: %s; put teleop arguments after --.\n' "$1" >&2; exit 2 ;;
    esac
done

if [[ "$(uname -s)" != Linux ]]; then
    printf 'This runner targets a native Linux Docker host for Unitree DDS networking.\n' >&2
    exit 1
fi

options=(--rm --init --stop-timeout 180 --shm-size 1g --user "$(id -u):$(id -g)")
for gid in $(id -G); do options+=(--group-add "$gid"); done
if [[ -t 0 && -t 1 ]]; then options+=(-it); fi
if [[ "$mode" == test ]]; then
    exec docker run "${options[@]}" --network none --workdir /opt/xr_teleoperate \
        "${XR_DOCKER_IMAGE:-xr-teleoperate:latest}" python -m unittest discover -s tests
fi

options+=(--network host)
records_dir="${XR_RECORDS_DIR:-$HOME/xr_teleoperate/records}"
neuracore_dir="${XR_NEURACORE_DIR:-$HOME/.neuracore}"
mkdir -p -- "$records_dir" "$neuracore_dir"
records_dir="$(cd -- "$records_dir" && pwd)"
neuracore_dir="$(cd -- "$neuracore_dir" && pwd)"
options+=(--mount "type=bind,src=$records_dir,dst=/records"
          --mount "type=bind,src=$neuracore_dir,dst=/home/teleop/.neuracore"
          --env HOME=/home/teleop)

cert_dir="${XR_CERT_DIR:-$HOME/.config/xr_teleoperate}"
if [[ -z "${XR_CERT_DIR:-}" && ! -f "$cert_dir/cert.pem" ]]; then
    cert_dir="$repo_root/teleop/televuer"
fi
if [[ -f "$cert_dir/cert.pem" && -f "$cert_dir/key.pem" ]]; then
    cert_dir="$(cd -- "$cert_dir" && pwd)"
    options+=(--mount "type=bind,src=$cert_dir/cert.pem,dst=/certs/cert.pem,readonly"
              --mount "type=bind,src=$cert_dir/key.pem,dst=/certs/key.pem,readonly"
              --env XR_TELEOP_CERT=/certs/cert.pem --env XR_TELEOP_KEY=/certs/key.pem)
elif [[ "$mode" == teleop ]]; then
    printf 'Set XR_CERT_DIR to a directory containing the Quest-trusted cert.pem and key.pem.\n' >&2
    exit 1
fi

if ((gpu)); then options+=(--gpus all); fi
if ((gui)); then
    : "${DISPLAY:?Set DISPLAY for --gui}"
    xauthority="${XAUTHORITY:-$HOME/.Xauthority}"
    [[ -f "$xauthority" ]] || { printf 'An Xauthority cookie is required for --gui.\n' >&2; exit 1; }
    options+=(--env DISPLAY --env XAUTHORITY=/tmp/container.xauthority
              --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly
              --mount "type=bind,src=$xauthority,dst=/tmp/container.xauthority,readonly")
fi
if ((usb)); then
    [[ -d /dev/bus/usb ]] || { printf 'No USB bus found at /dev/bus/usb.\n' >&2; exit 1; }
    options+=(--mount type=bind,src=/dev/bus/usb,dst=/dev/bus/usb --device-cgroup-rule 'c 189:* rwm')
fi

command=(/bin/bash)
if [[ "$mode" == teleop ]]; then command=(python teleop_hand_and_arm.py "$@"); fi
exec docker run "${options[@]}" "${XR_DOCKER_IMAGE:-xr-teleoperate:latest}" "${command[@]}"

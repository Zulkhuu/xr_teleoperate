#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "${1:-}" == "--help" ]]; then
    cat <<'HELP'
Usage: docker/build.sh [extra docker build options]

Builds xr-teleoperate:latest using the repository root as the build context.
Environment:
  XR_DOCKER_IMAGE   Image tag (default: xr-teleoperate:latest)
  XR_DOCKER_UID     Container user UID (default: host UID, or 1000 for root)
  XR_DOCKER_GID     Container user GID (default: host GID, or 1000 for root)

Example: docker/build.sh --no-cache
Submodule working copies must already be populated. Current local source changes
are included, while host environments, recordings, and secrets are excluded.
HELP
    exit 0
fi

for relative in teleop/televuer teleop/teleimager teleop/robot_control/dex-retargeting; do
    if [[ ! -f "$repo_root/$relative/pyproject.toml" ]]; then
        printf 'Missing submodule files: %s. Populate submodules before building.\n' "$relative" >&2
        exit 1
    fi
done

app_uid="${XR_DOCKER_UID:-$(id -u)}"
app_gid="${XR_DOCKER_GID:-$(id -g)}"
[[ "$app_uid" != 0 ]] || app_uid=1000
[[ "$app_gid" != 0 ]] || app_gid=1000

exec docker build \
    --file "$repo_root/docker/Dockerfile" \
    --tag "${XR_DOCKER_IMAGE:-xr-teleoperate:latest}" \
    --build-arg "APP_UID=$app_uid" \
    --build-arg "APP_GID=$app_gid" \
    "$@" "$repo_root"

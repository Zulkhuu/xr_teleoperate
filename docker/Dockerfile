# syntax=docker/dockerfile:1
# Build from the repository root with docker/build.sh. Both uv environments
# are installed in the image; no host virtual environment is copied.
FROM ghcr.io/astral-sh/uv:0.10.10 AS uv
FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
ARG APP_UID=1000
ARG APP_GID=1000
ARG UNITREE_SDK_REV=65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5
ARG CYCLONEDDS_REF=0.10.2
ARG PINOCCHIO_VERSION=4.1.0
ARG CASADI_VERSION=3.7.2r1

RUN apt-get update && apt-get install -y --no-install-recommends \
    adb build-essential ca-certificates cmake curl git libegl1 libgl1 \
    libglib2.0-0 libgomp1 libsm6 libssl-dev libusb-1.0-0 libxext6 \
    libxrender1 libxcb-xinerama0 libxkbcommon-x11-0 openssl pkg-config \
    python3 python3-dev python3-venv xauth \
    && rm -rf /var/lib/apt/lists/*

# robotpkg ships the CasADi Python bindings missing from the locked PyPI pin
# wheel. Use the same Pinocchio/CasADi pair as the host, including its native
# dependencies. This binary repository targets Ubuntu Jammy on amd64.
RUN test "$(dpkg --print-architecture)" = amd64 \
    && mkdir -p /etc/apt/keyrings \
    && curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
        -o /etc/apt/keyrings/robotpkg.asc \
    && echo '0582476e2e90b3666686e6c50d4d58714e8fddb90786db9f42942baf71e67e68  /etc/apt/keyrings/robotpkg.asc' | sha256sum -c - \
    && echo 'deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub jammy robotpkg' \
        > /etc/apt/sources.list.d/robotpkg.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        "robotpkg-py310-pinocchio=${PINOCCHIO_VERSION}" \
        "robotpkg-py310-casadi=${CASADI_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

COPY --from=uv /uv /uvx /usr/local/bin/
ENV UV_PYTHON_DOWNLOADS=never \
    UV_LINK_MODE=copy \
    PYTHONUNBUFFERED=1 \
    CYCLONEDDS_HOME=/opt/cyclonedds \
    LD_LIBRARY_PATH=/opt/cyclonedds/lib:/opt/openrobots/lib

# Unitree's Python SDK requires the Cyclone DDS C library at build and runtime.
RUN git clone --depth 1 --branch "${CYCLONEDDS_REF}" \
        https://github.com/eclipse-cyclonedds/cyclonedds.git /tmp/cyclonedds-src \
    && cmake -S /tmp/cyclonedds-src -B /tmp/cyclonedds-build \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds \
        -DCMAKE_INSTALL_LIBDIR=lib -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
    && cmake --build /tmp/cyclonedds-build --parallel 2 \
    && cmake --install /tmp/cyclonedds-build \
    && rm -rf /tmp/cyclonedds-src /tmp/cyclonedds-build

# Preserve pyproject.toml's ../unitree_sdk2_python path without changing it.
RUN git init /opt/unitree_sdk2_python \
    && git -C /opt/unitree_sdk2_python remote add origin https://github.com/unitreerobotics/unitree_sdk2_python.git \
    && git -C /opt/unitree_sdk2_python fetch --depth 1 origin "${UNITREE_SDK_REV}" \
    && git -C /opt/unitree_sdk2_python checkout --detach FETCH_HEAD

RUN (getent group "${APP_GID}" || groupadd --gid "${APP_GID}" teleop) \
    && useradd --create-home --uid "${APP_UID}" --gid "${APP_GID}" --shell /bin/bash teleop

WORKDIR /opt/xr_teleoperate
COPY --chown=teleop . .
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --locked --python /usr/bin/python3 \
    && uv sync --locked --project integrations/neuracore --python /usr/bin/python3

# Override the wheel's modules only in the teleop interpreter. A global
# PYTHONPATH would also override Neuracore's isolated dependencies. Keep CasADi,
# eigenpy and Pinocchio from one binary distribution to avoid ABI mismatches.
RUN echo 'import sys; sys.path.insert(0, "/opt/openrobots/lib/python3.10/site-packages")' \
    > .venv/lib/python3.10/site-packages/robotpkg.pth

ENV PATH="/opt/xr_teleoperate/.venv/bin:${PATH}"
USER teleop
WORKDIR /opt/xr_teleoperate/teleop
# Exercise real symbolic kinematics and the application's imports without DDS
# initialization or robot communication; an image with missing bindings fails.
RUN python - <<'PY'
import casadi as cs
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin
from robot_control.robot_arm_ik import G1_29_ArmIK
from dex_retargeting import RetargetingConfig

assert pin.__file__.startswith('/opt/openrobots/'), pin.__file__
assert cs.__file__.startswith('/opt/openrobots/'), cs.__file__
model = pin.buildModelFromUrdf('../assets/g1/g1_body29_hand14.urdf')
data = model.createData()
symbolic_model = cpin.Model(model)
symbolic_data = symbolic_model.createData()
q = cs.SX.sym('q', model.nq)
cpin.framesForwardKinematics(symbolic_model, symbolic_data, q)
fk = cs.Function('fk', [q], [symbolic_data.oMf[-1].translation])
neutral = pin.neutral(model)
pin.framesForwardKinematics(model, data, neutral)
np.testing.assert_allclose(np.asarray(fk(neutral)).ravel(), data.oMf[-1].translation)
print('Pinocchio', pin.__version__, 'CasADi', cs.__version__, 'symbolic FK OK')
PY
# Docker stop should give the app a chance to handle KeyboardInterrupt.
# Request the normal arm exit with Quest A-hold or keyboard q before stopping.
STOPSIGNAL SIGINT
CMD ["/bin/bash"]

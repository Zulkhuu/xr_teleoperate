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

RUN apt-get update && apt-get install -y --no-install-recommends \
    adb build-essential ca-certificates cmake git libegl1 libgl1 \
    libglib2.0-0 libgomp1 libsm6 libssl-dev libusb-1.0-0 libxext6 \
    libxrender1 libxcb-xinerama0 libxkbcommon-x11-0 openssl pkg-config \
    python3 python3-dev python3-venv xauth \
    && rm -rf /var/lib/apt/lists/*

COPY --from=uv /uv /uvx /usr/local/bin/
ENV UV_PYTHON_DOWNLOADS=never \
    UV_LINK_MODE=copy \
    PYTHONUNBUFFERED=1 \
    CYCLONEDDS_HOME=/opt/cyclonedds \
    LD_LIBRARY_PATH=/opt/cyclonedds/lib

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

ENV PATH="/opt/xr_teleoperate/.venv/bin:${PATH}"
USER teleop
WORKDIR /opt/xr_teleoperate/teleop
# Docker stop should give the app a chance to handle KeyboardInterrupt.
# Request the normal arm exit with Quest A-hold or keyboard q before stopping.
STOPSIGNAL SIGINT
CMD ["/bin/bash"]

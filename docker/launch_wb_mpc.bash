#!/usr/bin/env bash
#
# Usage:
#
# $ cd ~/your_colcon_ws/src/wb_humanoid_mpc/docker
# $ ./launch_wb_mpc.bash    # Launch the WB Humanoid MPC Docker container
#
# (Cross reference this file with the "run" section of ../.devcontainer/devcontainer.json)
#
set -euo pipefail

# Allow GUI applications
xhost +SI:localuser:root

# Generate Xauthority file for X11 forwarding
XAUTH=/tmp/.docker.xauth
if [ ! -f "${XAUTH}" ]; then
  touch "${XAUTH}"
  xauth nlist "${DISPLAY}" \
    | sed -e 's/^..../ffff/' \
    | xauth -f "${XAUTH}" nmerge -
  chmod a+r "${XAUTH}"
fi

# ROOT_COLCON_WS 
HOST_WS="$(realpath "${PWD}/../../..")"

# The WEMT API is kept next to the colcon workspace rather than baked into the
# image.  Mount it read-only when present.  Override with WEMT_API_DIR if the
# checkout is elsewhere.
HOST_WEMT_API="${WEMT_API_DIR:-$(realpath -m "${HOST_WS}/../WMET-API")}"
WEMT_DOCKER_ARGS=()
if [ -d "${HOST_WEMT_API}/wemt_api" ]; then
  WEMT_DOCKER_ARGS=(
    -v "${HOST_WEMT_API}:/opt/WMET-API:ro"
    -e "PYTHONPATH=/opt/WMET-API"
  )
else
  echo "Warning: WEMT API not found at ${HOST_WEMT_API}."
  echo "Set WEMT_API_DIR before launching if EM tracking is needed."
fi

# Run the container, mounting the entire workspace
docker run --rm -it \
  --name wb-mpc-dev \
  --net host \
  --privileged \
  -u root \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY="${XAUTH}" \
  -e XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${XAUTH}:${XAUTH}:rw" \
  -v "${HOST_WS}:/wb_humanoid_mpc_ws:cached" \
  "${WEMT_DOCKER_ARGS[@]}" \
  --workdir /wb_humanoid_mpc_ws \
  wb-humanoid-mpc:dev \
  bash

echo "Done."

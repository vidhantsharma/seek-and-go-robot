#!/usr/bin/env bash
set -e

IMAGE_NAME="seek_and_go_robot:latest"
CONTAINER_NAME="seek_and_go_robot_container"

# --- detect if sudo is needed for docker ---
if ! docker info >/dev/null 2>&1; then
  echo "[INFO] Docker requires sudo. Using sudo for all docker commands..."
  DOCKER_CMD="sudo docker"
else
  DOCKER_CMD="docker"
fi

# Allow GUI apps (Gazebo) to connect to host X server
echo "[INFO] Allowing X11 access for Docker..."
xhost +local:docker >/dev/null

# Build the Docker image
echo "[INFO] Building Docker image: ${IMAGE_NAME}"
${DOCKER_CMD} build -t ${IMAGE_NAME} .

# ROS / DDS settings (change ROS_DOMAIN_ID to match host)
ROS_DOMAIN_ID=42
RMW_IMPL=rmw_cyclonedds_cpp

# Run the container
echo "[INFO] Starting container: ${CONTAINER_NAME}"
${DOCKER_CMD} run -it \
  --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
  -e RMW_IMPLEMENTATION=${RMW_IMPL} \
  -e SDL_AUDIODRIVER=dummy \
  -e AUDIODEV=null \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri:/dev/dri \
  --name ${CONTAINER_NAME} \
  ${IMAGE_NAME} /bin/bash

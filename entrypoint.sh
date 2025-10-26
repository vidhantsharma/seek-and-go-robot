#!/usr/bin/env bash
set -e

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

# Source workspace overlay if built
WS=/root/ros2_ws
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
fi

# Forward any arguments to bash (so you can run e.g. ros2 launch ... directly via docker run)
exec "$@"

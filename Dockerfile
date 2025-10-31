# Use ROS2 Humble desktop as base (includes Gazebo desktop support)
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

# --- Install OS packages used by ROS/Gazebo and build tools ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-ros-base \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-plugins \
    ros-humble-teleop-twist-keyboard \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-slam-toolbox \
    git \
    wget \
    xterm \
    && rm -rf /var/lib/apt/lists/*

# initialize rosdep (required if you run apt install for ros deps in the container)
RUN rosdep init || true
RUN rosdep update || true

# create a workspace
ENV WS=/root/ros2_ws
RUN mkdir -p $WS/src
WORKDIR $WS

# copy your workspace packages into the image (assumes you place packages under ./src)
COPY src/ ./src/

# install python requirements if file exists (optional)
# You can include a requirements.txt at repo root if you need extra python libs
COPY requirements.txt /tmp/requirements.txt
RUN if [ -f /tmp/requirements.txt ]; then python3 -m pip install -r /tmp/requirements.txt; fi || true

# Resolve ros package dependencies with rosdep (best-effort)
RUN rosdep install --from-paths src --ignore-src -r -y || true

# Build the workspace
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# Add an entrypoint that sources ROS and the workspace overlay
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


ENV SDL_AUDIODRIVER=dummy
ENV AUDIODEV=null

ENV ROS_DOMAIN_ID=42

# small helpful default
ENV LANG=C.UTF-8
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

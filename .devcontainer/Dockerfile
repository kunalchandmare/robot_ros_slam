FROM ros:humble-ros-base

# Install essential build tools + ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-setuptools \
    python3-rosdep \
    # Basic ROS tools
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py \
    ros-humble-std-msgs \
    ros-humble-rqt-graph \
    ros-humble-rqt-common-plugins \
    # RTAB-Map (main packages)
    ros-humble-rtabmap \
    ros-humble-rtabmap-ros \
    # ros-humble-rtabmap-demos \      (unused, slows rebuilds)
    # ros-humble-rtabmap-examples \   (unused, slows rebuilds)
    # Navigation & simulation (optional but useful)
    # ros-humble-nav2-bringup \       (unused for now, slows rebuilds)
    # ros-humble-turtlebot3* \        (unused, very large, slows rebuilds)
    ros-humble-ros-gz \
    # Visualization and teleop (base image is ros:humble-ros-base, not desktop)
    ros-humble-rviz2 \
    ros-humble-teleop-twist-keyboard \
    xterm \
    # GUI / Qt dependencies (helps with RViz / rtabmap_viz)
    libqt5gui5 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Small extras go in their own step, so adding one later does not
# re-download everything above (Docker caches each RUN step separately).
# mesa-utils: provides glxinfo; "glxinfo -B" shows which GPU OpenGL uses.
# ros-humble-rqt-robot-steering: slider window to drive the robot (/cmd_vel)
#   without needing keyboard focus on a teleop terminal.
RUN apt-get update && apt-get install -y --no-install-recommends \
    mesa-utils \
    ros-humble-rqt-robot-steering \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (optional but recommended)
RUN rosdep update

# Auto-source ROS and workspace
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /workspaces/ros2-dev/install/setup.bash' >> /root/.bashrc

WORKDIR /workspaces/ros2-dev
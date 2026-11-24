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

# Navigation (Nav2) for autonomous frontier exploration (launch with explore:=true).
# Its own step, so the big step above stays cached.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# explore_lite (frontier exploration) has no apt package for Humble, so build it
# from source into its own small workspace /opt/explore_ws, pinned to a known commit.
# Only explore_lite and its message package are built (map_merge is skipped).
# ca-certificates: git (installed with --no-install-recommends) does not pull it in,
# and without it "git clone https://..." fails with "server certificate verification failed".
ARG M_EXPLORE_COMMIT=326cf8a0b487c34246bb8f3326afbcd69576dc60
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates \
    && update-ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /opt/explore_ws/src && cd /opt/explore_ws/src \
    && git clone https://github.com/robo-friends/m-explore-ros2.git \
    && cd m-explore-ros2 && git checkout ${M_EXPLORE_COMMIT} \
    && cd /opt/explore_ws \
    && . /opt/ros/humble/setup.sh \
    && colcon build --packages-up-to explore_lite --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# Initialize rosdep (optional but recommended)
RUN rosdep update

# Auto-source ROS and workspace
# Order matters: ROS, then explore_lite, then our workspace on top.
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /opt/explore_ws/install/setup.bash' >> /root/.bashrc && \
    echo 'source /workspaces/ros2-dev/install/setup.bash' >> /root/.bashrc

WORKDIR /workspaces/ros2-dev
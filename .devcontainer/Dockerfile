FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-setuptools \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py \
    ros-humble-std-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc

WORKDIR /workspaces/ros2-dev

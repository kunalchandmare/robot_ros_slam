# ROS 2 Humble Dev Container Setup

Simple reference for the Dockerfile and `devcontainer.json` used with VS Code.

---

## Dockerfile

```dockerfile
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
```

### Parameter explanations

| Parameter | Purpose |
|-----------|---------|
| `FROM ros:humble-ros-base` | Base image with ROS 2 Humble (minimal) |
| `build-essential`, `cmake`, `git` | Basic compilation and source control tools |
| `python3-pip`, `python3-colcon-common-extensions` | Python packages and ROS 2 build system |
| `ros-humble-demo-nodes-*` | Example nodes for testing |
| `ros-humble-std-msgs` | Standard message types |
| `echo 'source ...' >> /root/.bashrc` | Automatically sources ROS on every new terminal |
| `WORKDIR /workspaces/ros2-dev` | Sets the default working directory inside the container |

---

## devcontainer.json

```json
{
  "name": "ROS 2 Humble",
  "build": {
    "dockerfile": "Dockerfile"
  },
  "workspaceMount": "source=${localWorkspaceFolder},target=/workspaces/ros2-dev,type=bind",
  "workspaceFolder": "/workspaces/ros2-dev",
  "customizations": {
    "vscode": {
      "extensions": [
        "ms-vscode.cpptools",
        "ms-python.python"
      ],
      "settings": {
        "terminal.integrated.defaultProfile.linux": "bash"
      }
    }
  }
}
```

### Parameter explanations

| Parameter | Purpose |
|-----------|---------|
| `name` | Display name of the dev container in VS Code |
| `build.dockerfile` | Tells VS Code which Dockerfile to use to build the image |
| `workspaceMount` | Mounts your local project folder into the container |
| `workspaceFolder` | Sets the folder that opens when the container starts |
| `customizations.vscode.extensions` | VS Code extensions installed automatically in the container |
| `terminal.integrated.defaultProfile.linux` | Uses bash as the default terminal |

---

## How they work together

1. VS Code reads `devcontainer.json`
2. It builds the image using the Dockerfile
3. It starts the container and mounts your project folder
4. A terminal opens inside the container with ROS already sourced

This is a clean, minimal setup focused on ROS 2 development without GUI tools.

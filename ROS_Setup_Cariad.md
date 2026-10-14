# ROS 2 Docker setup — successful steps

Last updated: 14 October 2026.

This guide contains only confirmed successful setup milestones, in sequence. Simulator GUI setup will be added after it is successfully tested.

## 1. Install the development tools

Installed and confirmed available:

- Visual Studio Code and development extensions, including Dev Containers.
- Docker Desktop (reported application version: 4.76).
- Docker CLI: `29.5.2`, build `79eb04c`.

## 2. Install Microsoft WSL 2

Install the approved Microsoft Windows Subsystem for Linux x64 package, version 2.7.3.

Verify from Windows PowerShell:

```powershell
wsl --version
```

Confirmed output:

```text
WSL version: 2.7.3.0
Kernel version: 6.6.114.1-1
WSLg version: 1.0.73
MSRDC version: 1.2.6676
Direct3D version: 1.611.1-81528511
DXCore version: 10.0.26100.1-240331-1435.ge-release
Windows version: 10.0.26300.9457
```

## 3. Verify WSL status

Run in Windows PowerShell:

```powershell
wsl --status
```

Confirmed output:

```text
Default Version: 2
```

Hardware virtualization was also confirmed as Enabled in Task Manager → Performance → CPU.

The exact Windows feature-enablement actions were not recorded, so this guide does not claim a particular command was executed successfully.

## 4. Verify Docker can run a container

With Docker Desktop running, execute in Windows PowerShell:

```powershell
docker run --rm hello-world
```

Confirmed output included:

```text
Hello from Docker!
This message shows that your installation appears to be working correctly.
```

This verified that the Docker client contacted the daemon, downloaded the image, created a container, and received its output.

## 5. Download and test ROS 2 Humble

Run in Windows PowerShell:

```powershell
docker run --rm ros:humble-ros-base ros2 --help
```

The image downloaded successfully and ROS 2 command help appeared:

```text
usage: ros2 [-h] [--use-python-default-buffering]
            Call `ros2 <command> -h` for more detailed usage. ...

ros2 is an extensible command-line tool for ROS 2.
```

Confirmed commands included `run`, `launch`, `node`, `topic`, `pkg`, and `service`.

Recorded image digest:

```text
sha256:860e53793ccf575e5369b5e9a19939de2bfcfb46a299954d06f54f2af54fed19
```

## 6. Prepare the workspace and Dev Container files

Workspace folder on Windows:

```text
C:\Data\Repos\ros_robot_slam\robot_ros_slam
```

Open this project folder in VS Code, not the `.devcontainer` directory itself:

```powershell
code "C:\Data\Repos\ros_robot_slam\robot_ros_slam"
```

Configuration structure:

```text
robot_ros_slam/
├── .devcontainer/
│   ├── Dockerfile
│   └── devcontainer.json
├── ROS_Setup_Cariad.md
└── src/
```

The following configuration was provided for the successfully opened container. The local configuration files were not independently read back from the repository.

### .devcontainer/Dockerfile

```dockerfile
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc

WORKDIR /workspaces/ros2-dev
```

### .devcontainer/devcontainer.json

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

`${localWorkspaceFolder}` refers to the project folder opened in VS Code. That Windows folder appears inside the container at `/workspaces/ros2-dev`; the two folder names do not need to match.

The `.devcontainer` configuration is workspace-specific, not a global ROS installation setting.

## 7. Build and open the Dev Container

1. Keep Docker Desktop running.
2. Open the workspace folder in VS Code.
3. Save the configuration files.
4. Press Ctrl+Shift+P.
5. Select `Dev Containers: Reopen in Container`.
6. Wait for the build and connection to complete.

Confirmed: the development container opened successfully, and a new development image appeared in Docker Desktop.

## 8. Verify ROS inside the VS Code container terminal

Open Terminal → New Terminal in VS Code. Run these Linux commands inside the container:

```bash
pwd
echo "$ROS_DISTRO"
ros2 --help
```

Confirmed output:

```text
/workspaces/ros2-dev
humble
usage: ros2 ...
```

## 9. Test C++ talker and Python listener communication

In the first VS Code container terminal:

```bash
ros2 run demo_nodes_cpp talker
```

Leave it running and open a second container terminal using the terminal panel's `+` button:

```bash
ros2 run demo_nodes_py listener
```

Confirmed: both nodes worked and the listener received the talker's messages.

Typical output:

```text
[talker]: Publishing: 'Hello World: 1'
[listener]: I heard: [Hello World: 1]
```

Stop each node using Ctrl+C in its terminal.

## Current verified milestone

Docker runs Linux containers; VS Code opens the ROS 2 Humble Dev Container with the project mounted at `/workspaces/ros2-dev`; C++ and Python ROS nodes communicate successfully.

## Successful-change log

| Date | Successful milestone |
|---|---|
| 2026-10-14 | VS Code, extensions, and Docker installed |
| 2026-10-14 | Hardware virtualization confirmed Enabled |
| 2026-10-14 | WSL runtime 2.7.3.0 verified |
| 2026-10-14 | WSL status reports Default Version: 2 |
| 2026-10-14 | Docker hello-world completed |
| 2026-10-14 | ROS 2 Humble container printed ROS command help |
| 2026-10-14 | VS Code Dev Container opened successfully; development image visible in Docker Desktop |
| 2026-10-14 | Container workspace path and Humble environment verified |
| 2026-10-14 | C++ talker and Python listener communication confirmed |

Add subsequent setup steps here only after their commands and results have been confirmed successful.
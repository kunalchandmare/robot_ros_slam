# ROS 2 Docker setup — successful steps

Last updated: 12 November 2026.

This guide contains only confirmed successful setup milestones, in sequence. Simulator GUI setup (RViz2 and Gazebo via WSLg) is now included after successful testing.

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

## 3. Install Ubuntu (any version) for WSL2

A real Ubuntu distribution is required for full WSLg access and to run Docker commands from a normal Linux environment. Any Ubuntu version (e.g. 22.04 or 24.04) works. ROS 2 itself remains inside the Docker container; Ubuntu is only used on the host side.

From Windows PowerShell:

```powershell
wsl --install -d Ubuntu
```

(Or install the company-approved Ubuntu 24.04 WSL package if one is provided.)

After installation, create a Linux username and password when prompted. Then set it as the default (optional but recommended):

```powershell
wsl --set-default Ubuntu
```

Verify from PowerShell:

```powershell
wsl -l -v
```

You should see Ubuntu listed with Version 2.

Start Ubuntu and confirm WSLg:

```powershell
wsl -d Ubuntu
```

Inside the Ubuntu shell:

```bash
echo $DISPLAY
ls /mnt/wslg
ls /mnt/wslg/.X11-unix
```

Confirmed: `DISPLAY` shows `:0`, `/mnt/wslg` exists, and the X socket (`X0`) is present. This confirms WSLg is available for GUI applications.

## 4. Verify WSL status

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

## 5. Verify Docker can run a container

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

## 6. Download and test ROS 2 Humble

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

## 7. Prepare the workspace and Dev Container files

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
    ros-humble-rtabmap-demos \
    ros-humble-rtabmap-examples \
    # Navigation & simulation (optional but useful)
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    ros-humble-gazebo-ros-pkgs \
    # GUI / Qt dependencies (helps with RViz / rtabmap_viz)
    libqt5gui5 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (optional but recommended)
RUN rosdep update

# Auto-source ROS and workspace
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /workspaces/ros2-dev/install/setup.bash' >> /root/.bashrc

WORKDIR /workspaces/ros2-dev
```

### .devcontainer/devcontainer.json

Updated configuration that enables native WSLg for RViz2 and Gazebo (no external X server required):

```json
{
  "name": "ROS 2 Humble",
  "build": {
    "dockerfile": "Dockerfile"
  },
  "workspaceMount": "source=${localWorkspaceFolder},target=/workspaces/ros2-dev,type=bind",
  "workspaceFolder": "/workspaces/ros2-dev",
  "runArgs": [
    "--net=host"
  ],
  "containerEnv": {
    "DISPLAY": ":0",
    "WAYLAND_DISPLAY": "wayland-0",
    "XDG_RUNTIME_DIR": "/mnt/wslg/runtime-dir",
    "PULSE_SERVER": "/mnt/wslg/PulseServer",
    "QT_X11_NO_MITSHM": "1"
  },
  "mounts": [
    "source=/run/desktop/mnt/host/wslg/.X11-unix,target=/tmp/.X11-unix,type=bind",
    "source=/run/desktop/mnt/host/wslg,target=/mnt/wslg,type=bind"
  ],
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

**Key settings explained:**

- `"runArgs": ["--net=host"]` — shares the host network namespace (simplifies ROS 2 discovery).
- `containerEnv` — sets the display and runtime variables that WSLg expects:
  - `DISPLAY=:0` and `WAYLAND_DISPLAY=wayland-0` tell Qt/OpenGL applications which display to use.
  - `XDG_RUNTIME_DIR` and `PULSE_SERVER` point to the WSLg runtime and audio sockets.
  - `QT_X11_NO_MITSHM=1` avoids shared-memory issues common in containers.
- `mounts` — bind-mounts the host WSLg directories so the container can see the X11 socket and runtime files:
  - `/run/desktop/mnt/host/wslg/.X11-unix` → `/tmp/.X11-unix`
  - `/run/desktop/mnt/host/wslg` → `/mnt/wslg`

`${localWorkspaceFolder}` refers to the project folder opened in VS Code. That Windows folder appears inside the container at `/workspaces/ros2-dev`; the two folder names do not need to match.

The `.devcontainer` configuration is workspace-specific, not a global ROS installation setting.

The `containerEnv` and `mounts` sections forward the host WSLg display into the container so that Qt-based tools (RViz2, Gazebo, rtabmap_viz, etc.) can open windows directly on the Windows desktop.

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

## 10. Enable and test GUI tools (RViz2 and Gazebo) via WSLg

With the updated `devcontainer.json` (WSLg mounts and environment variables) and Ubuntu WSL running, GUI applications work natively.

Inside the VS Code container terminal:

```bash
# Verify display is available
echo $DISPLAY
ls /mnt/wslg
ls /tmp/.X11-unix

# Launch RViz2
ros2 run rviz2 rviz2

# In a second terminal, launch Gazebo (takes 30–90 seconds on first start)
gazebo
# or
ros2 launch gazebo_ros gazebo.launch.py
```

Confirmed: both RViz2 and Gazebo windows open on the Windows desktop. They can run in parallel. No external X server (MobaXterm/VcXsrv) is required when using this WSLg configuration.

If a window is blank, add `"LIBGL_ALWAYS_SOFTWARE": "1"` to `containerEnv` and rebuild.

## Current verified milestone

Docker runs Linux containers; VS Code opens the ROS 2 Humble Dev Container with the project mounted at `/workspaces/ros2-dev`; C++ and Python ROS nodes communicate successfully; RViz2 and Gazebo GUIs open via native WSLg.

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
| 2026-11-12 | Ubuntu WSL2 installed and WSLg verified (`DISPLAY=:0`, `/mnt/wslg` present) |
| 2026-11-12 | `devcontainer.json` updated with WSLg mounts and environment variables |
| 2026-11-12 | RViz2 and Gazebo confirmed working in parallel via native WSLg |

Add subsequent setup steps here only after their commands and results have been confirmed successful.
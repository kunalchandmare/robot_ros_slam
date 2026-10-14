# ROS 2 Docker setup — successful steps

Last updated: 14 October 2026.

This guide contains only the setup milestones confirmed successful. Workspace creation, Dev Container builds, and simulator GUI setup will be added after they are successfully tested.

## 1. Install the development tools

Installed and confirmed available:

- Visual Studio Code and development extensions, including Dev Containers.
- Docker Desktop (reported application version: 4.76).
- Docker CLI: `29.5.2`, build `79eb04c`.

## 2. Install Microsoft WSL 2

Install the approved Microsoft Windows Subsystem for Linux x64 package, version 2.7.3.

The installed runtime was verified from Windows PowerShell:

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

## Current verified milestone

Docker can run Linux containers, and ROS 2 Humble executes successfully inside its container image.

## Successful-change log

| Date | Successful milestone |
|---|---|
| 2026-10-14 | VS Code, extensions, and Docker installed |
| 2026-10-14 | Hardware virtualization confirmed Enabled |
| 2026-10-14 | WSL runtime 2.7.3.0 verified |
| 2026-10-14 | WSL status reports Default Version: 2 |
| 2026-10-14 | Docker hello-world completed |
| 2026-10-14 | ROS 2 Humble container printed ROS command help |

Add subsequent setup steps here only after their commands and results have been confirmed successful.

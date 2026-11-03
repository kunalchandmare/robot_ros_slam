# ROS 2 Package Installation Guide (Humble)

This guide explains how to install **any** ROS 2 package.  
There are two main methods:

1. **Binary packages** using `apt` (recommended for beginners)
2. **From source** using `git` + `colcon` (for latest versions or custom packages)

---

## Method 1: Install with `apt` (Easiest)

### Commands

```bash
sudo apt update
```

**What it does:**  
Updates the list of available packages on the system. Always run this before installing new software.

```bash
sudo apt install ros-humble-<package-name>
```

**What it does:**  
Installs the ROS 2 Humble version of the package.  
Replace `<package-name>` with the real package name (example: `rtabmap-ros`, `nav2-bringup`, `turtlebot3`).

**Example:**
```bash
sudo apt update
sudo apt install ros-humble-rtabmap-ros
```

```bash
source /opt/ros/humble/setup.bash
```

**What it does:**  
Makes the newly installed packages available in the current terminal.

---

## Method 2: Install from Source

Use this method when the package is not available via `apt`, or when you need the latest development version.

### Commands

```bash
cd /workspaces/ros2-dev/src
```

**What it does:**  
Goes to the `src` folder of your workspace (where source packages are placed).

```bash
git clone https://github.com/<user>/<repository>.git
```

**What it does:**  
Downloads the source code of the package from GitHub.

```bash
cd /workspaces/ros2-dev
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**What it does:**
- `rosdep update` → Updates the dependency database.
- `rosdep install` → Automatically installs all system dependencies required by the packages in `src`.
  - `--from-paths src` → Looks for packages inside the `src` folder.
  - `--ignore-src` → Skips packages that are already in the workspace.
  - `-r` → Continues even if some dependencies fail.
  - `-y` → Automatically confirms installation prompts.

```bash
colcon build --symlink-install
```

**What it does:**  
Builds all packages in the workspace.  
`--symlink-install` creates symbolic links so changes to source files are reflected without a full rebuild (useful during development).

```bash
source /workspaces/ros2-dev/install/setup.bash
```

**What it does:**  
Sources the workspace so the newly built packages become available.

---

## Useful Commands

| Command | Purpose |
|---------|---------|
| `ros2 pkg list \| grep <keyword>` | Check if a package is installed |
| `sudo apt search ros-humble-<keyword>` | Search for available packages |
| `source /opt/ros/humble/setup.bash` | Source the base ROS 2 installation |
| `source /workspaces/ros2-dev/install/setup.bash` | Source your workspace |

---

## Recommended Workflow for Beginners

1. Always try **Method 1 (`apt`)** first.
2. If the package is not found, use **Method 2 (from source)**.
3. After any installation, **source** the setup files.
4. Verify the installation with:
   ```bash
   ros2 pkg list | grep <package-name>
   ```

---

## Quick Comparison

| Method       | When to Use                      | Difficulty | Speed   |
|--------------|----------------------------------|------------|---------|
| `apt`        | Most official packages           | Easy       | Fast    |
| From source  | Latest version / custom packages | Medium     | Slower  |

---

**Note:** Always source the environment after installing or building packages, otherwise ROS 2 will not see them.

# ROS 2 Build Guide for `robot_ros_slam`

This guide describes how this repository builds and runs its battery-node example. The project uses ROS 2 Humble in the Dev Container configured under `.devcontainer/`.

## Project layout

The repository root is the colcon workspace root. `battery_node_demo/` is a grouping directory, not a ROS package; it contains two independent packages with different build systems:

```text
robot_ros_slam/                 # Workspace root
├── battery_node_demo/          # Groups the example packages
│   ├── battery_python/          # ament_python package
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── resource/battery_python
│   │   └── battery_python/
│   │       ├── battery_publisher.py
│   │       └── battery_processor.py
│   └── battery_cpp/             # ament_cmake package
│       ├── package.xml
│       ├── CMakeLists.txt
│       └── src/battery_listener.cpp
├── build/                       # Generated; do not commit
├── install/                     # Generated; do not commit
└── log/                         # Generated; do not commit
```

The Dev Container mounts the opened repository at `/workspaces/ros2-dev`. Run the commands below in a terminal inside that container, from the workspace root.

## Beginner mental model

Think of a ROS 2 workspace as a small factory:

```text
You write code in src/
        |
        v
colcon build
        |
        +--> build/     temporary work files
        |
        +--> install/   ready-to-use package + setup.bash
        |
        v
source install/setup.bash
        |
        v
ros2 run <package> <executable>
```

- `src/` is where you write and edit your ROS packages.
- `build/` is temporary scratch space created by `colcon`.
- `install/` contains the ready-to-run version of your packages.
- `install/setup.bash` tells the current terminal where your packages are.
- `ros2 run` starts an executable registered by a ROS package.

A ROS package is a folder with a `package.xml` file. It can contain Python nodes, C++ nodes, launch files, configuration files, message definitions, and other resources.

## What each folder means

| Folder | Purpose | Should you edit it? |
|---|---|---|
| `src/` | Your ROS packages and source code | Yes |
| `battery_node_demo/` | A grouping folder for the two demo packages | Yes |
| `build/` | Temporary files generated while building | No |
| `install/` | Installed packages, executables, and setup scripts | No |
| `log/` | Build logs, warnings, and errors | No |

`build/`, `install/`, and `log/` are generated output. You can delete them and rebuild, but do not delete `src/` because it contains your actual project code.

To clean and rebuild from scratch:

```bash
rm -rf build/ install/ log/
colcon build
```

## What registers each executable?

A ROS package must install its runnable executable into the location ROS 2 expects. In this project, each language uses its own package and build configuration:

| Package | Build type | Executable registration |
|---|---|---|
| `battery_python` | `ament_python` | `setup.py` declares the `battery_publisher` and `battery_processor` console scripts; `setup.cfg` places their wrappers under `lib/battery_python`. |
| `battery_cpp` | `ament_cmake` | `CMakeLists.txt` creates `battery_listener` and installs it under `lib/battery_cpp`. |

Each package's `package.xml` declares its build type and ROS dependencies. The Python package depends on `rclpy` and `std_msgs`; the C++ package depends on `rclcpp` and `std_msgs`. The `resource/battery_python` marker and Python package directory support package discovery and module installation.

The packages are separate, not a single mixed-language package. Colcon discovers and builds both from the workspace root.

## Python package structure

The Python package has this structure:

```text
battery_python/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/battery_python
└── battery_python/
    ├── battery_publisher.py
    └── battery_processor.py
```

| File or folder | Purpose |
|---|---|
| `package.xml` | Describes the ROS package: name, version, maintainer, build type, and ROS dependencies |
| `setup.py` | Python packaging file; registers executable commands through `console_scripts` |
| `setup.cfg` | Controls where Python executable wrappers are installed |
| `resource/battery_python` | Marker file used for ROS package discovery |
| `battery_python/` | Contains the importable Python modules and node code |

The `console_scripts` section in `setup.py` is what allows these commands to work:

```bash
ros2 run battery_python battery_publisher
ros2 run battery_python battery_processor
```

For example, if `setup.py` contains:

```python
entry_points={
    'console_scripts': [
        'battery_publisher = battery_python.battery_publisher:main',
        'battery_processor = battery_python.battery_processor:main',
    ],
},
```

then ROS knows that:

- `battery_publisher` starts `battery_publisher.py`
- `battery_processor` starts `battery_processor.py`

Python does not need compilation, but it still must be installed into the ROS workspace by `colcon`.

## C++ package structure

The C++ package has this structure:

```text
battery_cpp/
├── package.xml
├── CMakeLists.txt
└── src/
    └── battery_listener.cpp
```

| File or folder | Purpose |
|---|---|
| `package.xml` | Describes the ROS package and dependencies such as `rclcpp` and `std_msgs` |
| `CMakeLists.txt` | Tells CMake how to compile and install the C++ executable |
| `src/battery_listener.cpp` | Contains the C++ ROS node implementation |

After building, run:

```bash
ros2 run battery_cpp battery_listener
```

Unlike Python, C++ must be compiled. If you change `battery_listener.cpp`, rebuild the package before running it again:

```bash
colcon build --packages-select battery_cpp
source install/setup.bash
ros2 run battery_cpp battery_listener
```

## Build

In the Dev Container terminal:

```bash
cd /workspaces/ros2-dev
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select battery_python battery_cpp
```

`--symlink-install` is useful during development: Python source files are linked into the install space instead of copied. The C++ executable is still compiled during the build. `--packages-select` builds the two packages in this example.

After a successful build, source the workspace overlay in that terminal:

```bash
source install/setup.bash
```

For every new terminal, source both the ROS distribution and the workspace overlay before using the packages:

```bash
source /opt/ros/humble/setup.bash
source /workspaces/ros2-dev/install/setup.bash
```

Sourcing makes installed packages discoverable in the current shell; it does not build packages or start nodes.

## What happens during `colcon build`?

When you run:

```bash
colcon build --symlink-install --packages-select battery_python battery_cpp
```

colcon does the following:

1. Searches the workspace for ROS packages.
2. Finds `battery_python` and `battery_cpp`.
3. Reads each package’s `package.xml` to learn its name, build type, and dependencies.
4. For `battery_python`, installs Python modules and creates executable wrappers using `setup.py` and `setup.cfg`.
5. For `battery_cpp`, compiles `src/battery_listener.cpp` using `CMakeLists.txt` and installs the resulting executable.
6. Writes temporary build files into `build/`.
7. Places the usable package files and setup scripts into `install/`.

After the build, `install/` contains the versions ROS actually runs. If you edit source code, the installed version does not automatically update; rebuild the affected package and restart the node.

## What does `source` do?

`source` runs a shell script inside your current terminal. It changes the terminal’s environment: executable paths, Python module paths, and ROS variables.

### Source ROS

```bash
source /opt/ros/humble/setup.bash
```

This loads the base ROS 2 installation, called the **underlay**.

It provides:

- `ros2`
- `colcon`
- `rviz2`
- `rclpy`
- `rclcpp`
- Standard ROS message packages such as `std_msgs`

Without this command, you may see:

```text
ros2: command not found
```

or:

```text
ModuleNotFoundError: No module named 'rclpy'
```

### Source your workspace

```bash
source install/setup.bash
```

This loads your own built ROS packages, called the **overlay**.

```text
/opt/ros/humble                 -> underlay: base ROS 2
~/robot_ros_slam/install        -> overlay: your custom packages
```

The overlay tells ROS:

- where your package is installed
- which executables it provides
- where its Python modules and libraries are

That is why this command becomes available after sourcing:

```bash
ros2 run battery_python battery_publisher
```

Without sourcing `install/setup.bash`, ROS may not know that `battery_python` exists, even if you successfully built it.

## Correct sourcing order

Use this order:

```bash
source .venv/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
```

| Command | What it loads | Example result |
|---|---|---|
| `source .venv/bin/activate` | Your isolated Python packages | `import cv2` works |
| `source /opt/ros/humble/setup.bash` | ROS 2 itself | `ros2`, `colcon`, and `rclpy` work |
| `source install/setup.bash` | Your built workspace | `ros2 run battery_python battery_publisher` works |

These settings disappear when you close the terminal. In every new terminal, source the environment again before running ROS commands.

## Run the nodes

Run each executable in its own sourced terminal:

```bash
ros2 run battery_python battery_publisher
```

```bash
ros2 run battery_python battery_processor
```

```bash
ros2 run battery_cpp battery_listener
```

The publisher sends `std_msgs/msg/Int32` messages on `/battery_level`. The processor and C++ listener subscribe to that topic; the processor publishes low-battery warnings as `std_msgs/msg/String` on `/battery_warning` when the level is 20 or lower. See [`battery_node_demo/README.md`](../battery_node_demo/README.md) for the full communication diagram and inspection commands.

Press `Ctrl+C` in each node terminal to stop it.

## External pip packages

ROS dependencies and pip dependencies are managed differently.

| Dependency type | Examples | Install with | Declare in |
|---|---|---|---|
| ROS package | `rclpy`, `rclcpp`, `std_msgs`, `sensor_msgs` | `rosdep` / apt | `package.xml` |
| System Python package with ROS key | NumPy, sometimes YAML-related packages | `rosdep` / apt | `package.xml` |
| External PyPI package | `opencv-python`, `ultralytics`, `torch`, `transformers` | `pip` | `requirements.txt` |

Do not put ROS packages such as `rclpy`, `std_msgs`, or `sensor_msgs` in `requirements.txt`. They are installed by ROS, not pip.

### When to use `package.xml`

Add a dependency to `package.xml` when ROS can resolve it through `rosdep`:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>python3-numpy</exec_depend>
```

Then install declared dependencies with:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

`rosdep` reads dependency keys from `package.xml`, looks them up in the ROS dependency index, and installs the matching system or ROS packages.

### When to use `requirements.txt`

Use `requirements.txt` for external Python packages that are installed from PyPI:

```txt
opencv-python>=4.8,<5.0
ultralytics>=8.0
```

Install them with:

```bash
pip install -r requirements.txt
```

You do **not** need to declare every pip package in `package.xml`. Declare it there only when ROS provides a matching dependency key.

### Example: adding OpenCV

Install it:

```bash
pip install opencv-python
```

Add it to `requirements.txt`:

```txt
opencv-python>=4.8,<5.0
```

Import it in your Python node:

```python
import cv2
```

If your node uses camera images, also declare the ROS message dependency in `package.xml`:

```xml
<exec_depend>sensor_msgs</exec_depend>
```

## Using a virtual environment with ROS

A virtual environment isolates external pip packages. For ROS development, create it with `--system-site-packages` so it can still use ROS-installed modules such as `rclpy`.

Create the environment once:

```bash
cd /workspaces/ros2-dev
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r battery_node_demo/battery_python/requirements.txt
```

The `--system-site-packages` option lets the virtual environment import ROS packages installed system-wide, while keeping your pip packages isolated inside `.venv`.

Activate it in every new terminal before running a Python ROS node that needs external packages:

```bash
source .venv/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Then run:

```bash
ros2 run battery_python battery_publisher
```

To check that the virtual environment is active:

```bash
which python
which pip
```

Expected output:

```text
/workspaces/ros2-dev/.venv/bin/python
/workspaces/ros2-dev/.venv/bin/pip
```

To leave the virtual environment:

```bash
deactivate
```

## Complete beginner workflow

```bash
cd /workspaces/ros2-dev

# One-time setup: create and populate the Python environment
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r battery_node_demo/battery_python/requirements.txt

# Build the ROS workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select battery_python battery_cpp

# Prepare a new terminal for running nodes
source .venv/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run nodes
ros2 run battery_python battery_publisher
ros2 run battery_python battery_processor
ros2 run battery_cpp battery_listener
```

If you do not use external pip packages, you do not need `.venv` or `requirements.txt`. You only need to source ROS and your workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run battery_python battery_publisher
```

## Build and runtime flow

```text
battery_python/package.xml + setup.py + setup.cfg
    -> install Python modules and console-script wrappers
    -> ros2 run battery_python <executable>

battery_cpp/package.xml + CMakeLists.txt + src/battery_listener.cpp
    -> compile and install the C++ executable
    -> ros2 run battery_cpp battery_listener

colcon build
    -> workspace install/ space
    -> source install/setup.bash
    -> ROS 2 can discover the installed packages and executables
```

## When project files change

| Change | Action |
|---|---|
| Edit Python node logic | Restart that node. With `--symlink-install`, ordinary source edits usually do not require another build. |
| Add or rename a Python executable | Update the `console_scripts` list in `battery_python/setup.py`, rebuild, and use the registered executable name with `ros2 run`. |
| Edit C++ node logic | Rebuild `battery_cpp` and restart the node. |
| Add or rename a C++ executable | Add/update its target and install rule in `battery_cpp/CMakeLists.txt`, rebuild, and use the installed target name with `ros2 run`. |
| Add a ROS dependency | Declare it in the package's `package.xml` and ensure it is available in the development environment. |
| Add an external pip dependency | Add it to `requirements.txt`, install it with pip, and use a `--system-site-packages` virtual environment if needed. |
| Change package installation metadata | Rebuild the affected package. |

Build artifacts in `build/`, `install/`, and `log/` are generated and are excluded from Git by the repository's `.gitignore`.

## Common beginner errors

| Error | Likely cause | Fix |
|---|---|---|
| `ros2: command not found` | ROS is not sourced | `source /opt/ros/humble/setup.bash` |
| `Package 'battery_python' not found` | Workspace overlay is not sourced | `source install/setup.bash` |
| `No module named 'rclpy'` | ROS environment is missing, or the venv cannot see system packages | Source ROS; recreate the venv with `--system-site-packages` |
| `No module named 'cv2'` | External pip package is not installed in the active environment | Activate the venv and run `pip install -r requirements.txt` |
| Node does not reflect a code change | Package was not rebuilt or node was not restarted | Rebuild if needed, source the overlay, restart the node |
| C++ change has no effect | C++ must be recompiled | `colcon build --packages-select battery_cpp` |
| `colcon: command not found` | ROS build tools are not installed or ROS is not sourced | Install `python3-colcon-common-extensions` and source ROS |
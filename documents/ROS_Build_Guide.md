The key idea is **registration**: writing a node’s source code is not enough. The package must tell the build system how to install that code as an executable ROS can find. [docs.ros](https://docs.ros.org/en/humble/How-To-Guides/Developing-a-ROS-2-Package.html)

# How a ROS 2 project becomes runnable

## 1. The overall flow

```text
Write node code
      ↓
Describe the package and dependencies
      ↓
Define how its executables are built/installed
      ↓
colcon build
      ↓
Installed package: executables + modules + metadata
      ↓
source install/setup.bash
      ↓
ros2 run <package> <executable>
      ↓
Program starts and creates ROS node(s)
      ↓
Executor processes timers, messages, and callbacks
```

`colcon` coordinates package builds. The package’s selected build system determines how its code is compiled or installed. Sourcing the result makes the installed packages available to your shell. [docs.ros](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Colcon-Tutorial.html)

## 2. Workspace, package, executable, and node

These terms describe different things:

| Term | Meaning | Battery-demo example |
|---|---|---|
| Workspace | Folder containing one or more packages and their build outputs. | `robot_ws` |
| Package | A unit of code, dependencies, and installation instructions. | `simple_nodes_demo` |
| Executable | A program that can be started. | `battery_publisher` |
| Node | A ROS participant created while a program runs. | A node named `battery_publisher` |

A package can provide several executables. An executable can create one or more nodes; a node is not necessarily the same thing as an operating-system process. [docs.ros](https://docs.ros.org/en/humble/How-To-Guides/Developing-a-ROS-2-Package.html)

For the example:

```text
simple_nodes_demo package
├── battery_publisher executable → Python publisher node
├── battery_processor executable → Python processor node
└── battery_listener executable  → C++ subscriber node
```

# The configuration files you need

## 3. Choose one build route per package

There is no single set of configuration files required by every ROS 2 project. The main distinction is the package’s build type. [design.ros2](https://design.ros2.org/articles/ament.html)

| Package type | Build type | Main configuration files |
|---|---|---|
| Pure Python | `ament_python` | `package.xml`, `setup.py`, `setup.cfg` |
| C++ | `ament_cmake` | `package.xml`, `CMakeLists.txt` |
| Mixed C++ and Python in one package | `ament_cmake`, with Python integration | `package.xml`, `CMakeLists.txt`; typically `ament_cmake_python` |

A package does not normally run two independent build routes just because both `setup.py` and `CMakeLists.txt` exist. Mixed packages need one coherent installation strategy. [discourse.openrobotics](https://discourse.openrobotics.org/t/mixed-python-cpp-ament-package/1452)

## 4. `package.xml`: “What is this package?”

Every standard ROS 2 package has a manifest describing its identity, dependencies, and build type. [docs.ros](https://docs.ros.org/en/humble/How-To-Guides/Developing-a-ROS-2-Package.html)

Example for a pure Python battery package:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>simple_nodes_demo</name>
  <version>0.1.0</version>
  <description>Battery publisher and processor example</description>

  <maintainer email="developer@example.com">Developer</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Why it matters

| Part | Purpose |
|---|---|
| `<name>` | Identifies the ROS package. |
| Dependency tags | Tell tooling what the package needs. |
| `<buildtool_depend>` | Declares the required build tooling. |
| `<build_type>` | Selects the build route. |

Declaring a dependency is not the same as installing it. Dependencies must already be available or be resolved separately—for example, using `rosdep`. [docs.ros](https://docs.ros.org/en/humble/How-To-Guides/Developing-a-ROS-2-Package.html)

# Pure Python packages

## 5. Minimum practical structure

```text
simple_nodes_demo/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── simple_nodes_demo
└── simple_nodes_demo/
    ├── __init__.py
    ├── battery_publisher.py
    └── battery_processor.py
```

The outer directory is the ROS package. The inner directory is the importable Python package. [docs.ros](https://docs.ros.org/en/humble/How-To-Guides/Developing-a-ROS-2-Package.html)

## 6. `setup.py`: “What Python code and executables should be installed?”

A simplified example:

```python
from setuptools import setup

package_name = 'simple_nodes_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'battery_publisher = simple_nodes_demo.battery_publisher:main',
            'battery_processor = simple_nodes_demo.battery_processor:main',
        ],
    },
)
```

The important connection is:

```text
battery_publisher = simple_nodes_demo.battery_publisher:main
        │                       │                       │
        │                       │                       └─ Function to call
        │                       └─ Python module to import
        └─ Installed executable name
```

After building, this allows:

```bash
ros2 run simple_nodes_demo battery_publisher
```

The installed executable is a small wrapper that imports the module and calls `main()`. [docs.ros](https://docs.ros.org/en/humble/How-To-Guides/Developing-a-ROS-2-Package.html)

## 7. `setup.cfg`: “Where should the executable wrappers go?”

```ini
[develop]
script_dir=$base/lib/simple_nodes_demo

[install]
install_scripts=$base/lib/simple_nodes_demo
```

This places the scripts in the package-specific location expected by `ros2 run`, rather than an unrelated general script directory. [theconstruct](https://www.theconstruct.ai/ros2-tutorials-5-how-to-create-a-ros2-package-for-python-update/)

## 8. Resource marker and `__init__.py`

| File | Purpose |
|---|---|
| `resource/simple_nodes_demo` | Usually an empty marker installed into the ament resource index so ROS can discover the package. |
| `simple_nodes_demo/__init__.py` | Marks the inner directory as an ordinary Python package. It can be empty. |

The marker is installed through `setup.py`; it is not executable node code. [docs.ros](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Python-Documentation.html)

## 9. What happens when a Python node runs?

Using the battery publisher as an example:

```python
def main(args=None):
    rclpy.init(args=args)
    node = BatteryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

Conceptually:

```text
ros2 run
   ↓
Installed Python wrapper
   ↓
Import battery_publisher module
   ↓
Call main()
   ↓
Initialize ROS
   ↓
Construct node, publisher, and timer
   ↓
spin(): dispatch callbacks
   ↓
Timer callback publishes battery messages
```

Python does not need a C++-style native compilation step. Nevertheless, its package still needs installation of modules, executable wrappers, and discovery metadata. [docs.ros](https://docs.ros.org/en/humble/How-To-Guides/Developing-a-ROS-2-Package.html)

# C++ packages

## 10. Minimum practical structure

```text
simple_nodes_demo/
├── package.xml
├── CMakeLists.txt
└── src/
    └── battery_listener.cpp
```

An `include/` directory is added when you have headers to expose; it is not mandatory for every small executable. [docs.ros](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

## 11. `CMakeLists.txt`: “How should the executable be compiled and installed?”

```cmake
cmake_minimum_required(VERSION 3.8)
project(simple_nodes_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(
  battery_listener
  src/battery_listener.cpp
)

ament_target_dependencies(
  battery_listener
  rclcpp
  std_msgs
)

install(
  TARGETS battery_listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

### What each part does

| Instruction | Purpose |
|---|---|
| `find_package(...)` | Finds installed dependencies needed by the build. |
| `add_executable(...)` | Defines a program to compile from the listed source files. |
| `ament_target_dependencies(...)` | Connects the target to its ROS dependencies. |
| `install(TARGETS ...)` | Installs the executable where ROS expects it. |
| `ament_package()` | Generates and installs package metadata. |

Compiling successfully is not enough if the executable is never installed into the package’s expected location. [docs.ros](https://docs.ros.org/en/humble/How-To-Guides/Developing-a-ROS-2-Package.html)

The manifest for this route declares:

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<depend>rclcpp</depend>
<depend>std_msgs</depend>

<export>
  <build_type>ament_cmake</build_type>
</export>
```

These are relevant fragments, not a complete `package.xml`. [docs.ros](https://docs.ros.org/en/humble/How-To-Guides/Developing-a-ROS-2-Package.html)

## 12. What happens when a C++ node runs?

```text
battery_listener.cpp
       ↓ Build time
Compiler produces object files
       ↓
Linker produces native executable
       ↓
Installation into the workspace
       ↓ Runtime
ros2 run simple_nodes_demo battery_listener
       ↓
Operating system starts executable
       ↓
main() initializes ROS and creates node
       ↓
spin() processes incoming-message callbacks
```

Unlike Python, changing C++ source requires recompiling before the executable can use those changes. [docs.ros](https://docs.ros.org/en/humble/How-To-Guides/Developing-a-ROS-2-Package.html)

# Mixed C++ and Python projects

## 13. Two valid ways to organize them

### Option A: Separate packages in one workspace

```text
robot_ws/
└── src/
    ├── battery_python/
    │   ├── package.xml          ← ament_python
    │   ├── setup.py
    │   ├── setup.cfg
    │   └── battery_python/
    │       ├── battery_publisher.py
    │       └── battery_processor.py
    └── battery_cpp/
        ├── package.xml          ← ament_cmake
        ├── CMakeLists.txt
        └── src/
            └── battery_listener.cpp
```

Each package uses its own normal build route. `colcon` builds both as part of the workspace. They communicate through topics; they do not need to share a package to communicate. [docs.ros](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Colcon-Tutorial.html)

### Option B: One package containing both languages

```text
simple_nodes_demo/
├── package.xml                  ← ament_cmake
├── CMakeLists.txt               ← Handles both languages
├── src/
│   └── battery_listener.cpp
├── simple_nodes_demo/
│   ├── __init__.py
│   ├── battery_publisher.py
│   └── battery_processor.py
└── scripts/
    ├── battery_publisher        ← Executable wrapper
    └── battery_processor        ← Executable wrapper
```

Use `ament_cmake_python` to install the Python module directory, and explicit CMake rules to install executable wrappers. You do not need `setup.py` or `setup.cfg` for this approach. [docs.ros](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Python-Documentation.html)

The Python-related CMake rules look like:

```cmake
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/battery_publisher
    scripts/battery_processor
  DESTINATION lib/${PROJECT_NAME}
)
```

Example `scripts/battery_publisher`:

```python
#!/usr/bin/env python3

from simple_nodes_demo.battery_publisher import main

if __name__ == '__main__':
    main()
```

This wrapper performs the job that a setuptools console-script entry point performs in a pure Python package. `ament_python_install_package()` installs modules; the explicit executable installation makes them launchable through `ros2 run`. [docs.ros](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Python-Documentation.html)

# Building and running

## 14. The common workflow

From the workspace root:

```bash
source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

Then run the installed executables:

```bash
ros2 run simple_nodes_demo battery_publisher
ros2 run simple_nodes_demo battery_processor
ros2 run simple_nodes_demo battery_listener
```

Run each in a separate terminal for this example, sourcing the environment in each terminal. Sourcing changes the shell’s package and library search paths; it does not build or start nodes. [docs.ros](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Colcon-Tutorial.html)

## 15. Which additional files are optional?

| File or directory | Why you might add it | Required for a basic node? |
|---|---|---|
| `launch/*.launch.py` | Starts several executables with parameters and remappings. | No |
| `config/*.yaml` | Stores node parameters outside source code. | No |
| `msg/`, `srv/`, `action/` | Defines custom communication interfaces. | Only if needed |
| `test/` | Contains automated tests. | No |
| `.devcontainer/` or Dockerfile | Reproduces the development environment. | No—ROS does not require Docker |
| `README.md` | Explains usage. | Not required by the build |

Launch/config files need installation rules if they are to be found through the installed package. Custom interfaces need their own generation configuration. [docs.ros](https://docs.ros.org/en/humble/How-To-Guides/Developing-a-ROS-2-Package.html)

## 16. What must change when you add a node?

| Change | Required action |
|---|---|
| New Python executable in an `ament_python` package | Add the module and a `console_scripts` entry; rebuild. |
| New C++ executable | Add source, CMake target/dependencies, and install rule; rebuild. |
| New Python executable in a mixed CMake package | Add module, executable wrapper, and CMake install rule; rebuild. |
| Edit existing C++ logic | Rebuild, then restart the executable. |
| Edit existing symlink-installed Python logic | Usually restart the executable; rebuild if installation metadata changed. |
| Add a dependency | Declare it and ensure it is installed. |
| Open another terminal | Source the installed environment. |

`--symlink-install` can make ordinary Python source edits visible without copying the files again, but it is not live process reload and does not remove C++ compilation. [docs.ros](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

The essential distinction is:

```text
package.xml        → identity, dependencies, and selected build type

setup.py/setup.cfg → Python installation route
              OR
CMakeLists.txt     → C++ / mixed installation route

colcon build       → executes the selected build route

source setup.bash  → makes the installed result discoverable

ros2 run           → starts an installed executable
```
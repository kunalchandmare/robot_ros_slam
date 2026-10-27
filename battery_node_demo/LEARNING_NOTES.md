# Learning Notes: Separate Python and C++ ROS 2 Packages

## Workspace and packages

The repository root is the workspace.

`battery_node_demo/` groups two independent ROS packages: `battery_python` and `battery_cpp`.

Each package has its own package.xml and build configuration.

## Python build flow

```text
package.xml selects ament_python
    -> setup.py installs Python modules and console scripts
    -> setup.cfg places executables in lib/battery_python
    -> ros2 run battery_python battery_publisher
    -> the wrapper imports the module and calls main()
```

The resource/battery_python marker supports package discovery. The inner battery_python/__init__.py marks the Python package.

## C++ build flow

```text
package.xml selects ament_cmake
    -> CMakeLists.txt defines the executable and dependencies
    -> compiler and linker produce battery_listener
    -> install rules place it in lib/battery_cpp
    -> ros2 run battery_cpp battery_listener
    -> the executable enters main()
```

## Runtime flow

Both languages initialize ROS, create a node, and spin an executor.

The executor dispatches timer and subscription callbacks.

Python and C++ nodes communicate using compatible ROS topic names, message types, and communication settings. They do not need to be in the same package.

## Building versus sourcing

- `colcon build`: builds packages and installs their outputs into the workspace.
- `source install/setup.bash`: makes installed packages available in the current shell.
- `ros2 run`: starts an installed executable.

## When changing code

- Existing C++ logic: rebuild the package and restart the executable.
- Existing symlink-installed Python logic: usually restart the executable.
- New Python executable: register it in setup.py and rebuild.
- New C++ executable: add CMake build/install rules and rebuild.
- New external dependency: declare it and ensure it is installed.
- Ordinary source edits do not require rebuilding the Docker image.

## Git

Commit source code and build configuration. Do not commit build/, install/, or log/.

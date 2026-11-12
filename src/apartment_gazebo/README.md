# apartment_gazebo

A simple **10 m × 8 m** indoor apartment for mobile-robot navigation in
Gazebo Classic and Gazebo Sim (Harmonic and other SDF 1.7 releases).
Outer walls are closed. Interior doorways are empty gaps (no door meshes),
wide enough for a TurtleBot- or Husky-sized robot. A few static furniture
boxes act as obstacles; doorways and corridors stay clear (≥ 0.8 m).

The world file includes the wall model and each furniture model with
`model://` URIs. After you build and source this package, the environment
hook adds `share/apartment_gazebo/models` to `GZ_SIM_RESOURCE_PATH` and
`GAZEBO_MODEL_PATH` so those URIs resolve.

## The apartment

Walls are **0.15 m** thick and **2.5 m** tall. The building occupies
**x = [0, 10] m, y = [0, 8] m**. A horizontal divider at y = 3.50 m
separates the living room and kitchen (north) from the bathroom, hallway,
and bedroom (south). A vertical divider at x = 6.00 m splits living room
from kitchen and hallway from bedroom; another at x = 2.50 m splits the
bathroom from the hallway.

### Rooms (inner free space)

- **Living room**: 5.77 m × 4.27 m, x = [0.150, 5.925], y = [3.575, 7.850]
- **Kitchen**: 3.77 m × 4.27 m, x = [6.075, 9.850], y = [3.575, 7.850]
- **Hallway**: 3.35 m × 3.27 m, x = [2.575, 5.925], y = [0.150, 3.425]
- **Bedroom**: 3.77 m × 3.27 m, x = [6.075, 9.850], y = [0.150, 3.425]
- **Bathroom**: 2.27 m × 3.27 m, x = [0.150, 2.425], y = [0.150, 3.425]

### Doorways (clear opening width)

- **Living room ↔ hallway**: 1.00 m, in the wall at y = 3.50 m (x = 3.75–4.75)
- **Living room ↔ kitchen**: 1.60 m wide opening, in the wall at x = 6.00 m (y = 5.05–6.65)
- **Hallway ↔ bedroom**: 1.00 m, in the wall at x = 6.00 m (y = 1.25–2.25)
- **Hallway ↔ bathroom**: 0.90 m, in the wall at x = 2.50 m (y = 1.05–1.95)

### Furniture (static boxes)

- **Sofa** and **coffee table** in the living room, along the north wall
- **Kitchen counter** along the east wall of the kitchen
- **Bed** in the bedroom, against the south wall
- **Vanity** in the north-west corner of the bathroom

### Robot spawn

Place the robot in the living room, clear of furniture and doorways:

```
x = 4.00 m
y = 5.50 m
z = 0.05 m
yaw = 0.0 rad    # facing +X, toward the kitchen opening
```

## Build and launch

Put this package in a ROS 2 workspace, then:

```bash
cd /path/to/ws
colcon build --packages-select apartment_gazebo
source install/setup.bash
```

Sourcing the workspace is what applies the model-path hook. After that,
launch the world with Gazebo Sim / Harmonic:

```bash
gz sim $(ros2 pkg prefix apartment_gazebo)/share/apartment_gazebo/worlds/apartment.sdf
```

Or through `ros_gz_sim`:

```bash
ros2 launch ros_gz_sim gz_sim.launch.py   gz_args:=$(ros2 pkg prefix apartment_gazebo)/share/apartment_gazebo/worlds/apartment.sdf
```

Gazebo Classic alternative (also uses `GAZEBO_MODEL_PATH` from the hook):

```bash
gazebo $(ros2 pkg prefix apartment_gazebo)/share/apartment_gazebo/worlds/apartment.sdf
# or
gazebo --verbose $(ros2 pkg prefix apartment_gazebo)/share/apartment_gazebo/worlds/apartment.sdf
```

To spawn a robot at the suggested living-room pose with `ros_gz_sim`:

```bash
ros2 run ros_gz_sim create -file /path/to/robot.sdf -name robot   -x 4.00 -y 5.50 -z 0.05 -Y 0.0
```

With `gazebo_ros` on Gazebo Classic:

```bash
ros2 run gazebo_ros spawn_entity.py -entity robot -file /path/to/robot.sdf   -x 4.00 -y 5.50 -z 0.05 -Y 0.0
```

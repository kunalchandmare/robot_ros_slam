# apartment_gazebo

A simple **10 m × 8 m** indoor apartment for mobile-robot navigation, made
for ROS 2 Humble with Gazebo Fortress. It also loads in Garden and Harmonic,
and in Gazebo Classic (geometry only).
Outer walls are closed. Interior doorways are empty gaps (no door meshes),
wide enough for a TurtleBot- or Husky-sized robot. A few static furniture
boxes act as obstacles; doorways and corridors stay clear (≥ 0.8 m).

The world file includes the wall model and each furniture model with
`model://` URIs. After you build and source this package, the environment
hook adds `share/apartment_gazebo/models` to `GZ_SIM_RESOURCE_PATH`,
`IGN_GAZEBO_RESOURCE_PATH` (older Fortress releases) and `GAZEBO_MODEL_PATH`
so those URIs (and the wall textures) resolve.

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

### Textured walls and floor (for camera SLAM)

Plain one-colour walls give a stereo camera almost nothing to match, so
every room now has its own look. These are visual-only details. They have
no collisions, so walls, doorways and the spawn point haven't moved.

- **Walls**: each room's walls are covered with thin textured panels
  - Living room: red brick
  - Kitchen: colourful subway tiles up to about 1.3 m, painted plaster above
  - Hallway: blue striped wallpaper with small dots
  - Bedroom: pale green leaf pattern
  - Bathroom: blue ceramic tiles, with a few patterned ones
  
  All rooms except the bathroom have a baseboard along the bottom. The
  panels are split into pieces about 2 m wide, so the patterns stay at
  real-world size (bricks and tiles at centimetre to decimetre scale)
  instead of one image stretched across a whole wall.
- **Floors**: light wood planks in the living room and bedroom, dark wood
  in the hallway, and stone tiles in the kitchen and bathroom.
- **Posters**: six unique, colourful posters, one each in the bathroom,
  hallway, bedroom and kitchen and two in the living room. Their centres
  are at about 0.85 m, low enough for a small robot's cameras to see. They
  act as landmarks that help the robot recognise a room it has seen before
  (loop closure).
- **Lighting**: the sun casts shadows, the scene ambient light is a little
  brighter (0.5), and two soft ceiling lights without shadows keep the
  roofless rooms from going dark.

The textures are small PNG images (about 1.2 MB in total) in
`models/apartment_walls/materials/textures/`. They're used through
`<pbr><metal><albedo_map>`, which the ogre2 renderer in Gazebo Sim reads.
Gazebo Classic ignores them and shows plain colours instead.

### Simulation systems (sensors)

The world loads these Gazebo Sim systems: Physics, UserCommands,
SceneBroadcaster, **Sensors** (ogre2 renderer) and **Imu**. Without the
last two, robot cameras and IMUs don't publish anything. The plugin names
are the Fortress (`ignition-gazebo-*`) names that ship with ROS 2 Humble.
Garden and Harmonic still accept them, with a deprecation warning. Gazebo
Classic can't load these plugins, logs errors for them and skips them.

### Robot spawn

Place the robot in the living room, clear of furniture and doorways:

```
x = 4.00 m
y = 5.50 m
z = 0.05 m
yaw = 0.0 rad    # facing +X, toward the kitchen opening
```

## Build and launch

This package lives in the `src/` folder of the ROS 2 Humble workspace at
`/workspaces/ros2-dev` (the repo root is the workspace). Build it there:

```bash
cd /workspaces/ros2-dev
colcon build --packages-select apartment_gazebo
source install/setup.bash
```

Sourcing the workspace is what applies the model-path hook. After that,
launch the world with Gazebo Fortress (the version that comes with Humble):

```bash
ign gazebo $(ros2 pkg prefix apartment_gazebo)/share/apartment_gazebo/worlds/apartment.sdf
```

Or through `ros_gz_sim` (on Humble this starts Fortress too):

```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=$(ros2 pkg prefix apartment_gazebo)/share/apartment_gazebo/worlds/apartment.sdf
```

On Gazebo Garden or Harmonic (not the Humble setup), use `gz sim` instead:

```bash
gz sim $(ros2 pkg prefix apartment_gazebo)/share/apartment_gazebo/worlds/apartment.sdf
```

Gazebo Classic alternative (geometry only; also uses `GAZEBO_MODEL_PATH` from the hook):

```bash
gazebo --verbose $(ros2 pkg prefix apartment_gazebo)/share/apartment_gazebo/worlds/apartment.sdf
```

To spawn a robot at the suggested living-room pose with `ros_gz_sim`
(replace `<robot.sdf>` with your robot's SDF or URDF file):

```bash
ros2 run ros_gz_sim create -file <robot.sdf> -name robot -x 4.00 -y 5.50 -z 0.05 -Y 0.0
```

With `gazebo_ros` on Gazebo Classic:

```bash
ros2 run gazebo_ros spawn_entity.py -entity robot -file <robot.sdf> -x 4.00 -y 5.50 -z 0.05 -Y 0.0
```

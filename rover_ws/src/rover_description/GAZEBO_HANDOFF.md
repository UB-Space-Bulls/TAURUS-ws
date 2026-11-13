# Gazebo simulation and world editing handoff

Prepared 2026-10-28 for the TAURUS / Space Bulls rover workspace.

## 1. Scope and current state

Use this guide to start the existing rover simulation, drive it, inspect its
sensors, and change the environment. FAST-LIO mapping is not launched by the
current simulation launch file. Keep its integration as a separate next task.

The environment uses ROS 2 Jazzy, Gazebo Harmonic, and Zenoh inside Docker.
ROS nodes are running programs; topics are named message streams between
them. The ROS/Gazebo bridge converts Gazebo sensor messages into ROS messages.

Verified earlier in this session:

- `/imu`: 200 Hz in simulation time, frame `base_link`.
- `/lidar/points`: 10 Hz, frame `lidar_link`, cloud size 720 × 32.
- LiDAR mounting relative to `base_link`: translation `[0, 0, 0.25]` m,
  identity rotation. This is not its height above the ground.
- World: ground plane and sun; no walls or other mapping landmarks yet.
- Gazebo point clouds are instantaneous snapshots. Timing compatibility was
  tested separately; see [the timing results](../rover_localization/fast_lio_timing/RESULTS.md).

These are recorded observations, not a guarantee that the simulator is running
now. The commands below include fresh runtime checks.

## 2. Paths and responsibilities

| Location | Purpose |
|---|---|
| Host: `/home/ngtha/DEIMOS-ws` | Repository root, Docker scripts and image build context |
| Host: `/home/ngtha/DEIMOS-ws/rover_ws` | ROS workspace mounted into the container |
| Container: `/workspaces/rover_ws` | Same workspace, as seen inside Docker |
| `src/rover_description/worlds/rover_world.sdf` | Ground, obstacles, lights, physics, world systems |
| `src/rover_description/urdf/rover.urdf.xacro` | Rover geometry, joints, sensor definitions and mounting |
| `src/rover_description/launch/gazebo.launch.py` | Selects world, spawns rover, starts bridges and controls |
| `src/rover_description/config/rover_controllers.yaml` | Wheel and steering controller configuration |
| `src/rover_swerve_controller/rover_swerve_controller/swerve_kinematics_node.py` | Converts requested rover velocity into wheel/steering commands |

SDF (Simulation Description Format) is XML describing a world or model.
URDF/Xacro describes this rover; the launch converts it before spawning it.
Edit world obstacles in SDF and rover sensors/mounting in Xacro.

## 3. Open the dev environment

### Host terminal: check for an existing container

```bash
cd /home/ngtha/DEIMOS-ws
docker ps
```

The container used in this session is `rover-dev-container`. If it is already
running, open a shell in it:

```bash
docker exec -it rover-dev-container bash
```

If no dev container is running, start one from the repository root:

```bash
./docker/run-dev.sh
```

This script uses the `rover-dev` image, mounts the workspace, configures the
display, and runs as your host user to avoid root-owned build files. If the
image does not exist, build it first on the host:

```bash
docker build -f docker/dockerfile.dev -t rover-dev .
```

The startup script uses `--rm`: exiting the main container shell removes that
container. Source files in the mounted workspace persist; container-only
package installs and `/tmp` files do not. The isolated timing test installed
PCL dependencies temporarily; that does not mean a fresh image contains them.

### Every container terminal: source ROS and the workspace

```bash
cd /workspaces/rover_ws
source /opt/ros/jazzy/setup.bash
if [ -f install/setup.bash ]; then source install/setup.bash; fi
printenv ROS_DISTRO RMW_IMPLEMENTATION
```

Expected values here are `jazzy` and `rmw_zenoh_cpp`. Sourcing loads commands
and package paths into that terminal. Use the main workspace setup, not the
isolated `/tmp/fastlio-timing-test/install/setup.bash`.

The container entrypoint normally starts the Zenoh router. If ROS programs
cannot discover each other, inspect it before starting another copy:

```bash
ps -eo pid,args | grep '[r]mw_zenohd'
tail -30 /tmp/zenohd.log
```

If no router exists, run `ros2 run rmw_zenoh_cpp rmw_zenohd` in a dedicated
sourced terminal and leave that terminal open.

## 4. Build and launch

Build both rover packages on a fresh workspace. Repeat this after changing
their source or installed assets:

```bash
cd /workspaces/rover_ws
colcon build --packages-select rover_description rover_swerve_controller
source install/setup.bash
ros2 launch rover_description gazebo.launch.py
```

The current default world is `second_world.sdf`. Select the original world
with `ros2 launch rover_description gazebo.launch.py world:=rover_world.sdf`.

Keep this terminal open. The launch starts the Gazebo server and window,
robot transforms, rover spawning at world X=0, Y=0, Z=0.3 m, message bridges,
the controller spawners, and the swerve command node. Let the rover fall onto
the ground and settle before evaluating sensors.

Do not start a second copy of the launch while the first is still running.
Launching the world alone with `gz sim` does not start the ROS rover pipeline.

In Gazebo, use the play/pause control to run or pause simulation. The launch
requests running immediately with `-r`. Pausing also stops simulation time
and sensor timestamps. Rendering can be slower than real time, so 200 Hz in
simulation time need not appear as 200 messages per wall-clock second.

Official [Harmonic GUI guide](https://gazebosim.org/docs/harmonic/gui/) covers
the window controls, Entity Tree, and Component Inspector.

## 5. Drive and explicitly stop the rover

Open another container terminal and source ROS/workspace as above. Check that
all three controllers report `active`:

```bash
timeout 15 ros2 control list_controllers
```

Expected controllers: `joint_state_broadcaster`, `steer_controller`, and
`wheel_controller`. A timeout is a failed check, not confirmation of readiness.

For keyboard driving:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.2 -p turn:=0.2
```

Keep focus in that terminal and follow its displayed key map. It publishes
`geometry_msgs/msg/Twist` on `/cmd_vel`. Linear X is forward, linear Y is left,
and positive angular Z turns counterclockwise. Linear units are m/s; angular
units are rad/s. The robot's axes rotate with the rover; world axes do not.

For a small forward command without keyboard teleop, use:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

**Stop that publisher with Ctrl+C, then explicitly send zero velocity:**

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

The current swerve node has no command timeout. Ending a publisher alone may
leave the last wheel command active. Use one driving publisher at a time.

## 6. Check simulation and sensors

Run these in a sourced container terminal while the simulation is playing:

```bash
gz topic -l
ros2 topic list -t
timeout 10 ros2 topic echo /clock --once
timeout 10 ros2 topic echo /imu --once
timeout 10 ros2 topic echo /lidar/points --once --field fields
ros2 topic info /imu --verbose
ros2 topic info /lidar/points --verbose
timeout 5 ros2 run tf2_ros tf2_echo base_link lidar_link
```

`tf2_echo` reports a coordinate transform; it normally runs continuously,
so its timeout is intentional. Wait past its initial discovery message for
the actual transform output. A sensor echo timeout means data was not verified.

At rest on level ground, IMU angular velocity should be near zero and
acceleration approximately `[0, 0, +9.8]` m/s². Cloud fields currently include
XYZ, intensity, and ring, with no per-point time field. `/odom` is Gazebo
ground truth, not FAST-LIO output. `/scan` still has a bridge entry but is
not a verified sensor stream; use `/lidar/points` for 3D inspection.

To inspect the cloud visually:

```bash
rviz2 --ros-args -p use_sim_time:=true
```

Set Fixed Frame to `odom`, add a PointCloud2 display for `/lidar/points`, and
add TF if useful. If the display reports a delivery mismatch, select Best
Effort reliability. The current bridge publishers were observed as reliable.

## 7. Make a persistent world change

The launch resolves the world through the **installed package**, not directly
through the source path. The installed `worlds` directory inspected here was
a real directory, not a symlink. Use this edit/validate/build/restart cycle:

1. Stop the rover explicitly, then press Ctrl+C in the launch terminal and
   wait for shutdown.
2. Edit `src/rover_description/worlds/rover_world.sdf` in the mounted workspace.
3. Validate and rebuild inside the container:

   ```bash
   cd /workspaces/rover_ws
   gz sdf -k src/rover_description/worlds/rover_world.sdf
   colcon build --packages-select rover_description
   source install/setup.bash
   ```

4. Confirm the installed world matches the source:

   ```bash
   diff -u src/rover_description/worlds/rover_world.sdf \
     install/rover_description/share/rover_description/worlds/rover_world.sdf
   ```

5. Launch again and confirm the obstacle is visible and produces LiDAR returns:

   ```bash
   ros2 launch rover_description gazebo.launch.py
   ```

SDF validation checks the file; it does not prove rendering, contacts, or
sensor behavior. Editing a file while Gazebo runs does not reload the world.
Do not edit files under `install/` as your source of truth.

## 8. Example: add a static wall

Insert this complete block **inside `<world name="rover_world">`**, alongside
the existing ground model and light, before `</world>`:

```xml
<model name="mapping_wall_east">
  <static>true</static>
  <pose>5 0 1 0 0 0</pose>
  <link name="wall_link">
    <collision name="collision">
      <geometry>
        <box><size>0.2 6 2</size></box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>0.2 6 2</size></box>
      </geometry>
      <material>
        <ambient>0.6 0.6 0.7 1</ambient>
        <diffuse>0.6 0.6 0.7 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

`pose` is `x y z roll pitch yaw`: position in meters, angles in radians. This
model is centered at world `[5, 0, 1]`. The box dimensions are X=0.2, Y=6,
Z=2 m; its bottom rests at ground height because its center Z is half its
height. The nearer X face is at 4.9 m. Actual LiDAR range depends on sensor
pose and beam direction; it is not automatically 5 m.

`static=true` fixes the wall in place. `visual` supplies renderable geometry
for the GPU LiDAR and viewer; `collision` supplies physical contact geometry.
Keep their dimensions aligned. Give every additional model a unique name.
See Gazebo's [SDF world tutorial](https://gazebosim.org/docs/harmonic/sdf_worlds/)
for the world/model structure and [model construction tutorial](https://gazebosim.org/docs/harmonic/building_robot/)
for visual and collision elements.

For a simple mapping course, duplicate this model with unique names and poses:

| Suggested name | Pose | Box size | Purpose |
|---|---|---|---|
| `mapping_wall_north` | `0 5 1 0 0 0` | `6 0.2 2` | Perpendicular surface |
| `mapping_box_a` | `-3 2 0.75 0 0 0` | `1 1 1.5` | Distinct landmark |
| `mapping_box_b` | `2 -3 0.5 0 0 0` | `1.5 1 1` | Asymmetric landmark |

These are proposed additions, not changes already applied to the world.
Leave room around the origin for the rover to spawn and drive. Start with
simple shapes; complex terrain and meshes add contact/rendering variables.
GPU LiDAR needs visible geometry in its field of view, so a collision-only
obstacle is insufficient. Walls add vertical features beyond the ground.

Retain exactly one instance of each required existing world system: Physics,
UserCommands, SceneBroadcaster, Sensors with `ogre2`, and Imu. Keep these when
adding models; do not paste a second complete `<world>` or Sensors plugin.

## 9. Experiment with GUI editing

Pause simulation, select a model in the scene or Entity Tree, and use
Transform Control to adjust it. Harmonic shortcuts are `T` to translate,
`R` to rotate, and `Esc` to return to selection. The Inspector helps examine
properties. Toolbar shape insertion can quickly prototype obstacles.
See [Manipulating Models](https://gazebosim.org/docs/harmonic/manipulating_models/).

GUI modifications affect the current simulation. To keep them, use the file
menu's **Save World** action, which is separate from saving GUI configuration.
Save to a new file under the mounted workspace and review it before reuse.
Containers see `/workspaces/rover_ws/...`, not the host's `/home/ngtha/...` path.

An exported running world can include the spawned rover and runtime state.
Do not overwrite the hand-maintained world blindly: the ROS launch also
spawns `rover`. Copy the intended obstacle models into the source world,
preserving its plugins and excluding the runtime rover, then use the normal
validation/build/restart cycle. Review the diff to see exactly what changed.

## 10. Use a second world or external models

The launch now accepts a `world` argument and defaults to `second_world.sdf`.
For a separate scenario, put a new SDF in `worlds/`, rebuild
`rover_description`, and select it without editing Python:

```bash
ros2 launch rover_description gazebo.launch.py world:=new_world.sdf
```

Keep the world name `rover_world` unless you also update any world-specific
service commands.

All contents of `worlds/` are installed by the existing CMake install rule.
New directories outside the installed list require an install-rule change.
Fuel models or mesh files must also resolve inside the container; a host-only
path will not work. Begin with inline primitive models above for a portable
test world with no network asset dependencies.

## 11. Recovery and reproducibility

| Symptom | First check |
|---|---|
| `ros2` is missing | Source `/opt/ros/jazzy/setup.bash` inside the container |
| Rover package is missing | Build packages and source the main workspace |
| Edit does not appear | Check installed world, rebuild, stop old launch, relaunch |
| Rover does not move | Play simulation, check active controllers and `/cmd_vel` |
| Rover keeps moving | Stop other driving publishers and explicitly publish zero |
| Gazebo has sensor topics but ROS does not | Check launch bridge process, exact topic names, Zenoh discovery |
| Cloud is empty or sees only ground | Check visible obstacles, sensor field of view and range; inspect in RViz |
| GUI fails to open | Check host display/WSLg and the container's `DISPLAY`; start using the repo script |
| Physics is unexpectedly slow | Check real-time factor; reduce scene complexity before altering sensor timing |

For a clean new run, stop driving, send zero, stop the launch, and relaunch.
A GUI world reset rewinds time; restart stateful estimators such as FAST-LIO
after a time reset rather than treating it as continuous motion.

Before sharing a scenario, record the source diff, launch command, obstacle
poses, input topics, sensor configuration, and test route. Check:

```bash
git diff -- src/rover_description/worlds/rover_world.sdf \
  src/rover_description/launch/gazebo.launch.py
git diff --check
```

Run these Git commands from `/workspaces/rover_ws` as shown. No commit, push,
world modification, or new simulation launch was performed merely by writing
this handoff. The example wall is documentation, not an installed obstacle.

**TL;DR:** Enter the dev container, source ROS, build and launch the rover.
Drive through `/cmd_vel` and explicitly stop it. Add persistent obstacles to
the source world SDF, validate, rebuild `rover_description`, and relaunch.
Inspect both the Gazebo view and `/lidar/points` before using a new world for
FAST-LIO tests.

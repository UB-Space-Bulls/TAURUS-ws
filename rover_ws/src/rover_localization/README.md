# FAST-LIO simulation diagnostic

Inside the dev container, build and source the workspace:

```bash
cd /workspaces/rover_ws
colcon build --symlink-install --packages-up-to rover_localization
source install/setup.bash
ros2 launch rover_localization fast_lio_test.launch.py
```

Options: `world:=second_world.sdf`, `rviz:=false`, `comparator:=false`.
Run only one simulation launch. This includes `rover_description/gazebo.launch.py`
once, reusing its controllers, bridges and swerve node. FAST-LIO is included via
its `mapping.launch.py` with `config_path`, `config_file`, and simulation time.
RViz uses `camera_init`, registered clouds, the estimated path, and TF (coordinate
transforms). Gazebo's separate TF tree may show RViz transform warnings; the raw
LiDAR cloud and ground truth are deliberately not displayed in this fixed frame.
No diagnostic node adds edges to TF.

In a separate sourced interactive terminal, the requested basic teleop command is:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.2 -p turn:=0.2
```

For driving with this launch's watchdog, use repeat and key timeout parameters:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.2 -p turn:=0.2 -p repeat_rate:=10.0 -p key_timeout:=0.5
```

The basic command publishes only on key presses and therefore stops after the
watchdog's 0.75-second silence limit. Use `k` to stop, then Ctrl+C to exit teleop.
Explicitly send zero before shutting down the launch:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

The watchdog uses wall time and periodically publishes zero after command silence.
It remains enabled when the comparator is disabled. It cannot protect against its
own crash, a transport failure, or another publisher continuously commanding motion.
The controller itself still has no timeout. Stop all driving publishers first;
do not rely on launch shutdown to deliver a final message.

The comparator's terminal table uses estimate-minus-ground-truth errors in meters.
It records both reference poses at the same simulation timestamp, computes
`inverse(T_initial) * T_current`, and compares translations in initial body axes.
The current FAST-LIO source publishes the IMU pose as `body`; launch checks the
URDF IMU mounting and LiDAR extrinsics, and the node checks incoming frame names.
Only the verified, coincident `body == base_link` case is supported. Changed or
unknown mounting disables comparison; no guessed frame transform is applied.

Ground truth is buffered (1000 samples) and interpolated at estimate timestamps
with linear translation and shortest-arc quaternion SLERP. A bracket wider than
0.1 seconds is rejected; no extrapolation is used. The table reports the common
stamp and bracket width. Yaw is the normalized heading of the relative quaternion
error, not a subtraction of unwrapped Euler angles. Near pitch singularities it
should not be interpreted as an independent yaw measurement.

Missing sensors, odometry, clock progress, ambiguous odometry publishers and stale
data suppress numeric errors. Publisher identity changes, time regression,
wall-time gaps over 2 seconds, or translation jumps over 1 meter clear references.
These conservative resets can also discard legitimate motion or very slow runs.
An undetected in-process reset with continuous timestamps and a small pose jump is
not distinguishable from motion using these messages alone. After any known
FAST-LIO reset, explicitly reset the diagnostic:

```bash
ros2 service call /odometry_comparator/reset std_srvs/srv/Trigger '{}'
```

A reset starts a new measurement segment and discards accumulated drift. Keep the
rover stationary while FAST-LIO initializes and while references are established.
After a Gazebo time reset, restart FAST-LIO as well.

The configuration uses generic preprocessing (`lidar_type: 5`). Gazebo supplies
XYZ, intensity and ring, but no per-point timestamp. Motion compensation and
odometry accuracy have **not** been validated by this tool's implementation.
Ground truth uses Gazebo `dimensions=2`: Z/roll/pitch are not full 3D physics truth.
A numeric 3D norm is therefore a diagnostic, not a validated 3D accuracy score.

Runtime acceptance procedure (record results rather than assuming success):

1. Check `/clock`, `/imu`, `/lidar/points`, `/odom`, `/Odometry`, `/path`, and
   `/cloud_registered` with `ros2 topic info -v` and `ros2 topic echo --once`.
   Confirm odometry stamps advance in simulation time and expected frame names.
2. Keep the rover stationary after settling; observe displacement stability.
3. Drive slowly forward, stop explicitly, and compare displacement direction.
4. Turn slowly and check heading direction and normalized yaw error.
5. Run the comparator alone without publishers: it must report missing inputs.
6. Stop/restart FAST-LIO with the rover stationary: references must be cleared.
   Use the reset service for any reset the automatic heuristics cannot detect.
7. Stop teleop and verify zero `/cmd_vel` within the watchdog timeout. Send zero,
   Ctrl+C the launch, and verify no owned Gazebo/controller/mapping nodes remain.

Unit tests: `python3 -m pytest src/rover_localization/test` from the workspace.

## Validation recorded on 2026-11-13

- Docker Jazzy build with `--symlink-install --packages-up-to rover_localization`:
  all five packages built successfully (including Livox and FAST-LIO).
- Seven pytest tests passed in the container. They exercise frame invariance,
  interpolation/yaw wrapping, synthetic synchronized motion, missing inputs,
  timestamp regression and invalid-frame resets, and watchdog zero commands.
- Python compilation and Git whitespace checks passed. Launch argument discovery
  reports the requested defaults. RViz started with OpenGL and subscribed to
  `/cloud_registered`; FAST-LIO initialized with LiDAR type 5.
- The initial bounded simulation run advanced extremely slowly and reported
  absent/stale odometry and sensor data without crashing or displaying errors.
  Stationary accuracy, forward motion, turning, live TF consistency and a real
  estimator restart remain unverified. Callback tests are synthetic, not accuracy
  measurements or proof of complete ROS transport behavior.
- Final diagnostic nodes and RViz terminated cleanly on SIGINT. The inherited
  swerve node raised an already-shutdown exception; Gazebo required escalation
  and left children which were manually stopped. An earlier run also reported a
  Gazebo/Zenoh shutdown panic. Full clean simulation shutdown is **not verified**.
- A separately launched `rover_description` simulation appeared during the
  interrupted session and overlapped the final RViz smoke test. That run reported
  controller configuration failures and simulation-clock jumps, so its runtime
  data is unsuitable for accuracy evaluation. The independent simulation was left
  running; only diagnostic-test-owned processes were cleaned up.

Before acceptance testing, use a single fresh simulation session and repeat the
runtime acceptance procedure above. Do not treat a successful package build as
validated FAST-LIO odometry.

# Point-LIO in Gazebo (Sprint 2, task L2)

Point-LIO is a LiDAR-inertial odometry (LIO) algorithm: it fuses the 3D LiDAR
point cloud with the IMU to estimate where the rover is and build a point-cloud
map at the same time. This package runs the ROS 2 port
[dfloreaa/point_lio_ros2](https://github.com/dfloreaa/point_lio_ros2) (based
on Unitree's ROS 1 [point_lio_unilidar](https://github.com/unitreerobotics/point_lio_unilidar))
on the simulated rover and compares it with FAST-LIO on identical data.

**TL;DR:** Point-LIO tracks the simulated rover to 3.7 cm RMSE (0.23 % drift)
over a 13.4 m drive, about 4x more accurate than FAST-LIO on the same bag, but
uses about 15x more CPU. Gazebo's LiDAR emits `inf` points that both
estimators must not see; `lidar_cloud_filter` removes them.

## Build (inside the dev container)

```bash
./docker/run-dev.sh            # from the repo root
cd /workspaces/rover_ws
vcs import --recursive < src/third_party.repos   # --recursive: FAST-LIO needs its ikd-Tree submodule
colcon build --symlink-install --packages-up-to rover_localization
source install/setup.bash
```

## Run live in simulation

```bash
ros2 launch rover_localization point_lio_test.launch.py
```

Arguments: `world:=second_world.sdf`, `headless:=true` (no Gazebo window, works
without a display), `rviz:=false`, `comparator:=false`, `sim:=false` (see
below). Drive with teleop in a second sourced terminal, exactly as in
[README.md](README.md). RViz reuses `rviz/fast_lio_test.rviz`: Point-LIO
publishes the same `/cloud_registered`, `/path`, `/Odometry` and frames
(`camera_init` -> `body`) as FAST-LIO, so `odometry_comparator` scores either.

What the launch starts:

| Node | Job |
|---|---|
| Gazebo + bridges + controllers | `rover_description/gazebo.launch.py` (only with `sim:=true`) |
| `lidar_cloud_filter` | `/lidar/points` -> `/lidar/points_filtered`: drops `inf`/NaN points, adds `time = 0` |
| `pointlio_mapping` | Point-LIO with `config/point_lio_gazebo.yaml`; odometry remapped to `/Odometry` |
| `odometry_comparator` | Live error vs. Gazebo ground truth (`estimator_name: Point-LIO`) |
| `cmd_vel_watchdog` | Sends zero velocity if teleop goes silent |

## Why the cloud filter exists

Measured on one sim scan (720 x 32 = 23 040 points): **9 026 points are `inf`**
(rays that hit nothing) and ~2 700 are under 0.5 m (the rover's own body).
Point-LIO's Unilidar handler and FAST-LIO's generic handler both only reject
points closer than `blind`, so `inf` points pass straight into the map.

The filter also adds a per-point `time` field set to 0. That is the true
value for Gazebo, which renders a whole scan at one instant (no motion
distortion). **The real Unitree L1/L2 provides its own per-point time and must
not go through this filter.**

## Comparison with FAST-LIO (bag replay)

Both estimators replayed one recorded sim run (84 s, 13.4 m ground-truth path:
straight, 90-degree turn, sideways swerve strafe, arcs, spin in place,
reverse). Each ran alone, so no TF conflicts. Every run was repeated and gave
identical numbers.

| | Point-LIO | FAST-LIO (filtered cloud) | FAST-LIO (raw cloud, current `fast_lio_test`) |
|---|---|---|---|
| Translation RMSE | **0.037 m** | 0.158 m | 2.175 m |
| Max translation error | **0.127 m** | 0.325 m | 4.614 m |
| Final error (drift) | **0.031 m (0.23 %)** | 0.166 m (1.24 %) | 0.688 m (5.13 %) |
| Yaw error final / max | **-0.26 / 2.37 deg** | -3.51 / 5.17 deg | -90.7 / 91.0 deg |
| Tracking failures | 0 | 0 | 0 |
| Output rate | 10 Hz | 10 Hz | 10 Hz |
| CPU time for the 84 s bag | 144-158 s (~1.8 cores) | **10 s** | 8 s |

Takeaways:

- Point-LIO is clearly more accurate here, especially in heading during the
  spin-in-place and strafe segments.
- Point-LIO costs ~15x the CPU. That matters on the Orin Nano; measure it on
  the real compute before choosing.
- FAST-LIO on the raw cloud loses heading by ~90 deg. The only difference from
  the filtered run is the input topic, so **`fast_lio_test.launch.py` should
  also read `/lidar/points_filtered`** (FAST-LIO owner's call).
- Sim limits: perfect sensor timing, no IMU noise model, flat ground truth
  (`odom` is planar). Real-L1 data is the real test.

CPU was measured on an Apple Silicon Mac (arm64 Docker); Point-LIO's CMake
only enables its multi-thread option on x86, so numbers will differ on the
ThinkPad and the Orin.

### Reproduce

```bash
# 1. Record (sim only, no estimator):
ros2 launch rover_description gazebo.launch.py headless:=true
ros2 bag record -s mcap -o bags/shared_sim_run /clock /imu /lidar/points /odom
#    ...drive (teleop or a script), then Ctrl+C the recorder.

# 2. Replay into ONE estimator at a time, recording its output:
ros2 launch rover_localization point_lio_test.launch.py sim:=false rviz:=false comparator:=false
ros2 bag record -s mcap -o bags/out_point_lio /odom /Odometry
ros2 bag play bags/shared_sim_run          # do not add --clock: the bag has /clock

# 3. Score it:
ros2 run rover_localization trajectory_eval bags/out_point_lio --name point_lio
```

For FAST-LIO on the same bag, run `lidar_cloud_filter` (remap `input:=/lidar/points
output:=/lidar/points_filtered`) and `fastlio_mapping` with
`--params-file config/fast_lio_gazebo.yaml -p use_sim_time:=true -p
common.lid_topic:=/lidar/points_filtered`. `bags/` is git-ignored; share the
bag with the FAST-LIO owner directly (it is ~600 MB).

## Tuning notes (`config/point_lio_gazebo.yaml`)

- `point_filter_num: 3`, `filter_size_surf/map: 0.5` (upstream defaults).
  With 1 / 0.2 Point-LIO ran at 1.4 Hz on 10 Hz input, fell further behind
  every scan, and grew past 1.3 GB of memory.
- `acc_norm: 9.81`: the Gazebo IMU reports m/s^2. The real L1 IMU may report
  in g (then 1.0); check before the real-L1 test.
- `extrinsic_T: [0, 0, 0.25]`: LiDAR pose in the IMU (= `base_link`) frame,
  verified against the URDF at launch.
- `fov_degree: 360`, `scan_line: 32`, `det_range: 30` match the sim sensor.

## Gotchas hit while setting this up

- **Gazebo hangs on "Requesting list of world names"** when the container runs
  with `--user` (as `run-dev.sh` does): gz-transport could not build its
  default partition name. Fixed in `docker/dockerfile.dev` with
  `ENV GZ_PARTITION=rover_dev`; rebuild the image.
- Leftover bridge processes from a previous sim attach to the next Gazebo and
  double-publish `/imu` and `/clock` (Point-LIO then logs `imu loop back`).
  Check `ros2 topic info /imu` shows one publisher, or restart the container.

## Not done yet

- Demo recording (RViz screen capture) for the deliverable: needs a machine
  that can open the RViz window.
- Real Unitree L1 data: blocked until the L1 delivers scans (task H1).

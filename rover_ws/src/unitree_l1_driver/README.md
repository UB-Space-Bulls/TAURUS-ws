# unitree_l1_driver — physical L1 only

Runs the supplied Unitree L1 SDK through a supervised native reader. Publishes raw
PointCloud2, IMU and diagnostics, with optional RViz. No terrain processing,
Gazebo, map, ground mount or pose estimator is started. Linux x86_64 and aarch64
SDK archives are included; see `vendor/LICENSE` and `vendor/UPSTREAM_README.md`.

## Build

Place this folder at `~/DEIMOS-ws/rover_ws/src/unitree_l1_driver`.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/DEIMOS-ws/rover_ws
rosdep install --from-paths src/unitree_l1_driver --ignore-src -r -y --rosdistro jazzy
colcon build --packages-select unitree_l1_driver
source install/setup.bash
ros2 pkg executables unitree_l1_driver
```

Both packages can be built independently. Regular installation copies the runtime
files and avoids depending on source symlinks. The script install rules also
specify executable permissions explicitly. If choosing `--symlink-install`, keep
source/build directories in place and rebuild after changing installed scripts.

## Find the physical connection first

```bash
lsusb
ls -l /dev/serial/by-id/
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Use the actual adapter port, preferably its stable `/dev/serial/by-id/...` path.
If no port appears, watch `sudo journalctl -kf` while reconnecting the USB data
cable. Rotation alone is not a data test. A ROS rebuild cannot fix absent USB
enumeration. The last reported laptop output showed no detected serial adapter;
this split does not resolve that hardware connection by itself.

For permission errors, inspect the device group/ACL and your groups. On a usual
Ubuntu serial device owned by `dialout`, add yourself with
`sudo usermod -aG dialout "$USER"`, then log out and back in. Stop the old
`unilidar_l1_check`, Unitree viewer, SDK test and all other serial readers first.
Only one program should own this port.

## Start the L1 and raw RViz

```bash
ros2 launch unitree_l1_driver l1.launch.py port:=/dev/ttyUSB0
```

Replace the port if needed. The launch checks that the path exists, is a character
device, and is readable/writable before starting RViz. It does not automatically
choose among USB devices. The native reader opens at 2,000,000 baud and retains
the SDK and buffering workaround from your existing test package.

Arguments: `port`, `cloud_scan_num:=18`, `rviz:=true`, `publish_imu_tf:=true`.
Use `rviz:=false` when the terrain package will provide the visualization.
For stationary terrain testing, `cloud_scan_num:=90` produces larger batches
with more latency. This batches sensor scans; it does not estimate motion.

RViz uses `unilidar_lidar` as its fixed frame, so no measured world mount is
needed to view raw data. The IMU orientation display has an explicitly arbitrary
reference; it is not a level/world transform. `publish_imu_tf` publishes only the
SDK's fixed lidar-to-IMU mounting offset. Set it false if your robot already owns
that edge. There is no `terrain_map` transform in this package.

| Output | Type / meaning |
|---|---|
| `/unilidar/cloud` | PointCloud2; XYZ metres, intensity, time, ring; frame `unilidar_lidar` |
| `/unilidar/imu` | Imu; frame `unilidar_imu` |
| `/unilidar/imu_orientation` | PoseStamped for diagnostic display, not odometry |
| `/diagnostics` | DiagnosticArray; reader, cloud and IMU stream health |

Cloud and IMU QoS are Best Effort/Volatile. Headers use host receipt time with
`use_sim_time=false`; they are not hardware acquisition timestamps. The driver
reports `power_verified=false`; no voltage/current/battery charge measurement is
inferred from motor rotation or received data.

## Verify before starting terrain

In a second sourced terminal:

```bash
timeout 15s ros2 topic echo /diagnostics --once
timeout 15s ros2 topic echo /unilidar/cloud sensor_msgs/msg/PointCloud2 \
  --field width --once --qos-reliability best_effort
timeout 15s ros2 topic echo /unilidar/imu sensor_msgs/msg/Imu \
  --field header --once --qos-reliability best_effort
```

Expect positive cloud width and advancing messages. The driver intentionally
stops its reader after a fatal error/stall but stays alive to publish diagnostics;
restart after correcting the connection. A topic name or a running node alone
is not evidence of received sensor data.

For standalone SDK/wire checks, stop the ROS driver first and use
[SDK_BENCH_GUIDE.md](docs/SDK_BENCH_GUIDE.md). Software test results and limits are
in [VALIDATION.md](docs/VALIDATION.md).

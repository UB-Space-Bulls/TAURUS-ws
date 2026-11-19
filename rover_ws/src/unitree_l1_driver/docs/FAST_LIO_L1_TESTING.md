# Physical L1 FAST-LIO test

This test runs the physical L1 reader, FAST-LIO and optional RViz. The starting
configuration is the supplied `config/fast_lio_l1.yaml`, for the integrated IMU
and 18 SDK scans per cloud. It does not start Gazebo, rover controls, terrain
processing or Nav2. Physical timing, calibration and accuracy remain to be
verified with the actual sensor.

The configuration uses FAST-LIO's timed type-2 preprocessing path with feature
extraction disabled. It does not make the L1 a Velodyne sensor. In this pinned
FAST-LIO version, type 5 discards point timing; Point-LIO's type 5 has a different
meaning and must not be copied directly into this FAST-LIO configuration.

## Build

The imported FAST-LIO checkout includes a one-line local change to accept Best
Effort IMU messages. On a fresh import, apply its recorded patch once before
building:

```bash
cd ~/DEIMOS-ws/rover_ws
git -C src/fast_lio apply ../../patches/fast_lio-imu-sensor-qos.patch
```

The current checkout already has the patch. Build after changing the driver or
FAST-LIO source:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/DEIMOS-ws/rover_ws
PYTHONNOUSERSITE=1 CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
  colcon build --packages-up-to fast_lio unitree_l1_driver --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

## First verify the raw sensor

Source ROS and this workspace in each terminal. Find your adapter in
`/dev/serial/by-id/`, or use its actual `/dev/ttyUSB*` device. Keep other serial
readers and simulations stopped for this standalone test.

Terminal 1:

```bash
ros2 launch unitree_l1_driver l1.launch.py \
  port:=/dev/ttyUSB0 cloud_scan_num:=18 rviz:=true
```

Terminal 2:

```bash
ros2 run unitree_l1_driver ros_topic_check.py \
  --seconds 30 --report /tmp/l1_raw_check.json
```

Expect nonempty clouds and IMU messages with a PASS report. This tests received
data, not synchronization, calibrated noise or odometry. Stop Terminal 1 before
starting the combined launch below; it includes its own serial reader.

## Run FAST-LIO

Hold the sensor stationary on a stable support during initialization, in an
area with observable surfaces. Start:

```bash
ros2 launch unitree_l1_driver l1_fastlio.launch.py port:=/dev/ttyUSB0
```

Use `rviz:=false` for a headless test. To tune a copy of the supplied config:

```bash
ros2 launch unitree_l1_driver l1_fastlio.launch.py \
  port:=/dev/ttyUSB0 config:=/absolute/path/to/fast_lio_l1.yaml
```

The combined launch fixes the driver to `profile:=fastlio`, 18 scans/cloud and
wall time, and includes upstream `fast_lio/mapping.launch.py`. Its RViz fixed
frame is `camera_init`; inspect registered scans and the estimated path. Raw
sensor frames are not connected to the estimator TF tree by a guessed mounting
transform. FAST-LIO publishes `camera_init -> body`, where body is the IMU.

In another sourced terminal, check outputs after initialization:

```bash
timeout 15s ros2 topic echo /diagnostics --once
timeout 15s ros2 topic echo /Odometry --field header --once
timeout 15s ros2 topic echo /cloud_registered --field header --once
timeout 10s ros2 topic hz /Odometry
```

`timeout` ends the continuous rate command; exit status 124 is expected for
that command. Expected output frames are `camera_init` and odometry child
`body`. A topic name alone is not evidence of received messages.

After the estimate settles, translate slowly along a measured distance, stop,
then rotate slowly and return to the starting position. Check direction,
stationary drift, measured displacement error and map doubling/smearing.
Record results; successful initialization alone does not establish accuracy.
This launch sends no movement commands to the rover.

Optional recording, from a directory with space available; stop with Ctrl+C:

```bash
ros2 bag record -o l1_fastlio_test \
  /unilidar/cloud /unilidar/imu /Odometry /cloud_registered /path /tf /diagnostics
```

## What the FAST-LIO profile changes

- Headers retain each SDK `stamp` instead of using Python receipt time. Cloud
  point `time` values remain seconds relative to that SDK cloud stamp, as
  declared by the bundled SDK. No independent clock offset is invented.
- Points are sorted by time, and ring is checked and converted from UINT32 to
  UINT16. XYZ, intensity and relative times are preserved, including nonzero
  first-valid-point offsets. Feature extraction remains disabled.
- Empty clouds, invalid/negative/excessive offsets, all-zero timing and ring
  overflow fail visibly. This prevents the Velodyne yaw-based timing fallback.
- SDK stamps must advance per stream and be within 1 second behind / 50 ms
  ahead of ROS wall time. Out-of-window or repeated/regressed stamps stop the
  reader and produce an ERROR diagnostic. Restart the driver and FAST-LIO
  together after correcting the source; do not conceal a reset by restamping.
- Both FAST-LIO sensor subscriptions accept Best Effort messages. The cloud
  publisher's point stride remains 24 bytes, with UINT16 ring at offset 20.

These clock checks screen gross errors; they do not prove that SDK LiDAR and
IMU stamps share calibrated acquisition time. Check their actual relationship,
scan duration and buffering on hardware before relying on deskew. The configured
zero fixed time offset is a starting value, not a measured result. IMU noise
settings and the 10 m detection range also require evaluation on the sensor.

The extrinsic is LiDAR origin in the IMU frame, not the rover mount height.
Its sign agrees with the inverse of the integrated IMU origin documented in
the [Unitree SDK](https://github.com/unitreerobotics/unilidar_sdk) and with the
[Unitree L1 Point-LIO reference](https://github.com/unitreerobotics/point_lio_unilidar/blob/main/config/unilidar_l1.yaml).
No rotor/yaw model is used to create missing L1 point times.

## Terrain and Nav2

The existing terrain adapter can consume `/cloud_registered` with tracking
frame `body` after FAST-LIO is working. It additionally needs a verified level
target frame and measured body ground clearance; see
[FASTLIO_INTEGRATION.md](../../terrain_geometry_physical/terrain_geometry/docs/FASTLIO_INTEGRATION.md).
Nav2 still needs the correct rover mounting and map/odom/base frame chain.
The current simulated terrain/Nav2 launch continues to use Gazebo odometry.

## Software checks

After building, run the driver contract and synthetic serial tests:

```bash
cd ~/DEIMOS-ws/rover_ws
PYTHONNOUSERSITE=1 L1_READER_DIR="$PWD/install/unitree_l1_driver/lib/unitree_l1_driver" \
  python3 -m unittest discover -s src/unitree_l1_driver/tests -v
```

The bounded ROS integration check feeds synthetic serial packets through the
actual vendor SDK, driver and FAST-LIO. Use a ROS domain without other nodes:

```bash
ROS_DOMAIN_ID=192 PYTHONNOUSERSITE=1 \
  python3 src/unitree_l1_driver/tests/smoke_fastlio_ros.py
```

It checks the cloud field layout, ordered point times, received IMU, advancing
finite odometry and shutdown. Logs/results are written to
`/tmp/l1_fastlio_runtime.log` and `/tmp/l1_fastlio_runtime_result.json`.
Synthetic scans do not validate L1 optics, SDK acquisition timing on the device,
IMU calibration, motion compensation accuracy or physical navigation.

Recorded on 18 November 2026: all three required packages built on Jazzy,
20 driver tests passed, and the 18-second ROS integration run received 172
clouds, 3091 IMU messages and 169 odometry messages with advancing stamps.
The launch and its children shut down cleanly. The fixture uses broad synthetic
scan geometry; it is not an L1 accuracy measurement. No physical L1 was attached
for these checks.

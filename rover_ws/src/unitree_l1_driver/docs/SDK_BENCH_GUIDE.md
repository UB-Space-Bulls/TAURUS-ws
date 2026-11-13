# Unitree L1: power, serial data, point-cloud and IMU bench checks

This package tests a **physical Unitree L1 using its USB/serial adapter**. It uses the supplied Unitree SDK 1.0.16 directly, then optionally publishes to ROS 2 for RViz. It does not use Gazebo, SLAM, a ZED camera, or the L2 SDK.

Start with the SDK test. A successful USB connection alone does not prove that the L1 is powered or producing usable readings.

## 1. What each check establishes

| Check | Evidence | What it cannot establish |
|---|---|---|
| Linux USB/serial detection | Adapter appears and port can open | Sensor supply health or valid data |
| Direct SDK test | Nonempty finite clouds; valid IMU values; rates, gaps, IDs, advancing timestamps | Supply voltage/current; distance accuracy; calibration |
| Passive wire test | Incoming bytes, valid MAVLink packets, observed CRC errors, raw auxiliary values | Fully usable cloud geometry or every possible lost byte |
| ROS topic test | Subscribers receive usable cloud/IMU messages with compatible QoS | Device sequence loss hidden before publishing |
| RViz/manual movement | Scene changes and IMU orientation responds to motion | Calibrated mapping or absolute heading |
| External electrical measurement | Voltage/current behavior at the power path | Whether decoded point clouds are correct |

**Power limitation:** the SDK has no input-supply-voltage, current, or battery-charge getter. The protocol's `apd_voltage` is a photodetector-related value, **not the 12 V supply**. Reports always include `power_verified: false`. A steady stream supports an inference that the device remained operational; it does not certify electrical stability.

## 2. Connect and check power

Use the L1's supplied conversion module, appropriate **12 V DC supply**, and a known data-capable USB cable. Unitree's documented topology is L1 → adapter → USB to PC, with a separate external power input to the adapter. Do not assume the USB cable powers the L1. Use the supplied power adapter/rating; the L1 manual's troubleshooting guidance specifies 12 V, 1 A.

1. Secure the sensor on a stable surface with open space around its scanning area. Place a wall or box roughly 1–3 m away.
2. Connect the L1 to the correct adapter, the supply to the adapter, and USB to the computer. Initially avoid hubs and extension cables.
3. Observe startup/rotation and the serial port. LEDs or motor activity alone are insufficient; continue with the data test.
4. For a real power test, use the supply's voltage/current display or a suitable inline DC meter on the **12 V path**, logging startup and steady scanning. A USB meter only measures the USB side. Look for voltage drops or current-limit events that coincide with lost data or restarts. Use the supply's specified limits rather than an invented software tolerance.
5. If using a multimeter, measure voltage in parallel at an accessible, correctly identified power connector. Current measurement needs an appropriate series connection; never put a current-mode meter across the supply. Avoid probing the small live sensor connector where a slip can short pins.

Do not wire 12 V to the data pins. The sensor's UART signals are 3.3 V logic. For this bench procedure, retain the manufacturer adapter instead of making a new cable. See the linked manual for connector orientation/pin numbering.

In a terminal:

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
sudo dmesg --follow
```

Reconnect USB once while watching the kernel log. Identify the new port; repeated disconnect/reconnect messages are a transport/power-path clue. A charge-only USB cable will not enumerate a serial adapter. For stable naming, prefer the appropriate `/dev/serial/by-id/...` path when available.

If you lack serial permissions:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in, then check `id -nG`. Do not run the entire ROS stack as root. Before **each** test, stop the original Unitree viewer/driver and other serial readers:

```bash
fuser -v /dev/ttyUSB0
```

Replace `/dev/ttyUSB0` throughout with your actual port. Only one test/driver can own the port. These tools lock against each other; unrelated programs may ignore the advisory lock.

## 3. Build and run the direct SDK test — no ROS needed

Supported target: Linux x86_64 or aarch64. The archive retains the supplied vendor static library for each architecture. Instructions assume Ubuntu; native Ubuntu 24.04 / ROS 2 Jazzy is the intended ROS target.

Extract the updated ZIP into a new directory. It contains one folder named `unitree_l1_driver`:

```bash
mkdir -p ~/l1_bench
unzip ~/Downloads/unitree_l1_driver.zip -d ~/l1_bench
sudo apt update
sudo apt install build-essential python3 cmake
cd ~/l1_bench/unitree_l1_driver
chmod +x build_standalone.sh
./build_standalone.sh
```

The build compiles two native readers and runs software tests, including synthetic serial packets. **Those tests do not validate your physical sensor.**

Run the real sensor check:

```bash
python3 scripts/l1_check.py \
  --port /dev/ttyUSB0 \
  --warmup 10 --seconds 60 \
  --report reports/sdk_60s.json
echo $?
```

The default setup is **2,000,000 baud, 8-N-1, 18 scans per cloud**. The reader requests NORMAL mode once and continuously drains the SDK. It does not run an LED show, cycle through standby, or wait for a firmware-version packet. `--passive` suppresses the NORMAL request if you want to observe an already-running device without that command.

The script waits up to 10 seconds for initialization, excludes 10 seconds of warmup, then measures for 60 seconds. It prints rates/gaps approximately once per second and saves a JSON report. A missing or stalled cloud/IMU stream ends the test after 3 seconds of silence during measurement. A blocked SDK read is terminated by the parent process. It does not auto-reconnect and conceal the interruption.

| Default SDK pass condition | Threshold |
|---|---|
| Cloud rate | At least 5 messages/s |
| IMU rate | At least 100 messages/s |
| Largest observed gap in each stream | At most 0.5 s, including start/end of measurement |
| Point-cloud data | Nonempty clouds and no nonfinite point values |
| IMU payload | Finite quaternion/gyro/acceleration; quaternion norm 0.8–1.2 |
| IMU IDs | No observed duplicates/skips; 16-bit wrap handled |
| SDK timestamps | Finite, positive and strictly advancing in both streams |
| Completion | Full requested measurement; no reader error or malformed records |

These are **bench screening thresholds**, not manufacturer acceptance limits. At 18 scans/cloud, roughly 10 clouds/s and 250 IMU messages/s are useful nominal expectations, not guaranteed rates under every host condition. Changing `--scans` changes cloud rate: adjust `--min-cloud-hz` accordingly. `--min-imu-hz`, `--max-gap`, and `--stall-timeout` are also adjustable. Do not lower thresholds just to hide an unexplained fault.

Exit code `0` means all configured checks passed; `1` means failure/incomplete measurement; `2` covers invalid command arguments or a report-write error. Early Ctrl+C produces an incomplete, failing report. Even a PASS is limited to that observation window and is not a calibration or power certificate.

Useful report fields: `checks`, `cloud.rate_hz`, `imu.rate_hz`, `max_gap_s`, `age_s`, `estimated_missing_ids`, `stamp_regressions`, `valid_points`, `empty_clouds`, `invalid_imus`, `last_imu`, and `worker_log`. Cloud ID discontinuities are informational because aggregation can affect ID semantics; IMU IDs follow the documented 16-bit wire packet counter. ID jumps can indicate loss or resets, and do not identify the cause by themselves.

## 4. Inspect bytes and CRCs directly

After the SDK check has exited, run:

```bash
python3 scripts/l1_check.py \
  --mode wire --port /dev/ttyUSB0 \
  --warmup 1 --seconds 30 \
  --report reports/wire_30s.json
```

This is a **separate, passive** serial reader using the vendor's MAVLink headers. It sends no mode commands. If the L1 was left in standby, run the SDK test first so it requests NORMAL mode. Never run wire and SDK/ROS tests together.

The report counts received bytes, valid frames, bad-CRC frames, and range/IMU/auxiliary packets. Byte gaps are measured with a monotonic host clock and reported in periodic snapshots. Unlike the SDK measurement, wire totals include the warmup period. A wire PASS requires all three packet types, zero observed CRC failures and acceptable byte gaps. It does **not** establish a sustained per-type rate or decoded geometry; the SDK test does that.

The latest auxiliary record also exposes raw working status, motor periods, temperatures, optical communication quality and APD voltage. The package does not invent status meanings or engineering thresholds for undocumented fields. Optical-quality fields concern the sensor's internal optical communication, not a USB signal-quality percentage. CRC errors suggest damaged/misaligned frames or a protocol mismatch; a zero CRC count does not prove that no packets were lost.

## 5. Check consistency over time and detect a disconnect

With the sensor stationary, run a 10-minute soak:

```bash
python3 scripts/l1_check.py --port /dev/ttyUSB0 \
  --warmup 10 --seconds 600 --report reports/sdk_10min.json
```

Keep `sudo dmesg --follow` and any external power logging running in separate terminals. Save a short note with the supply, USB cable, port, host, test duration, and observed voltage/current range. Compare faults by timestamp. This separates “the adapter disconnected,” “bytes stopped,” “clouds stopped but IMU continued,” and “ROS did not receive data.”

As a separate **intentional failure test**, start a 60-second SDK run, let both streams appear, then unplug only the USB data connection. It should fail within roughly the 3-second stall timeout plus process scheduling/cleanup time, or immediately on a read error. Do not expect PASS for this test. Reconnect, re-identify the port, and start a fresh run. The software leaves the L1's working mode unchanged on exit; it may continue scanning until its supply is turned off.

## 6. ROS 2 build and RViz

First stop all direct SDK/wire tests. Use a small separate workspace for the first hardware check, so your Gazebo and rover drivers cannot publish competing data:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install python3-colcon-common-extensions \
  ros-jazzy-ament-cmake ros-jazzy-ament-index-python \
  ros-jazzy-rclpy ros-jazzy-sensor-msgs ros-jazzy-diagnostic-msgs \
  ros-jazzy-geometry-msgs ros-jazzy-launch-ros ros-jazzy-tf2-ros ros-jazzy-rviz2
mkdir -p ~/l1_ros_ws/src
cp -a ~/l1_bench/unitree_l1_driver ~/l1_ros_ws/src/
cd ~/l1_ros_ws
colcon build --packages-select unitree_l1_driver --symlink-install \
  --cmake-args -DL1_BUILD_ROS2=ON
source install/setup.bash
ros2 launch unitree_l1_driver l1.launch.py port:=/dev/ttyUSB0
```

Alternatively put just `unitree_l1_driver` in your existing `~/DEIMOS-ws/rover_ws/src` and build that package. Do not run the old Unitree driver or Gazebo bridge on the same topics during the hardware check. For headless operation append `rviz:=false`.

| Topic | Type/purpose |
|---|---|
| `/unilidar/cloud` | `sensor_msgs/msg/PointCloud2`; x/y/z/intensity/time/ring |
| `/unilidar/imu` | `sensor_msgs/msg/Imu`; SDK quaternion, angular velocity and acceleration |
| `/unilidar/imu_orientation` | `geometry_msgs/msg/PoseStamped`; orientation illustration in an arbitrary reference |
| `/diagnostics` | Initialization, cumulative stream health, or explicit ERROR on reader exit/stall |

The launch provides the documented LiDAR-to-IMU static transform. RViz's **Fixed Frame is `unilidar_lidar`**, so cloud viewing does not require `map`, `odom`, or SLAM. Clouds and IMU use best-effort, volatile sensor-data QoS. If you add a display manually, set its Reliability Policy to **Best Effort**.

The ROS node uses **host receipt timestamps** for easy bench visualization. Source SDK timestamps are still checked in diagnostics; they are not used for ROS message headers. Point-relative `time` is preserved, but receipt-stamped headers are not a synchronized measurement-time contract for motion compensation. Covariances are unknown (all zeros). The quaternion is normalized only after a plausibility check. Treat this as a bench driver, not a tuned sensor-fusion driver.

### Visual check

1. A stationary sensor should show recognizable walls/floor/boxes, with new points arriving continuously. The default cloud decay is zero so old scans cannot hide a freeze.
2. Move a box in the field of view. Its returns should move promptly. Stop moving it and confirm the display stabilizes.
3. Slowly tilt/rotate the securely held sensor. The IMU orientation axes should change smoothly; inspect `/unilidar/imu` at the same time. The orientation display uses an arbitrary reference and is not a heading/pose estimate in a mapped world.
4. Return it to the original position. Acceleration and gyro should respond sensibly to movement and settle when stationary. For ROS IMU conventions, acceleration is m/s² and gyro rad/s; a stationary gravity-inclusive accelerometer is normally near 9.81 m/s² in magnitude. This driver passes the SDK values through, matching the supplied wrapper; verify the behavior/scale on your hardware before fusion.

Increasing cloud decay can make the scene denser, but moving the L1 will smear accumulated points because this package does not compensate sensor motion. Keep decay at zero for a freshness test.

### Automated ROS subscriber check

While the launch remains running, open a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/l1_ros_ws/install/setup.bash
ros2 topic info /unilidar/cloud --verbose
ros2 topic info /unilidar/imu --verbose
ros2 run unitree_l1_driver ros_topic_check.py \
  --warmup 5 --seconds 60 --report ~/l1_ros_report.json
ros2 topic echo /diagnostics --once
ros2 topic echo /unilidar/imu --once --qos-reliability best_effort
```

The topic test examines **all received points**, checks fields/layout/frame IDs, IMU values, received rates/gaps, and advancing ROS stamps. It uses matching sensor QoS. Its sequence counters are local receipt counters because these ROS messages do not carry the original device IDs; use the direct SDK report or `/diagnostics` for device-ID evidence. A ROS topic test alone cannot expose points that the driver already removed as nonfinite—diagnostics retain that count.

The ROS node remains alive after a stream failure so `/diagnostics` can continue showing ERROR. It stops the SDK child, publishes no replacement/fake sensor data, and requires a launch restart after you fix the connection.

Optional recording for comparison or a demo:

```bash
ros2 bag record -o l1_bench_recording \
  /unilidar/cloud /unilidar/imu /unilidar/imu_orientation \
  /diagnostics /tf_static
```

Capture a brief RViz screen recording showing the stationary scene, moving box, and sensor tilt. Keep the JSON reports alongside it.

## 7. Troubleshooting

| Symptom | Next useful check |
|---|---|
| No serial port | USB data cable, adapter connection, host USB port, kernel log; supply separately |
| Permission denied | `dialout` membership after a fresh login; device permissions |
| Port in use | Stop other tools; use `fuser -v`; run only one serial owner |
| Port opens but zero bytes | L1 supply, sensor-to-adapter connection, correct port; USB enumeration proves only the adapter |
| Bytes but no valid packets | Correct L1 protocol and 2 Mbaud adapter, cable/signal quality; raw CRC report |
| IMU works, no cloud | Standby mode, motor/obstruction, cover/environment; SDK NORMAL request, then raw range/aux counts |
| Clouds empty | Actual objects in view, cover cleanliness, physical scan path; avoid an empty/very distant scene |
| SDK succeeds, ROS fails | Workspace sourcing, one publisher, matching ROS_DOMAIN_ID, host load and QoS |
| ROS receives, RViz blank | Fixed frame, PointCloud2 topic, Best Effort, display status, view distance |
| Rates/gaps fail under load | Repeat without RViz/recording; compare direct SDK vs ROS to isolate host/serialization load |
| High dirt percentage | Inspect/clean per manufacturer guidance; SDK percentage is informational, not a supply reading |
| Cumulative warning remains after recovery | The run records earlier gaps/errors; restart a fresh run after fixing the cause |

## 8. Package review and validation scope

Removed: embedded Git metadata, stale build trees/binaries, Python caches, ROS 1 wrapper, original ROS 2 wrapper, Windows executables/DLLs, PCL adapter and unrelated UDP examples. Retained: the supplied x86_64/aarch64 SDK libraries, needed headers/MAVLink dialect, vendor license and reference documentation.

Replaced the original examples' indefinite version/auxiliary waits and unsafe “print 10 points” indexing. The new reader checks initialization, requests NORMAL once, continuously parses instead of throttling the blocking SDK in a ROS timer, and lets a supervising process enforce timeouts. There is no PCL dependency.

Synthetic serial testing also exposed a buffering problem in the supplied SDK: `runParse()` waits for new serial bytes before draining packets already in its internal queue. Coalesced reads can cause a backlog and dropped readings. `src/sdk_serial_drain.cpp` is a small, version-specific GNU linker shim: it bypasses that wait so the SDK can check available bytes and drain its queue, with a short sleep on NONE to avoid busy-spinning. Both builds enable it. The vendor archives themselves remain unchanged. This shim is tied to the included 1.0.16 archives; revalidate it before replacing the SDK. It is not an upstream Unitree patch. The synthetic full-stream regression fails without it and passes with it.

**Verified during preparation:** native x86_64 compilation/linking, standalone/CMake build, Python syntax, launch/RViz configuration structure, software health checks and synthetic PTY tests through the actual SDK archive. Synthetic tests exercise cloud/IMU decoding, timestamp precision, CRC failures, port locking and timeout behavior. See `docs/VALIDATION.md` for the exact run results.

**Still to verify on your machine:** real L1 readings and power, aarch64 compilation, ROS 2 colcon/runtime integration, RViz rendering and actual sustained sensor rates. No L1 or ROS installation was available in the preparation environment. The SDK libraries are vendor prebuilts; their internal implementation is not supplied.

## Sources

- Supplied archive: `vendor/include/unitree_lidar_sdk.h`, `vendor/SDK_README.md`, `vendor/PROTOCOL.md`, `vendor/UPSTREAM_README.md`, and MAVLink headers. These define the API, frame offset and packet fields used here.
- [Unitree L1 SDK](https://github.com/unitreerobotics/unilidar_sdk)
- [Unitree L1 manual](https://oss-global-cdn.unitree.com/static/52b72f707b304d229d4321eea223738f.pdf): supply and connector definitions. Confirm the rating on your supplied adapter.
- [Unitree Unilidar user manual](https://oss-global-cdn.unitree.com/Lidar/Unilidar%20User%20Manual_v1.0.pdf): adapter/USB/external-power topology.
- [Unitree download center](https://www.unitree.com/download/LiDAR/)
- [ROS sensor_msgs/Imu definition](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Imu.msg): units and unknown-covariance semantics.

Prepared 2 November 2026. This is a focused test harness around the supplied L1 SDK, not an official Unitree release.

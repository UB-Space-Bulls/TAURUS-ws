# ZED 2 bringup for DEIMOS

Starts a physical ZED 2 through the separately installed Stereolabs wrapper.
Enables RGB, depth, a registered colored point cloud, IMU, and visual inertial
odometry. Includes an RViz preset. Does not start SLAM or navigation.

**Status: prepared and statically checked; hardware acceptance is pending.**
No ROS installation, Jetson, camera, or graphical RViz session was available
during preparation. Configured rates are starting points, not measurements.

## Files and placement

Paths are relative to the DEIMOS repository root, not `rover_ws`.

| Action | Path | Purpose |
| --- | --- | --- |
| Add | `rover_ws/src/rover_bringup/package.xml` | Declare the package and runtime dependencies. |
| Add | `rover_ws/src/rover_bringup/CMakeLists.txt` | Install launch, YAML, RViz, and this README. |
| Add | `rover_ws/src/rover_bringup/launch/zed2.launch.py` | Start the wrapper with project settings and optional RViz. |
| Add | `rover_ws/src/rover_bringup/config/zed2.yaml` | Configure the five streams and initial rates. |
| Add | `rover_ws/src/rover_bringup/rviz/zed2.rviz` | Show RGB, depth, point cloud, TF, and odometry. |
| Add | `rover_ws/src/rover_bringup/README.md` | Setup boundaries and hardware acceptance instructions. |
| Edit | `README.md` | Add the link to these instructions. |

Suggested branch: `zed-setup/bringup`. This is a configuration-only `ament_cmake`
package, so no `setup.py`, `setup.cfg`, or Python resource marker is needed.
No wrapper source files are supplied or changed.

## Changes from Lyra

Reviewed `src/bringup/config/zed2.yaml`, `src/bringup/launch/zed_rtab.launch.py`,
and `src/bringup/config/zed_sim.yaml`. Retained the project-owned configuration
pattern, camera name `zed2`, and HD720 capture at 30 FPS. Changes:

- Use `ros_params_override_path`, which the reference wrapper declares, instead
  of Lyra's `config_path` argument.
- Use `/**` as the ROS parameter selector and supported parameter names.
  In the reference release, the IMU switch is `sensors.publish_imu`, not
  `sensors.publish_imu_data`.
- Use `NEURAL_LIGHT` instead of copying the old `PERFORMANCE` depth setting.
  Leave image controls at supported defaults instead of copying negative values.
- Explicitly enable all five required publishers. Keep IMU updates independent
  of image rate with `sensors_image_sync: false`.
- Set TF publication switches in launch arguments because upstream launch
  arguments override the corresponding YAML values.
- Do not copy RTAB-Map, its startup timer, or its database-deletion option.
- Do not copy `zed_sim.yaml`: arbitrary Gazebo image-topic names do not establish
  a supported ZED simulation input. This launch uses the live USB camera.

## Check hardware and versions before installing

The files target the **interface** of wrapper **v5.2.0**, commit
`506e04757fdee442f055ddf280dfd36875732623`. Its README requires ZED SDK **5.2**
and lists Jazzy on Ubuntu 24.04. This is a reproducible reference, not a claim
that it is the latest release or a validated Jetson deployment. If using another
release, verify its launch arguments, parameters, topics, and SDK requirements.

DEIMOS currently contains conflicting platform information:

- `CLAUDE.md`: Orin Nano 8 GB, JetPack 6.x, approximately 6.1 / L4T 36.4,
  Ubuntu 22.04 host, Jazzy intended to run in a container.
- Root `README.md` and `docker/dockerfile.jetson`: JetPack 7.2 assumptions;
  the Dockerfile defaults to `nvcr.io/nvidia/l4t-jetpack:r39.2.0` and includes
  an Isaac ROS repository.

Run these on the **Jetson host** and record their output:

```bash
cat /proc/device-tree/model
cat /etc/nv_tegra_release
cat /etc/os-release
uname -m
dpkg-query -W nvidia-jetpack nvidia-l4t-core
nvcc --version
lsusb
lsusb -t
```

A missing `nvidia-jetpack` meta-package or `nvcc` does not identify the installed
driver/runtime version; record that fact and use the L4T release/package data.
Confirm **ZED 2**, not ZED 2i or ZED X, from the label and SDK tools; save its serial.

| Observed environment | Decision |
| --- | --- |
| JetPack 6.1 / L4T 36.4 | NVIDIA documents Ubuntu 22.04 and CUDA 12.6. Match the SDK installer to the exact installed L4T. |
| Ubuntu 22.04 host | Do not install Jazzy's Ubuntu 24.04 deb packages directly on it. |
| Jazzy container on JetPack 6.x | Validate container OS, ARM64 SDK, CUDA/TensorRT and host driver together. Containerization alone does not establish GPU compatibility. |
| Ubuntu 24.04 computer with a supported NVIDIA GPU | Native Jazzy is a candidate for a first camera test using the matching desktop SDK/CUDA combination. This does not accept the Jetson deployment. |

Record actual board, host OS, L4T, CUDA/TensorRT, SDK installer/version, wrapper
tag/commit, ROS distribution, and container OS/image digest. Verify that an
appropriate SDK installer exists in Stereolabs' download selector before
installing. Do not substitute an unverified nearby Docker base-image tag.

### Existing files to change after that check

| File | Required follow-up |
| --- | --- |
| `docker/dockerfile.jetson` | Replace the stale base and ROS installation assumptions with the validated Jazzy/GPU environment; add matching SDK/runtime dependencies. Remove the obsolete Isaac ROS repository if following the current `CLAUDE.md` plan. Merely changing `r39.2.0` to an Ubuntu 22.04 L4T tag leaves the Jazzy apt install invalid. |
| `docker/README.md` | Document tested image/SDK, GPU and camera-device access, and Zenoh connection commands. Verify both camera and IMU access inside the container; one `/dev/video*` mapping may be insufficient. |
| `README.md` | Reconcile JetPack 7.2 / Isaac ROS claims with observed hardware and current architecture. This patch only adds the ZED guide link. |
| `CLAUDE.md` | Replace approximate hardware versions with verified values and list `rover_bringup` when accepted. |
| `rover_ws/src/rover_description/urdf/rover.urdf.xacro` | Later, add the measured camera mount and agree who publishes the camera subtree; do not invent mounting coordinates for a bench test. |

The portable dev image already includes ROS base, Nav2, RViz, and Zenoh. Keep
SDK/wrapper hardware dependencies in the camera's validated runtime. The new
`package.xml` records the wrapper runtime dependency; whole-workspace dependency
installation may therefore require the separately provisioned wrapper.

## First test the SDK outside ROS

Install the SDK selected for the observed platform using Stereolabs' instructions.
Connect the camera over USB 3 and run the installed SDK tools:

```bash
/usr/local/zed/tools/ZED_Diagnostic
/usr/local/zed/tools/ZED_Explorer
/usr/local/zed/tools/ZED_Depth_Viewer
```

Check recognition, stereo images, depth, and cloud. If containerized, also verify
the SDK can open the camera inside the intended container, proving GPU/USB access
and library compatibility there. Close the tools before launching ROS. Complete
any initial neural-model downloads/optimization before measuring steady-state rates.

## Build and launch

The wrapper and its dependencies must already be installed in the same Jazzy
environment. This package neither installs nor modifies them. Source Jazzy and
any separately built ZED workspace first in every terminal. An underlay is another
ROS workspace whose installed packages this workspace uses.

```bash
source /opt/ros/jazzy/setup.bash
# If ZED is in a separate workspace, source its install/setup.bash here.

# From the DEIMOS repository root:
cd rover_ws
colcon build --symlink-install --packages-select rover_bringup
source install/setup.bash

ros2 pkg prefix zed_wrapper
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 --show-args
ros2 launch rover_bringup zed2.launch.py --show-args
```

This selected-package build only installs the configuration package; it does not
build or validate the wrapper. Check upstream declares `ros_params_override_path`,
`publish_tf`, `publish_map_tf`, `publish_urdf`, `publish_imu_tf`, and `enable_ipc`.

Use the same `RMW_IMPLEMENTATION` and `ROS_DOMAIN_ID` for camera, CLI, and RViz.
DEIMOS uses `rmw_zenoh_cpp`, the communication transport. For a single-computer
bench test, if the entrypoint has not already started a router, run in a separate
sourced terminal and leave running:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Use the team's router/endpoints for cross-computer tests; a local router does not
configure the rover-to-base-station link. In another sourced terminal, run **one**
of these camera commands:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch rover_bringup zed2.launch.py

# Alternatively, with a graphical session:
ros2 launch rover_bringup zed2.launch.py rviz:=true

# Append serial_number:=YOUR_ACTUAL_SERIAL to select a specific camera.
```

For remote RViz, build/source this configuration package on the viewing computer,
establish ROS/Zenoh connectivity, and run:

```bash
rviz2 -d "$(ros2 pkg prefix --share rover_bringup)/rviz/zed2.rviz"
```

The preset uses fixed frame `zed2_odom`, two Image displays, PointCloud2, TF, and
Odometry. Open the RGB and Depth panels if the RViz window layout hides them.
Sensor displays use Best Effort reliability and Volatile durability. Quality of
service (QoS) specifies delivery behavior; incompatible settings can leave a
display empty even when its topic exists.

## Verify settings and five streams

Topics are named channels for data. Some ZED outputs are generated only while
subscribed; RViz, `echo`, and `hz` create subscriptions. Listing a topic alone does
not prove data is flowing.

```bash
ros2 node list
ros2 param get /zed2/zed_node general.pub_frame_rate
ros2 param get /zed2/zed_node depth.point_cloud_freq
ros2 param get /zed2/zed_node sensors.publish_imu
ros2 param get /zed2/zed_node pos_tracking.odometry_frame
ros2 param get /zed2/zed_node pos_tracking.publish_map_tf
ros2 topic list -t
```

Expected parameter values: `15.0`, `5.0`, `true`, `zed2_odom`, `false`. If different,
check the override-path log, sourced workspace, and wrapper version.

| Output | Topic | Message type | Configured rate |
| --- | --- | --- | --- |
| RGB | `/zed2/zed_node/rgb/color/rect/image` | `sensor_msgs/msg/Image` | 15 Hz |
| Depth | `/zed2/zed_node/depth/depth_registered` | `sensor_msgs/msg/Image` | 15 Hz |
| Point cloud | `/zed2/zed_node/point_cloud/cloud_registered` | `sensor_msgs/msg/PointCloud2` | 5 Hz |
| IMU | `/zed2/zed_node/imu/data` | `sensor_msgs/msg/Imu` | 100 Hz |
| Odometry | `/zed2/zed_node/odom` | `nav_msgs/msg/Odometry` | Tracking-dependent; measure it |

RGB calibration should also appear at `/zed2/zed_node/rgb/color/rect/camera_info`.
Names match the reference release; inspect `topic list -t` if using another version.

Inspect publishers and QoS:

```bash
ros2 topic info /zed2/zed_node/rgb/color/rect/image --verbose
ros2 topic info /zed2/zed_node/depth/depth_registered --verbose
ros2 topic info /zed2/zed_node/point_cloud/cloud_registered --verbose
ros2 topic info /zed2/zed_node/imu/data --verbose
ros2 topic info /zed2/zed_node/odom --verbose
```

Sample messages without dumping large pixel/cloud arrays:

```bash
ros2 topic echo /zed2/zed_node/rgb/color/rect/image --once --field header --qos-reliability best_effort
ros2 topic echo /zed2/zed_node/depth/depth_registered --once --field encoding --qos-reliability best_effort
ros2 topic echo /zed2/zed_node/point_cloud/cloud_registered --once --field fields --qos-reliability best_effort
ros2 topic echo /zed2/zed_node/imu/data --once --qos-reliability best_effort
ros2 topic echo /zed2/zed_node/odom --once --qos-reliability best_effort
```

Expect `32FC1` depth in meters, XYZ/color cloud fields, nonempty frame IDs and
advancing timestamps. IMU acceleration includes gravity; a stationary camera need
not report zero acceleration, but angular velocity should be near zero.

Run each command separately for about 30 seconds, then stop it with Ctrl+C:

```bash
ros2 topic hz /zed2/zed_node/rgb/color/rect/image
ros2 topic hz /zed2/zed_node/depth/depth_registered
ros2 topic hz /zed2/zed_node/point_cloud/cloud_registered
ros2 topic hz /zed2/zed_node/imu/data
ros2 topic hz /zed2/zed_node/odom
```

These are receiver-side rates affected by load and network throughput. If `hz`
is silent, inspect `ros2 topic hz --help` and endpoint QoS; use supported QoS
options to match the publisher. Record measured rates against the configured
targets and agree performance tolerances with the team; require continuous
delivery without sustained stalls for the basic data-path check.

## Motion and RViz acceptance

1. Hold the camera still facing a textured, well-lit scene. Confirm current RGB,
   useful depth, and a nonempty cloud representing real geometry.
2. Place an object at two measured distances within the SDK's usable depth range.
   Confirm depth changes correctly and cloud geometry matches. Record measured
   versus estimated distances; normalized image brightness alone is not an
   accuracy measurement.
3. Slowly translate and rotate the camera. Check all streams update, gyro responds,
   and IMU orientation/odometry change consistently. Move a known short distance
   and return near the start; record odometry drift.
4. In RViz verify RGB, depth, colored 3D cloud, and healthy display status. Check
   TF (the relationships between coordinate frames):

   ```bash
   ros2 run tf2_ros tf2_echo zed2_odom zed2_camera_link
   ros2 run tf2_ros tf2_echo zed2_camera_link zed2_imu_link
   ```

5. Run at least five minutes with all required streams subscribed. Record rates,
   thermal/power mode, warnings, and disconnects. Tune publication rates/resolution
   if necessary and repeat. Lower publication rate alone need not reduce SDK
   capture/depth computation. Never report configured rates as measurements.
6. Attach RViz screenshots, SDK diagnostics, launch logs, topic/rate results, and
   environment details to the task/PR before claiming hardware acceptance.

| Acceptance record | Result on hardware |
| --- | --- |
| Board / camera model / serial | Pending |
| Host OS / JetPack / L4T / CUDA / TensorRT | Pending |
| Container OS / image digest, if applicable | Pending |
| SDK installer/version / wrapper tag and commit | Pending |
| SDK camera/depth checks | Pending |
| RGB / depth / cloud / IMU / odometry received Hz | Pending |
| Depth distances / motion response / odometry drift | Pending |
| RViz images / cloud / transforms | Pending |
| Five-minute stability | Pending |
| Evidence location / tester / date | Pending |

## Later integration with the rover

The default is a **standalone camera test**: the camera publishes
`zed2_odom -> zed2_camera_link`, with no `map -> odom` transform. `zed2_odom`
differs from the rover's future `odom` frame; the topic stays `/zed2/zed_node/odom`.
Before attaching the camera to the rover frame tree, disable camera motion TF:

```bash
ros2 launch rover_bringup zed2.launch.py publish_tf:=false
```

Have localization own rover motion and publish the measured fixed mount from
`base_link` to `zed2_camera_link`. Feed camera odometry to localization using its
real frames and calibration; renaming a frame to `base_link` does not transform
the measurement. Do not give the camera both `zed2_odom` and `base_link` as parents.

Keep `publish_urdf:=true` if the wrapper supplies internal optical transforms.
Use `publish_urdf:=false` only once the rover description provides that complete
subtree. In the reference release, disabling motion TF also suppresses SDK IMU
TF; provide the calibrated camera-to-IMU transform from a single agreed publisher
before using IMU data in the rover frame tree.

For integrated RViz, select the rover's connected `odom` or `base_link` fixed
frame. Disable the bench Camera odometry display if the independent camera
odometry frame is not connected. Do not invent an identity transform to hide
an RViz warning.

## Sources and inspected revisions

- DEIMOS: `f1920c1de3f0cfbc03fbfaaf010c403499853f60`.
- Lyra: `af1049678138989f5e0f531db950ad75152145e1`.
- [Wrapper v5.2.0 launch](https://github.com/stereolabs/zed-ros2-wrapper/blob/v5.2.0/zed_wrapper/launch/zed_camera.launch.py).
- [Wrapper v5.2.0 parameters](https://github.com/stereolabs/zed-ros2-wrapper/blob/v5.2.0/zed_wrapper/config/common_stereo.yaml).
- [Wrapper v5.2.0 prerequisites](https://github.com/stereolabs/zed-ros2-wrapper/blob/v5.2.0/README.md).
- [NVIDIA JetPack 6.1 details](https://developer.nvidia.com/embedded/jetpack-sdk-61).
- [Stereolabs Jetson SDK installation](https://docs.stereolabs.com/docs/development/zed-sdk/linux/work-with-nvidia-jetson/).
- [SDK downloads](https://www.stereolabs.com/developers/release).
- [Stereolabs ROS 2 containers](https://docs.stereolabs.com/docs/integrations/docker/create-a-ros-2-docker-image).

Plain-language summary: add `rover_bringup`, provision the compatible vendor
stack separately, and use these checks to prove the actual camera works.

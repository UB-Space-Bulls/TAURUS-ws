# Unitree 4D LiDAR L1 integration

The demonstration now targets **L1 PM** by default, with **L1 RM** selectable.
It is a documented Gazebo approximation plus a physical-driver launch path;
it is not an official Unitree simulator or a calibrated digital twin.

## Sensor settings and their limits

| Property | L1 reference | Implementation |
|---|---|---|
| Native field of view | 360° azimuth, 90° upper hemisphere | Azimuth −π…π, elevation 0…π/2 in sensor coordinates |
| Minimum range | 0.05 m | 0.05 m; close-range loss of accuracy is not reproduced |
| Maximum range | PM 20 m / RM 30 m at 90% reflectivity | Variant-selectable geometric cutoff; no reflectivity-dependent dropout |
| Effective output | 21,600 points/s | 240×90 attempted rays per 1 Hz raster snapshot; actual returns depend on scene |
| Mirror rates | Horizontal 11 Hz, vertical 180 Hz | Not reproduced by this regular raster |
| Range resolution | 8 mm | SDF range metadata; no claim of enforced output quantization |
| Accuracy | ±2 cm under specified vendor test conditions | Chosen Gaussian noise σ=1 cm for testing; not an equivalent vendor error model |
| IMU reporting | 250 Hz | Ideal Gazebo IMU at 250 Hz; no bias/drift calibration |
| Device size / mass | 75×75×65 mm, 230 g | Stationary virtual sensor only; no housing, dynamics or self-occlusion model |
| “4D” | XYZ position plus grayscale/reflectivity | Terrain uses XYZ only; Gazebo intensity is not calibrated L1 grayscale |

The **1 Hz raster is an explicit stationary-demo choice**, representing a
one-second point budget as an instantaneous image. It is not a hardware
packet rate, does not reconstruct the nonrepeating mirror path, and is not
suitable for testing motion distortion or LIO. There is no temporal map
accumulation in the terrain processor. A more faithful future simulator needs
the optical/mirror geometry, beam times and measured scan distribution.

The core defaults are in `terrain_geometry_impl/l1_profile.py`; the world generator
uses them. Tests check generated PM/RM worlds against that profile. This
single-package bundle does not modify a rover URDF or Xacro.

## Orientation and frame conventions

The official L1 frame origin is at the bottom mounting-surface center, +x
points away from the cable, and +z points upward in the device's normal stance.
The native field of view is above that surface. A level upright L1 on a mast
therefore does not scan the ground below it.

For this test stand, `terrain_map → unilidar_lidar` has translation `(0,0,1.2)` m
and **roll=π, pitch=yaw=0**. This inverted installation directs the hemisphere
at the terrain. The Gazebo model pose and static TF use the same rotation.
A rover installation needs its own measured mounting transform and clearance
checks. Never copy the test-stand TF to hardware unless it matches the
measured installation.

`unilidar_lidar → unilidar_imu` has the SDK offset
`(-0.007698,-0.014655,0.00667)` m and aligned axes. The simulation publishes:

- `/unilidar/cloud`: PointCloud2, frame `unilidar_lidar`.
- `/unilidar/imu`: Imu, frame `unilidar_imu`.

Gazebo's `/unilidar/points` is bridged to `/unilidar/cloud`. The processor
transforms XYZ into the level reference before computing slope. It does not
derive attitude from the IMU; that remains a separate localization concern.
Ray casting starts at a virtual sensor-frame origin; optical-origin offsets
and real lens geometry are not modeled.

## Simulation startup

```bash
ros2 launch terrain_geometry demo.launch.py l1_variant:=PM
# Or, after stopping the PM demo:
ros2 launch terrain_geometry demo.launch.py l1_variant:=RM
```

For a rover simulation that already supplies a 3D cloud, use
`terrain.launch.py` and a valid level-frame transform. The included
`terrain_rover.sdf` contains terrain only; it does not spawn a rover or sensor.

## Physical L1 path

Use Unitree's **unilidar_sdk** repository for L1, not the L2 SDK. The inspected
upstream commit was `1bd7d95d8ab7ce7a22058d2bb07e39fd62612aa6`. Its README lists
Foxy validation, so its build/runtime on Jazzy still needs verification.

Build upstream in its original directory layout: its CMake file references
`../../../unitree_lidar_sdk`. Moving only the ROS package loses SDK headers
and the architecture-specific library. From a separate parent directory:

```bash
git clone https://github.com/unitreerobotics/unilidar_sdk.git
cd unilidar_sdk/unitree_lidar_ros2
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
source /absolute/path/to/your_workspace/install/setup.bash
```

After establishing the **measured** level-frame TF and selecting a terrain
configuration with the correct known ground height:

```bash
ros2 launch terrain_geometry l1_hardware.launch.py \
  port:=/dev/ttyUSB0 l1_variant:=PM target_frame:=your_level_frame \
  config:=/absolute/path/to/your_terrain.yaml
```

This launches the official serial driver and the processor, with simulation
time disabled. It creates no sensor mounting TF. `cloud_scan_num` defaults
to upstream's 18; the vendor packet grouping differs from the simulated
one-second raster. Inspect observed density before tuning grouping/resolution.
Sparse real cells must remain unknown. The node retains the source timestamps;
verify that driver timestamps and ROS time share an epoch before processing.

Do not run the physical driver and simulator together on these same topics.
Original intensity/ring/time fields remain on `/unilidar/cloud`; separated
terrain visualization clouds contain XYZ only. No real-device testing or
Jazzy-driver build has been completed in this authoring environment.

## Primary sources

- [Unitree L1 product specifications](https://www.unitree.com/LiDAR/).
- [Unitree L1 User Manual v1.1, June 2024](https://oss-global-cdn.unitree.com/static/52b72f707b304d229d4321eea223738f.pdf), printed pp. 6–9 and 14–15.
- [Official L1 SDK and coordinate definitions](https://github.com/unitreerobotics/unilidar_sdk).
- [Official ROS 2 wrapper](https://github.com/unitreerobotics/unilidar_sdk/tree/main/unitree_lidar_ros2/src/unitree_lidar_ros2), including its launch parameters and header implementation.

# rover_nav2tag — DEIMOS Autonomous Marker Navigation

`rover_nav2tag` is the autonomous fiducial-marker targeting and navigation package for the Space Bulls DEIMOS Mars Rover (ROS 2 Jazzy). Defaults to **ArUco** markers (frames named `aruco_<id>`); AprilTag still works via settings.

---

## Architecture Overview

```
Requested Tag (param or /target_tag_id)
                ↓
Scan TF tree & filter Tag ID (ignore decoy tags)
                ↓
Transform tag position into Nav2 'map' frame
                ↓
Calculate 0.5m standoff pose (facing the tag)
                ↓
Send goal to Nav2 (navigate_to_pose action)
                ↓
Rover navigates and stops 0.5m in front of tag
```

---

## Key Features

1. **Requested Tag Filtering**:
   - Only navigates to the requested tag ID.
   - If other tags appear (e.g., Tag 3), the node logs that it's ignoring them and continues searching for the requested ID (e.g., Tag 5).
   - If `target_tag_id` is set to `-1`, it operates in wildcard mode (accepts any tag, like Lyra's original behavior).

2. **Dynamic Target Selection**:
   - Change targets at runtime via the `/target_tag_id` topic (`std_msgs/msg/Int32`) or ROS 2 parameters (`target_tag_id`).
   - If the target changes while navigating, the active Nav2 goal is cancelled immediately and the new tag is targeted.

3. **0.5m Standoff Calculation**:
   - Uses the tag's heading in the `map` coordinate frame to back off exactly 0.5m from the tag.
   - Sets the rover's target orientation to face directly toward the tag.
   - Which side is "the front" comes from the marker's z-axis, so `marker_z_axis` must match the detector (see Marker settings).

6. **Stale-Marker Filtering**:
   - TF keeps a marker's last pose forever, so markers not seen within `max_tag_age` seconds are ignored instead of chased.

4. **Visual Debugging**:
   - Publishes the target goal to `/nav2tag/current_goal` (`geometry_msgs/msg/PoseStamped`) for inspection in RViz2.

5. **Standalone Fake Tag Testing**:
   - Includes `fake_tag_publisher` to test the node without waiting for the camera or perception team.

---

## Marker settings

| Parameter | Default | Meaning |
|---|---|---|
| `tag_family` | `aruco` | Frame-name prefix: `aruco` matches `aruco_14` / `aruco:14`. Use `tag36h11` for AprilTag. |
| `marker_z_axis` | `out` | `out` = z points out of the marker face (OpenCV ArUco). `in` = into the face (AprilTag). Wrong value → rover parks behind the marker. |
| `max_tag_age` | `1.0` | Seconds; ignore markers not seen more recently than this. `0` disables. |

**Needs from the detector:** one TF frame per visible marker (e.g. `<camera optical frame> → aruco_14`), with its full 3D pose. Pixel-only detections (`/detections`) aren't enough.

---

## Usage

### 1. Build the package
From the workspace root:
```bash
colcon build --packages-select rover_nav2tag
source install/setup.bash
```

### 2. Standalone testing with fake tags
Launch both the navigation node and the fake tag publisher:
```bash
ros2 launch rover_nav2tag nav2tag.launch.py launch_fake_tags:=true target_tag_id:=5
```

### 2b. Watch it in RViz (one command, any laptop with Docker)
From the repo root on your host (needs the `rover-dev` image built once):
```bash
rover_ws/src/rover_nav2tag/scripts/view_sim.sh       # or: view_sim.sh 3  to target tag 3
```
Opens RViz in your browser at http://localhost:6080/vnc.html?autoconnect=1&resize=scale
showing the fake rover, both tags, and the computed goal (red arrow). No Nav2 runs,
so nothing drives. Stop with `view_sim.sh stop`.

### 3. Change the requested tag at runtime
Via topic:
```bash
ros2 topic pub --once /target_tag_id std_msgs/msg/Int32 "{data: 3}"
```
Or via parameter:
```bash
ros2 param set /navigate_to_tag target_tag_id 3
```

### 4. Running tests
Unit tests:
```bash
python3 rover_ws/src/rover_nav2tag/test/test_nav2tag.py
```
Integration flow tests:
```bash
python3 rover_ws/src/rover_nav2tag/test/test_integration_flow.py
```

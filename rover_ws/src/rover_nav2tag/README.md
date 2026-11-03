# rover_nav2tag — DEIMOS Autonomous AprilTag Navigation

`rover_nav2tag` is the autonomous AprilTag targeting and navigation package for the Space Bulls DEIMOS Mars Rover (ROS 2 Jazzy).

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

4. **Visual Debugging**:
   - Publishes the target goal to `/nav2tag/current_goal` (`geometry_msgs/msg/PoseStamped`) for inspection in RViz2.

5. **Standalone Fake Tag Testing**:
   - Includes `fake_tag_publisher` to test the node without waiting for the camera or AprilTag team.

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

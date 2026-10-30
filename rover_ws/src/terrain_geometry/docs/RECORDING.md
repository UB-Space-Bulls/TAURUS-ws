# Capture the P3 demonstration

Launch the isolated demo and confirm its input frame and TF using the README.
Use your desktop's screen recorder or OBS to capture a 45–60 second video.
A ROS bag is additional replayable evidence; it is not a screen recording.

Suggested sequence:

1. Show the Gazebo scene: flat floor, 15° wedge, uneven mesh and 0.7 m box.
2. Show RViz ground/obstacle points, green/red. Toggle unclassified returns.
3. Show height, then slope (degrees), then roughness (metres). The saved displays
   have names and fixed scales. Disable the other grid displays while switching.
4. Show gray unknown cells, including the blind area and occluded space behind
   objects. Explain that missing returns do not establish safe ground.
5. Show the live checker output and its measured ramp slope. Capture failures
   as failures; do not substitute the offline JSON for this result.

In another sourced terminal, from your workspace root:

```bash
ros2 run terrain_geometry check_demo --seconds 20 > terrain_live_checks.json
bash "$(ros2 pkg prefix --share terrain_geometry)/scripts/record_demo.sh" terrain_demo_bag 60
ros2 bag info terrain_demo_bag
```

Keep the video, live JSON, bag metadata and the tested Git commit hash together.
Check the video plays and the bag includes `/unilidar/cloud`, `/terrain/grid`,
`/tf_static` and `/clock`. If bag QoS does not match the deployment's middleware,
inspect `ros2 topic info -v` and configure recorder QoS for that deployment.

For visual replay without starting Gazebo or another processor:

```bash
# Terminal 1 (clock is already in the recorded bag; do not also generate --clock)
ros2 bag play terrain_demo_bag
# Terminal 2, source both ROS and this workspace first
rviz2 -d "$(ros2 pkg prefix --share terrain_geometry)/config/terrain.rviz" \
  --ros-args -p use_sim_time:=true
```

Start RViz before playback if the bag is short. Replay publishes the recorded
outputs; it is not a new algorithm validation. This workspace must be sourced
so `terrain_geometry/msg/TerrainGrid` is available to tools inspecting the grid.

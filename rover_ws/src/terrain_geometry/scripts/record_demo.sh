#!/usr/bin/env bash
# Record live ROS evidence, NOT a screen video. Start demo.launch.py separately.
set -euo pipefail
output_dir="${1:-terrain_demo_$(date +%Y%m%d_%H%M%S)}"
seconds="${2:-60}"
if [[ ! "$seconds" =~ ^[1-9][0-9]*$ ]]; then
  echo 'Duration must be a positive integer number of seconds.' >&2
  exit 2
fi
set +e
timeout --signal=INT --kill-after=10s "${seconds}s" \
  ros2 bag record --use-sim-time -o "$output_dir" \
  /clock /tf /tf_static /unilidar/cloud /unilidar/imu \
  /terrain/grid /terrain/ground_points /terrain/obstacle_points \
  /terrain/unclassified_points /terrain/grid_points \
  /terrain/unknown_cells /terrain/costmap_preview
status=$?
set -e
if [[ $status -ne 0 && $status -ne 124 && $status -ne 130 ]]; then
  exit "$status"
fi
echo "Recording stopped. Verify: ros2 bag info '$output_dir'"

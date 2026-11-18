# Move from the combined demo to two physical packages

The ZIPs contain `unitree_l1_driver/` and `terrain_geometry/`. Each is one ROS
package, placed directly under `src`. The old simulation package has the same
`terrain_geometry` name; keep its backup outside the workspace's `src` tree.
Existing `~/l1_mount.yaml` and `~/l1_terrain.yaml` remain usable.

Stop existing ROS launches and serial test programs first. Download both new ZIPs
to `~/Downloads` with the exact names below. Then:

```bash
(
set -e
cd ~/DEIMOS-ws/rover_ws
split_backup=$(mktemp -d "$HOME/l1-package-backup.XXXXXX")
for package in terrain_geometry unitree_l1_driver; do
  for folder in src build install; do
    if [ -e "$folder/$package" ] || [ -L "$folder/$package" ]; then
      mkdir -p "$split_backup/$folder"
      mv "$folder/$package" "$split_backup/$folder/"
    fi
  done
done
unzip ~/Downloads/unitree_l1_driver.zip -d src
unzip ~/Downloads/terrain_geometry_physical.zip -d src
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/unitree_l1_driver src/terrain_geometry \
  --ignore-src -r -y --rosdistro jazzy
colcon build --packages-select unitree_l1_driver terrain_geometry
printf 'Previous package files: %s\n' "$split_backup"
)
```

Only continue after both packages build successfully. Source `install/setup.bash`
in every new terminal. Old sourced prefix warnings can occur when the backed-up
install directories are removed; do not launch from a failed build.

Terminal 1 (use the actual detected serial port):

```bash
source /opt/ros/jazzy/setup.bash
source ~/DEIMOS-ws/rover_ws/install/setup.bash
ros2 launch unitree_l1_driver l1.launch.py \
  port:=/dev/ttyUSB0 cloud_scan_num:=90 rviz:=false
```

Terminal 2 (after verifying raw clouds and preparing measured YAMLs):

```bash
source /opt/ros/jazzy/setup.bash
source ~/DEIMOS-ws/rover_ws/install/setup.bash
ros2 launch terrain_geometry terrain.launch.py \
  mount_file:=$HOME/l1_mount.yaml config:=$HOME/l1_terrain.yaml
```

For L1-only viewing, run terminal 1 with `rviz:=true` and leave terrain stopped.
For a different existing driver, use only terminal 2 and provide its topic/frame.
Do not run the old `unilidar_l1_check` at the same time as `unitree_l1_driver`.
The new terrain package has no `l1_hardware.launch.py` or `demo.launch.py`.

Message fields and units are unchanged. The physical default grid is now
40x40 at 0.3 m resolution (12x12 m). Consumers must read actual dimensions/origin
from each message, particularly when reusing an older custom YAML.

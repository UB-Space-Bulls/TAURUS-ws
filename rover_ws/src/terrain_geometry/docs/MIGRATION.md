# Moving from the earlier two-package bundle

For a new workspace, simply put `terrain_geometry` in `src` and follow the
README. This archive has one package manifest and no repository patches.

If the previous bundle is already installed:

1. Back up and move its `src/terrain_geometry` folder outside the workspace
   before extracting this version. Renaming it inside `src` still leaves a
   duplicate package for colcon to discover.
2. The old `terrain_geometry_msgs` package is no longer needed by this node.
   Keep it only if other software or old recordings still require the old
   message type; otherwise move it outside `src` too.
3. Use a fresh terminal and source `/opt/ros/jazzy/setup.bash`. Because the build
   type changed from `ament_python` to `ament_cmake`, build into fresh output
   directories instead of reusing the old package's build/install artifacts:

   ```bash
   colcon build --symlink-install --packages-select terrain_geometry \
     --build-base build_terrain_single --install-base install_terrain_single
   source install_terrain_single/setup.bash
   ros2 launch terrain_geometry demo.launch.py
   ```

   Use that same build base for `colcon test`. A new workspace also avoids the
   old artifacts and can use the normal README commands.

The ROS message type changes from `terrain_geometry_msgs/msg/TerrainGrid` to
**`terrain_geometry/msg/TerrainGrid`**. Field definitions and `/terrain/grid`
stay the same, but ROS treats the qualified names as different message types.
Update and rebuild any Nav2 adapter/subscriber:

```python
from terrain_geometry.msg import TerrainGrid
```

C++ consumers use `terrain_geometry/msg/terrain_grid.hpp`,
`terrain_geometry::msg::TerrainGrid`, and a dependency on `terrain_geometry`.
Existing bags retain their original type; they are not converted by this change.

The standalone demo needs no `rover_description` edits. If you already applied
the earlier optional rover sensor patches, they remain separate integration
work and are not included or undone by this archive. Run either the standalone
sensor demo or your rover's L1 source, not both on the same topics/TF frames.

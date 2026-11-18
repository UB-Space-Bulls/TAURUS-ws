# Validation — 15 November 2026

Passed offline:

- Original binary STL size and triangle count; source SHA-256 recorded.
- Detailed tile contains 31,578 triangles from the original inside the selected XY bounds.
- Both output meshes load, have finite coordinates, and have the documented bounds.
- Detailed tile face normals all face upward. Overview is an approximation and may change tiny features.
- Default scanner lies above the interpolated central tile surface.
- Python launch/node syntax, package XML, YAML/RViz and generated SDF XML parse.
- Gazebo visual and collision use the same selected mesh and scale.
- Source mesh preserved; no hidden flat floor or filled holes added.
- ZIP integrity and executable entry-point permission checked.

Not run: colcon/ament build, RViz, Gazebo Harmonic, GPU rendering, ROS bridge,
physical L1 or real Nav2 integration. ROS 2 and Gazebo are not installed in this
runtime. Runtime success and scan rate must be checked on the Jazzy computer.
XML parsing is not a full SDFormat schema or Gazebo runtime test.

Source units, geographic location, vertical datum and original scan method are
unknown. Scale=1 assumes metres; it does not verify them. Neither the mesh nor
these software checks establish real-world traversability.

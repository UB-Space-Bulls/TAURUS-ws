# arm_assembly_2_description

URDF + meshes for the arm, auto-exported from the SolidWorks arm assembly
(`sw2robot`). Like `rover_description`, treat this as an **early/interim
export** — link names, masses, joint limits, and origins are provisional
until they come from released-for-manufacture CAD.

View it in RViz (with sliders for every joint):

```bash
ros2 launch arm_assembly_2_description display.launch.py
```

## Meshes: `.dae` vs `.stl`

Every part in `meshes/` exists twice, and the URDF uses each copy for a
different job:

| File   | Used by URDF tag | What reads it                         | Needed onboard the rover? |
|--------|------------------|---------------------------------------|---------------------------|
| `.dae` | `<visual>`       | RViz, Gazebo rendering (looks only)   | **No**                    |
| `.stl` | `<collision>`    | MoveIt 2 collision checking, physics  | **Yes**                   |

`.dae` (COLLADA) files carry colors/materials so the arm looks like the CAD
model; they're the bulk of this package's size (~87 MB of ~136 MB). Nothing
that runs on the rover itself needs them — `robot_state_publisher` only
reads the URDF text, and MoveIt 2 plans against the `<collision>` (`.stl`)
geometry. They only matter on machines that *render* the arm: dev laptops
and the base station (RViz), and Gazebo.

So when deploying to the Orin / ThinkPad, the `.dae` files can be left out
to save space. **Do not drop the `.stl` files** — without them MoveIt 2 has
no collision geometry for the arm.

## Naming

Links and joints are numbered along the arm, base to gripper. Every name
starts with `arm_` so nothing clashes with the rover chassis's own
`base_link` (etc.) once the arm is mounted on it. Each mesh file is named
after the link that uses it.

| Joint    | Type       | Moves          | What it is                        |
|----------|------------|----------------|-----------------------------------|
| —        | —          | `arm_link0`    | Base rail (root link)             |
| `arm_j1` | prismatic  | `arm_link1`    | Carriage sliding along the rail   |
| `arm_j2` | revolute   | `arm_link2`    | Shoulder → upper arm              |
| `arm_j3` | revolute   | `arm_link3`    | Elbow → forearm                   |
| `arm_j4` | revolute   | `arm_link4`    | Wrist pitch                       |
| `arm_j5` | continuous | `arm_link5`    | Wrist roll → gripper housing      |
| `arm_end_effector_joint` | revolute | `arm_end_effector_*` | Gripper open/close |

Only `arm_j1`–`arm_j5` and `arm_end_effector_joint` are driven. The rest are **passive**: they use
`<mimic>` to follow a driven joint automatically, and are named after it:

- `arm_j3_rod`, `arm_j3_crank` (`arm_link3_rod`, `arm_link3_crank`): the
  elbow's drive linkage, follows `arm_j3`.
- `arm_end_effector_*_joint` (links `arm_end_effector_{left,right}_{crank,finger,jaw,pad}`):
  the gripper linkage, follows `arm_end_effector_joint`, which drives the
  left crank directly. `arm_end_effector_pad` is one mesh shared by both pads.

## Notes on this export

- **Names were changed by hand after the export.** SolidWorks gave most
  parts timestamp names (`c_2026_10_13_17_13_30_019_1`, `Assembly43_1`,
  ...) and one part was named `FUCK_1` (now `arm_link1`). A fresh export
  will bring the old names back unless the parts are renamed in CAD.
  "Left"/"right" on the gripper is assigned arbitrarily for now.
- `package.xml` maintainer/license are still the exporter's placeholders.

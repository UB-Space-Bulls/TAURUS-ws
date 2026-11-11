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

## Notes on this export

- The SolidWorks part originally named `FUCK_1` was renamed to
  `arm_slider_1` (link and mesh files). It's the carriage on the prismatic
  joint off `base_link`. Rename the part in SolidWorks too, or the next
  re-export will bring the old name back.
- Most link names are SolidWorks auto-generated timestamps
  (`c_2026_10_13_...`). Worth giving the parts real names in CAD before the
  next export so MoveIt 2 configs are readable.
- `package.xml` maintainer/license are still the exporter's placeholders.

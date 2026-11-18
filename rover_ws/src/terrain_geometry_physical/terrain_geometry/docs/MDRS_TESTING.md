# MDRS historical LiDAR terrain test data

Actual USGS airborne LiDAR points around the Mars Desert Research Station near
Hanksville, Utah. This is a **600 × 600 m sample around the habitat**, not the full
MDRS region or confirmed 2027 competition course. The survey is from **2018**,
published in 2019; the site may have changed. It is useful for testing terrain
geometry and map visualization. It cannot replace rover sensor testing.

## Included data

- `data/mdrs_local.npz`: **3,439,281 points**, arrays `xyz` (N×3 float32 metres)
  and `classification` (uint8); includes ground and nonground returns.
- `data/mdrs_crop.laz`: same cropped points in projected map coordinates, with
  classifications, intensity, return numbers, airborne GPS time and source IDs.
  Other source LAS attributes are omitted/defaulted. These are derived files,
  not original scan packets.
- `provenance/source.json`: frame, origin, sources, filtering, each downloaded
  octree node's URL and SHA-256 checksum.
- `provenance/source_tiles.json`: native project tile metadata documenting CRS.
- `overview.png`: elevation overview with the three example patch locations.
- `examples/`: generated terrain results, metrics and preview figures.
- `tools/`: reproducible downloader, evaluator/config generator and overview plot.

The source had 2,788,925 class-2 ground points and 650,356 class-1 points in this
crop after filtering. Class 1 means unclassified, **not automatically obstacle**.
Withheld, nonfinite and noise classes 7/18 are excluded. Do not discard all
nonground points before running the terrain processor.

## Coordinates

`mdrs_map` axes are local projected grid East, North, Up. They are formed by
subtracting `[518169.08589079534, 4250934.474917951, 1372.96]` metres from native
NAD83(2011) / UTM zone 12N XYZ. Horizontal CRS is EPSG:6341; vertical reference
is NAVD88 height (Geoid12B in the project metadata). The crop's approximate center
is latitude 38.4064583, longitude −110.7919028. The Z offset is median surveyed
ground elevation within 20 m of that center, not a sensor height.

The public EPT source stores horizontal coordinates in EPSG:3857. The download
script converts XY into UTM before localizing them; **Web Mercator metres must
not be treated as ground-distance metres for slope estimation**. Z is preserved
from the source. PROJ uses its installed datum operations, and the chosen center
is approximate. This dataset is not geodetic control. The local frame is not
FAST-LIO `camera_init`, rover `odom`, Unity world, or a GNSS odometry frame.

## Offline test, no ROS needed

Extract this archive to `~/mdrs_site` and the terrain package to your rover
workspace. Run from the extracted dataset directory:

```bash
sudo apt-get install python3.12-venv
cd ~/mdrs_site
python3 -m venv .venv
env -u PYTHONPATH .venv/bin/python -m pip install -r requirements.txt
env -u PYTHONPATH .venv/bin/python tools/evaluate_patch.py \
  --terrain-source ~/DEIMOS-ws/rover_ws/src/terrain_geometry_physical/terrain_geometry \
  --x 0 --y 150 --output ./my_patch
```

`python3.12-venv` provides `ensurepip` on Ubuntu 24.04. If environment creation
fails, fix that step before installing requirements. Re-running `python3 -m
venv .venv` completes an incomplete environment; deleting it is not required.
The explicit `.venv/bin/python` commands use that environment without activation
and avoid installing into externally managed system Python. Activation with
`source .venv/bin/activate` is optional after creation succeeds.

The source path above matches this workspace's current package location. If
your package is elsewhere, pass the directory containing `terrain_geometry_impl`.
Keep the offline analysis dependencies in the venv; mixing a user-installed
NumPy with Ubuntu's Matplotlib can produce binary compatibility errors.
`env -u PYTHONPATH` prevents a sourced ROS workspace from injecting its Python
packages into the offline environment. It applies only to each analysis command
and does not change the terminal's ROS setup.

This writes `my_patch/terrain_preview.png`, `terrain_result.npz`, `metrics.json`
and a `patch.yaml` containing the absolute data path on **your** computer.
Rerun the evaluator after moving the data. Included example outputs deliberately
omit host-specific generated YAML files.

| Example | Center XY (m) | Known / total cells | Median estimated slope |
| --- | --- | --- | --- |
| North | 0, 150 | 1242 / 1600 | 4.09° |
| West | −150, 0 | 323 / 1600 | 9.94° |
| Hab south | 0, −40 | 0 / 1600 | No supported ground estimate |

These are outputs of this conservative algorithm, not accuracy scores or claims
that the locations are traversable. The hab-south patch is retained as a failure
case: the seed/observations did not support a connected ground estimate. It stays
unknown rather than being filled as free space. The west example reports 41
obstacle cells; these are algorithm predictions, not independently labeled truth.

The default 0.6 m cells suit this aerial sample better than the live pipeline's
0.3 m cells. Full-density local processing still has missing/unsupported cells.
Try `--resolution 1.0` for a coarser comparison; do not interpret coarsening as
better detection of small obstacles. Aerial data cannot resolve or validate all
rocks, drop-offs, rover occlusions, vegetation or sensor noise.

The evaluator uses survey class 2 to initialize ground height near the patch
center only. That is an explicit **offline fixture prior**. The live FAST-LIO
mode instead uses body position and measured ground clearance. Other input
classifications are not supplied to the terrain algorithm.

## View the patch through ROS

In a new terminal, source ROS and the built terrain workspace; the analysis venv
is not needed for the ROS nodes. Use the separate graphical inspection launch:

```bash
source /opt/ros/jazzy/setup.bash
source ~/DEIMOS-ws/rover_ws/install/setup.bash
export ROS_DOMAIN_ID=187
ros2 launch terrain_geometry site_view.launch.py \
  config:=$HOME/mdrs_site/my_patch/patch.yaml
```

This opens RViz automatically with the historical map points, a rainbow ground
height grid, and obstacle returns. Optional displays show slope, roughness,
unknown ground, and the diagnostic cost preview. Enable one grid measurement
at a time. The fixed frame is `mdrs_map`; the camera and reference grid follow
`mdrs_patch_origin`, placed at the patch center and seed elevation from the YAML.
For the north patch this is (0,150,1.52) m, so the data is centered immediately
instead of appearing far outside an origin-centered view. No manual camera or
sensor transform is required.

The terminal stays occupied while ROS publishes data; that is normal, not a
hang. Stop the viewer and nodes with **Ctrl+C**. The earlier `site_test.launch.py`
is retained as the publisher/processor-only launch; it intentionally opens no
viewer and runs until stopped.

For a finite test that closes automatically after 15 wall seconds:

```bash
ros2 launch terrain_geometry site_view.launch.py \
  config:=$HOME/mdrs_site/my_patch/patch.yaml test_duration:=15
```

Add `rviz:=false` for a terminal-only run with the same automatic shutdown.
`test_duration:=0` is the default and keeps the launch running. A timed run is
a convenient smoke test, not an automatic accuracy score.

The new launch defaults to **1 Hz** processing, overriding the 2 Hz value in
older generated YAML files. A 0.51 s computation slightly exceeds a 2 Hz budget
of 0.5 s; that warning did not indicate a deadlock. Override the rate if needed:

```bash
ros2 launch terrain_geometry site_view.launch.py \
  config:=$HOME/mdrs_site/my_patch/patch.yaml processing_rate:=0.5
```

In a second terminal using `ROS_DOMAIN_ID=187`, check the live outputs:

```bash
ros2 topic hz /terrain/grid
ros2 topic echo /terrain/grid --field header --once
ros2 run tf2_ros tf2_echo mdrs_map mdrs_patch_origin
```

Stop each continuous check with Ctrl+C before the next one. The viewer itself
does not need to stop for a check. Use a fresh ROS terminal rather than the
offline analysis venv; the viewer launch disables Python user-site packages in
its child processes so ROS uses the system NumPy installation.

Run only one patch/terrain pipeline in this ROS domain: it uses the same
`/terrain/*` output names as other terrain modes. This is an independent dataset
viewer rather than a moving rover, Nav2, FAST-LIO, or physical LiDAR test. The
historical NPZ point data has no STL surface mesh; the colored points and fitted
grid are the available map geometry.

The publisher repeats a static map patch at 2 Hz with synthetic current stamps
for inspecting the ROS pipeline. It is **not sensor simulation**: no ray casting,
occlusion, per-point rover acquisition time, motion, IMU or bag is generated.
Do not feed it into FAST-LIO or claim it validates localization.

## Download again or change the area

The included data is ready to use; no download is required to run the tests.
To reproduce it:

```bash
python tools/download_mdrs.py
```

Downloads use the public USGS EPT mirror without account credentials. The script
walks all intersecting octree levels (including parent points), caches nodes and
records checksums. The included crop used 131 nodes. `cache/` can be deleted
after a successful run. It is not included in this archive. Expand with
`--half-width 500` (a 1 km square), or set `--latitude` and `--longitude` for a
specific test area within the survey. Downloads and output size increase.
The script stops if a requested subset exceeds its 2000-node limit.

## Sources

- URC resources confirm LiDAR is available around MDRS:
  https://urc.marssociety.org/home/resources
- Public USGS source used:
  https://usgs-lidar-public.s3.amazonaws.com/USGS_LPC_UT_Southern_QL1_2018_LAS_2019/ept.json
- Project catalog:
  https://portal.opentopography.org/usgsDataset?dsid=USGS_LPC_UT_Southern_QL1_2018_LAS_2019
- Native CRS/tile catalog:
  https://prd-tnm.s3.amazonaws.com/StagedProducts/Elevation/LPC/Projects/USGS_LPC_UT_Southern_QL1_2018_LAS_2019/UT_Southern_QL1_2018.vpc

Credit: USGS 3DEP, Utah Southern QL1 2018 survey. Public data is provided without
warranty; preserve source attribution when redistributing derived datasets.

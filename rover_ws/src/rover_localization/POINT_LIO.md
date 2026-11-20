# Point-LIO in Gazebo (Sprint 2, task L2)

**TL;DR:** This runs the Point-LIO localization algorithm on the simulated
rover, so you can watch it build a map and track the rover in RViz. On a
13.4 m test drive it was off by only 3.7 cm on average, about 4x more accurate
than FAST-LIO on the same data, but it used about 15x more CPU. To try it,
follow [Quick start](#4-quick-start-watch-it-work) below. It takes about
20 minutes the first time, mostly waiting for Docker.

---

## 1. What problem does this solve?

A self-driving rover has to keep answering **"where am I right now?"**, about
10 times a second. Outdoors on a Mars-analog course there is no reliable GPS,
so the rover works it out from its own sensors. Estimating your own motion
this way is called **odometry**. Doing it while also building a map of the
surroundings is called **SLAM** (Simultaneous Localization And Mapping).

The rover has two sensors that help:

| Sensor | What it measures | Good at | Bad at |
|---|---|---|---|
| **IMU** (Inertial Measurement Unit) | Acceleration and rotation speed, 200x per second | Very fast, reacts instantly | Errors **add up**: a tiny error becomes metres within seconds |
| **LiDAR** | Spins a laser around and returns thousands of 3D points (a **point cloud**), 10x per second | Accurate; actually "sees" walls and rocks | Slower; a scan taken while moving is slightly smeared |

An analogy: the IMU is like walking with your **eyes closed** and counting
steps. That's fine for a few steps, but you drift. The LiDAR is like
**opening your eyes** and comparing what you see with what you saw before.

## 2. How Point-LIO works

**LIO** = LiDAR-Inertial Odometry: it combines the two sensors. It repeats two
steps forever:

1. **Predict:** use the IMU to guess "I have probably moved to here".
2. **Correct:** take the new LiDAR points and line them up against the **map**
   built so far. If the points don't sit on the walls at the guessed
   position, shift the guess until they do. Then add the new points to the map.

The maths that blends "guess" and "measurement", trusting each one according
to how noisy it is, is a **Kalman filter**.

**Point-LIO vs FAST-LIO.** Both are LIO algorithms. FAST-LIO collects a whole
LiDAR scan (~0.1 s of points) and corrects **once per scan**. Point-LIO
corrects as **each point (or group of points measured at the same instant)**
arrives. That makes Point-LIO more accurate during fast turns, at the cost of
much more computation.

## 3. What the launch file starts

A few ROS 2 terms first:

- **Node:** one running program.
- **Topic:** a named data channel between nodes (the names starting with `/`).
- **Launch file:** a script that starts many nodes at once.
- **RViz:** the 3D viewer.
- **Gazebo:** the physics simulator that fakes the rover and its sensors.

```
Gazebo (fake world + fake rover)
   │ fake LiDAR: /lidar/points          fake IMU: /imu
   ▼                                        │
lidar_cloud_filter                          │
 (removes "inf" points = rays that hit      │
  nothing; adds per-point time = 0)         │
   │ /lidar/points_filtered                 │
   ▼                                        ▼
                  Point-LIO (pointlio_mapping)
   │ estimated pose /Odometry, trajectory /path, map points /cloud_registered
   ▼
odometry_comparator: compares /Odometry with Gazebo's TRUE pose (/odom)
   → prints the error in metres every second
```

| Node | Job |
|---|---|
| Gazebo + bridges + controllers | `rover_description/gazebo.launch.py` (only with `sim:=true`, the default) |
| `lidar_cloud_filter` | `/lidar/points` → `/lidar/points_filtered` (see [section 6](#6-why-the-cloud-filter-exists)) |
| `pointlio_mapping` | Point-LIO with `config/point_lio_gazebo.yaml`; its odometry is renamed to `/Odometry` |
| `odometry_comparator` | Live error versus Gazebo's ground truth |
| `cmd_vel_watchdog` | Stops the rover if keyboard driving goes silent for 0.75 s |
| RViz | Shows the map and trajectory (`rviz:=false` to skip) |

The nice thing about simulation: Gazebo knows exactly where the rover really
is (the **ground truth**), so we can measure how wrong Point-LIO is. On the
real rover that is much harder.

## 4. Quick start: watch it work

You need Docker and the repo checked out on the `localization/point-lio`
branch. A **Linux** machine is needed to see the Gazebo and RViz windows; on a
Mac or with no display, see [Running without a display](#running-without-a-display-mac-ssh).

**Step 1: build the Docker image** (on your computer, from the repo root;
5–15 min the first time):

```bash
docker build -f docker/dockerfile.dev -t rover-dev .
```

Rebuild it if you built it before 2026-11-19: the image now contains a fix
without which Gazebo hangs (see [Troubleshooting](#8-troubleshooting)).

**Step 2: open a shell inside the container:**

```bash
./docker/run-dev.sh
```

Your prompt changes. You are now inside Linux with ROS 2 installed, and the
repo's `rover_ws/` folder is shared with your computer at
`/workspaces/rover_ws`.

**Step 3: download the third-party code and build everything** (inside the
container; ~2 min):

```bash
cd /workspaces/rover_ws
```
```bash
vcs import --recursive < src/third_party.repos
```
```bash
colcon build --symlink-install
```
```bash
source install/setup.bash
```

- `vcs import` downloads Point-LIO, FAST-LIO and the Livox driver into `src/`.
  They are git-ignored, so they are never committed. `--recursive` matters:
  FAST-LIO breaks without its `ikd-Tree` submodule.
- `colcon build` compiles every package. It should end with
  `Summary: 6 packages finished`.
- `source` tells this shell where the freshly built packages are. Repeat it in
  every new shell after building.

**Step 4: start the simulation with Point-LIO:**

```bash
ros2 launch rover_localization point_lio_test.launch.py
```

After 30–60 s the Gazebo and RViz windows open. The terminal fills with logs.
Look for `IMU Initializing: 100.0 %` (Point-LIO is ready) and a block like this
every second:

```
Point-LIO diagnostic | sim clock 42.300 s
Synchronized; displacements in initial base_link axes
Coordinate | Gazebo disp (m) | Point-LIO disp (m) | Error (m)
...
3D error: 0.0184 m | yaw error: -0.123 deg
```

**Keep the rover still until that block appears.** Point-LIO measures gravity
during its first second.

**Step 5: drive the rover.** Open a **second** terminal on your computer and
attach to the same container:

```bash
docker exec -it rover-dev-container bash
```
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.2 -p repeat_rate:=10.0 -p key_timeout:=0.5
```

Click on that terminal and hold `i` (forward), `j`/`l` (turn), `,` (back);
`k` stops. Hold the key down: the watchdog stops the rover if no key is
pressed for 0.75 s.

**What you should see in RViz:** coloured points piling up into a map of the
world, a green line tracing where the rover went, and the error in the first
terminal staying at a few centimetres.

**Step 6: stop.** Press `k`, then Ctrl+C in the teleop terminal, then Ctrl+C in
the launch terminal. Type `exit` to leave the container.

### Launch options

Add them after the launch command, e.g. `... point_lio_test.launch.py rviz:=false`.

| Option | Default | Meaning |
|---|---|---|
| `world:=<file>` | `second_world.sdf` | World file from `rover_description/worlds/` |
| `headless:=true` | `false` | Run Gazebo with no window (no display needed) |
| `rviz:=false` | `true` | Don't open RViz |
| `comparator:=false` | `true` | Don't print the error table |
| `sim:=false` | `true` | Don't start Gazebo; for replaying a recorded bag (section 7) |

### Running without a display (Mac, SSH)

The Gazebo and RViz windows need an X11 display, which Docker on macOS does
not provide. Everything else still works:

```bash
ros2 launch rover_localization point_lio_test.launch.py headless:=true rviz:=false
```

You'll see the error table in the terminal and can drive with teleop as
above. To *see* the map, run on a Linux machine.

## 5. Results: Point-LIO vs FAST-LIO

Both algorithms replayed **one identical recording** of a sim drive (84 s,
13.4 m: straight, 90° turn, sideways swerve strafe, arcs, spin in place,
reverse). Each ran alone, and every run was repeated with identical numbers.

| | Point-LIO | FAST-LIO (filtered cloud) | FAST-LIO (raw cloud, current `fast_lio_test`) |
|---|---|---|---|
| Average position error (RMSE) | **0.037 m** | 0.158 m | 2.175 m |
| Worst position error | **0.127 m** | 0.325 m | 4.614 m |
| Error at the end (drift) | **0.031 m (0.23 %)** | 0.166 m (1.24 %) | 0.688 m (5.13 %) |
| Heading error at end / worst | **-0.26° / 2.37°** | -3.51° / 5.17° | -90.7° / 91.0° |
| Times it lost track | 0 | 0 | 0 |
| Output rate | 10 Hz | 10 Hz | 10 Hz |
| CPU time for the 84 s drive | 144–158 s (~1.8 cores) | **10 s** | 8 s |

Takeaways:

- Point-LIO is clearly more accurate, especially in heading during the spin
  and strafe.
- Point-LIO costs ~15x the CPU. The Orin Nano is small and is already busy
  with the ZED camera, so measure on the real computer before choosing.
- FAST-LIO on the raw cloud loses its heading by ~90°. The only difference
  from the filtered run is the input topic, so `fast_lio_test.launch.py`
  should read `/lidar/points_filtered` too (FAST-LIO owner's call).
- Simulation limits: perfect sensor timing, no IMU noise model, flat ground
  truth. Expect larger errors on the real Unitree L1.

CPU was measured on an Apple Silicon Mac (arm64 Docker). Point-LIO only
enables its multi-thread option on x86, so numbers will differ on the
ThinkPad and the Orin.

## 6. Why the cloud filter exists

In one simulated scan (720 × 32 = 23 040 points), **9 026 points are `inf`**:
rays that hit nothing within 30 m, so their distance is "infinite". About
2 700 more are under 0.5 m: the rover's own body. Both estimators only throw
away points that are too *close* (`blind: 0.5`), so the `inf` points went
straight into their maps. `lidar_cloud_filter` removes them.

It also adds a per-point `time` field set to 0. Point-LIO uses that field to
"unsmear" scans taken while moving. Gazebo captures the whole scan in one
instant, so 0 is the true value here. **The real Unitree L1/L2 reports its
own per-point time, so real LiDAR data must NOT go through this filter.**

## 7. Reproducing the comparison

A **rosbag** is a recording of topics that can be played back later as if
they were live. That is how both algorithms get exactly the same input.
Run each command in its own container shell (`docker exec -it
rover-dev-container bash`), from `/workspaces/rover_ws`.

**1. Record a drive** (simulator only, no algorithm running):

```bash
ros2 launch rover_description gazebo.launch.py headless:=true
```
```bash
ros2 bag record -s mcap -o bags/shared_sim_run /clock /imu /lidar/points /odom
```

Drive with teleop, then Ctrl+C the recorder, then the launch.

**2. Replay it into ONE algorithm**, recording its output:

```bash
ros2 launch rover_localization point_lio_test.launch.py sim:=false rviz:=false comparator:=false
```
```bash
ros2 bag record -s mcap -o bags/out_point_lio /odom /Odometry
```
```bash
ros2 bag play bags/shared_sim_run
```

When `play` finishes, Ctrl+C the recorder and the launch. Don't add
`--clock` to `play`: the bag already contains the sim clock.

**3. Score it:**

```bash
ros2 run rover_localization trajectory_eval bags/out_point_lio --name point_lio
```

For FAST-LIO on the same bag, replace step 2's launch with these two nodes:

```bash
ros2 run rover_localization lidar_cloud_filter --ros-args -p use_sim_time:=true -r input:=/lidar/points -r output:=/lidar/points_filtered
```
```bash
ros2 run fast_lio fastlio_mapping --ros-args --params-file install/rover_localization/share/rover_localization/config/fast_lio_gazebo.yaml -p use_sim_time:=true -p common.lid_topic:=/lidar/points_filtered
```

`bags/` is git-ignored. The recorded bag is ~600 MB, so share it directly
rather than through git.

## 8. Troubleshooting

| Symptom | Cause and fix |
|---|---|
| Log repeats `Requesting list of world names` forever | Old Docker image. The container runs as your user (`--user`), and Gazebo's internal messaging couldn't build its default channel name. Rebuild the image (Step 1); `dockerfile.dev` now sets `GZ_PARTITION`. |
| `colcon build` fails in `fast_lio`: `Cannot find source file: include/ikd-Tree/ikd_Tree.cpp` | Submodule missing. Run `cd src/fast_lio && git submodule update --init --recursive`, then build again. |
| Point-LIO logs `imu loop back, clear deque` | Two simulations are running at once, so every IMU message arrives twice. `ros2 topic info /imu` should say `Publisher count: 1`. Stop everything, or `exit` and re-run `./docker/run-dev.sh`. |
| Error table says `Comparison unavailable` | Normal for the first seconds. If it persists, the warning lines under it name the missing topic. |
| `ros2: command not found` or `Package 'rover_localization' not found` | Run `source install/setup.bash` in that shell. |
| Gazebo or RViz window doesn't open | No display (Mac/SSH): use `headless:=true rviz:=false`. On Linux, make sure you started with `./docker/run-dev.sh`. |
| `PermissionError` when building on the host later | Build inside the container only (see CLAUDE.md, "Host vs. container UID mismatch"). |

## 9. Tuning notes (`config/point_lio_gazebo.yaml`)

- `point_filter_num: 3`, `filter_size_surf/map: 0.5` (upstream defaults). With
  1 / 0.2, Point-LIO ran at only 1.4 Hz on 10 Hz input, fell further behind
  every scan, and grew past 1.3 GB of memory.
- `acc_norm: 9.81`: the Gazebo IMU reports m/s². The real L1's IMU may report
  in g (then use 1.0). Check this before the real-L1 test.
- `extrinsic_T: [0, 0, 0.25]`: where the LiDAR sits relative to the IMU
  (= `base_link`). The launch file checks this against the URDF.
- `fov_degree: 360`, `scan_line: 32`, `det_range: 30` match the sim sensor.

## 10. Running on the real Unitree L1 (not tested yet)

> **Status:** written from the driver's and Point-LIO's own code and configs,
> but never run against real hardware. The L1 currently sends IMU data but no
> scans (task H1). Update this section after the first real test.

**How it changes from sim:** Point-LIO only listens to ROS 2 topics. In sim,
Gazebo publishes them; on hardware, Unitree's driver does. The algorithm code
stays the same. Only the topic names and a few hardware parameters change.

| | Simulation | Real Unitree L1 |
|---|---|---|
| Who publishes the data | Gazebo → bridge → `lidar_cloud_filter` | Unitree driver `unitree_lidar_ros2` ([unilidar_sdk](https://github.com/unitreerobotics/unilidar_sdk)) |
| Point-cloud topic | `/lidar/points_filtered` | `/unilidar/cloud` (frame `unilidar_lidar`) |
| IMU topic | `/imu` | `/unilidar/imu` (frame `unilidar_imu`) |
| Per-point `time` | Added as 0 by the filter | Sent by the driver (time since scan start) |
| Use `lidar_cloud_filter`? | Yes | **No.** It would overwrite the real timing |
| Launch | `rover_localization point_lio_test.launch.py` | `point_lio mapping_unilidar_l1.launch.py` (Point-LIO's own; already uses the topics above) |

For the L2, use [unilidar_sdk2](https://github.com/unitreerobotics/unilidar_sdk2)
and `mapping_unilidar_l2.launch.py` instead. The steps are the same.

### Steps

You need a **Linux** computer: Docker on macOS can't pass a USB device into
a container. Per the Sprint 2 plan, don't use the Jetson (it's reserved for the ZED).

**1. Plug in the L1 and find its serial port** (on the host):

```bash
ls /dev/ttyUSB*
```

The driver expects `/dev/ttyUSB0`. If nothing shows up, the cable or driver
is the problem, not ROS.

**2. Start the container with the device passed in.** `run-dev.sh` doesn't
pass USB devices, so use this instead (from the repo root):

```bash
docker run -it --rm --net=host --name rover-dev-container -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix -v "$(pwd)/rover_ws:/workspaces/rover_ws" --user "$(id -u):$(id -g)" --group-add dialout --device /dev/ttyUSB0 rover-dev
```

`--device` hands the serial port to the container. `--group-add dialout`
lets your user open it (serial ports belong to the `dialout` group).

**3. Get and build the Unitree driver** (inside the container, once):

```bash
cd /workspaces/rover_ws/src && git clone https://github.com/unitreerobotics/unilidar_sdk.git
```
```bash
cd /workspaces/rover_ws && colcon build --symlink-install --packages-up-to unitree_lidar_ros2 point_lio && source install/setup.bash
```

`src/unilidar_sdk/` is git-ignored. It isn't in `third_party.repos` yet
because it hasn't been verified to build in the dev image; add it once it has.

**4. Start the driver and check the data** (terminal 1):

```bash
ros2 launch unitree_lidar_ros2 launch.py
```

In a second terminal (`docker exec -it rover-dev-container bash`):

```bash
ros2 topic hz /unilidar/cloud
```
```bash
ros2 topic echo /unilidar/imu --once --field linear_acceleration
```

- `/unilidar/cloud` should arrive at roughly 10 Hz. **No messages = the H1
  "no scans" problem**; Point-LIO can't help until that's fixed.
- With the lidar sitting still and flat, the IMU's `z` is about **9.8** if it
  reports m/s² (keep `acc_norm: 9.81`), or about **1.0** if it reports g
  (set `acc_norm: 1.0`). Getting this wrong makes the pose shoot off at startup.

**5. Run Point-LIO** (terminal 2; opens RViz):

```bash
ros2 launch point_lio mapping_unilidar_l1.launch.py
```

Keep the lidar **still for the first few seconds** while the log shows
`IMU Initializing: ... 100.0 %`. Then carry it or drive slowly. Watch the
map build in RViz.

**6. Record the data** so it can be replayed into Point-LIO and FAST-LIO
later without the hardware (terminal 3):

```bash
ros2 bag record -s mcap -o bags/l1_run1 /unilidar/cloud /unilidar/imu
```

### Parameters to re-check for the real sensor

These live in `point_lio/config/unilidar_l1.yaml` (third-party, so copy it
into `rover_localization/config/` before editing):

| Parameter | Unitree default | Why check it |
|---|---|---|
| `acc_norm` | `9.81` | Must match the IMU unit (step 4) |
| `extrinsic_T` / `extrinsic_R` | L1's built-in IMU → lidar offset | Correct for the L1 by itself. Point-LIO then tracks the **L1's IMU**, not `base_link`; mounting it on the rover needs a fixed transform to `base_link` (measure from CAD) |
| `scan_line` | `18` | Must match the driver's `cloud_scan_num` |
| `blind` | `0.5` m | Raise it if the map shows the rover's own body around the sensor |
| `det_range` | `100` m | Lower it (e.g. 30) to save CPU outdoors |
| `point_filter_num`, `filter_size_surf/map` | `1`, `0.1` | Same CPU trade-off as in sim (section 9); expect to raise them on the Orin |
| `pcd_save_en` | `true` | Saves the whole map to `point_lio/PCD/` on exit; turn off for long runs |

## 11. Not done yet

- Demo recording (RViz screen capture) for the deliverable: needs a Linux
  machine with a display.
- Real Unitree L1 test: section 10 is ready but blocked until the L1
  delivers scans (task H1).

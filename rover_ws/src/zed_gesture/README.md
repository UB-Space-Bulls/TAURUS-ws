# ZED gesture input

Live ZED RGB images → YOLO COCO 17-joint poses → tracked person → arm geometry
and timed holds → JSON command intents. The supervisor consumes `/gesture/intent`;
this package does not command motors. Hardware acceptance is still required.

## Fetch and build

`git pull` updates the wrapper's recorded commit, but does not populate an
uninitialized submodule. From the **DEIMOS repository root**, run:

```bash
git submodule update --init --recursive -- rover_ws/src/zed-ros2-wrapper
```

Use the explicit path: this branch also records `fast_lio` and
`livox_ros_driver2` as Git links without `.gitmodules` mappings, so a blanket
submodule update can stop before reaching ZED. The wrapper is pinned at
`b19902daf0136b6e841b5e415c2d01f425411c45` (5.5.0 development revision).
Do not replace it with an arbitrary latest checkout. Install a compatible ZED
SDK, CUDA and ROS dependencies on the camera computer; see
[`rover_bringup`](../rover_bringup/README.md).

Source ROS Jazzy, then install PyTorch/torchvision for the computer that will run
inference. On Jetson use wheels compatible with the installed JetPack/CUDA;
desktop CUDA wheels are not Jetson wheels. Keep ROS's Python interpreter version.
The requirements pin NumPy below 2 for Jazzy's Ubuntu OpenCV/matplotlib binaries.
One way to keep pip dependencies isolated while retaining ROS packages:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages src/zed_gesture/.venv
source src/zed_gesture/.venv/bin/activate
# Install the platform-specific torch and torchvision here first.
python -m pip install -r src/zed_gesture/requirements.txt

rosdep install --from-paths src/zed-ros2-wrapper src/zed_gesture src/rover_bringup --ignore-src -r -y
python "$(command -v colcon)" build --symlink-install --packages-up-to zed_wrapper rover_bringup zed_gesture --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The commands after `source /opt/ros/...` run from `rover_ws`. Running colcon
through the activated environment's Python installs the gesture executable with
that interpreter, so it can find the inference dependencies when launched by ROS.
If it was previously built with system Python, rebuild `zed_gesture` with the
environment's Python as above. The `.venv` directory is ignored by Git.

Model weights (`yolo11n-pose.pt`, and `yolo11n.pt` when `crop_pose` is enabled)
must be accessible to the inference computer. Ultralytics downloads missing
weights on first run; provision them before operating offline. You can set
absolute model paths in a copy of `config/gesture.yaml` and pass `params_file`.

## Live hardware

First verify the camera in the ZED SDK tools and close those tools. Use the same
ROS domain and middleware for the camera, gesture process and CLI. For the
project's Zenoh configuration, start/connect to the configured router as described
in the bringup guide.

```bash
# Starts one physical ZED 2 plus gesture inference, using GPU 0.
ros2 launch zed_gesture gesture.launch.py
# A specific camera:
ros2 launch zed_gesture gesture.launch.py serial_number:=YOUR_SERIAL
# The wrapper is already running (including on another connected computer):
ros2 launch zed_gesture gesture.launch.py launch_zed:=false
# CPU inference (the camera's ZED SDK still requires a supported NVIDIA GPU):
ros2 launch zed_gesture gesture.launch.py launch_zed:=false device:=cpu
# Optional long-range mode: 960px tracking plus a 640px target-crop pose pass.
ros2 launch zed_gesture gesture.launch.py params_file:="$(ros2 pkg prefix --share zed_gesture)/config/gesture_long_range.yaml"
```

The default image topic is `/zed2/zed_node/rgb/color/rect/image`, matching the
rover bringup and pinned wrapper. `camera_model`, `camera_name`, `image_topic`,
`zed_params_file`, `params_file` and `publish_tf` are launch arguments.
The gesture launch uses `config/zed_camera.yaml`: native HD720 RGB (1280×720 at
15 Hz), with depth, positional tracking and unused sensor streams disabled to
leave GPU time for gesture inference. Camera motion TF publication defaults to
false. Rover depth/odometry bringup keeps its existing settings; if using that
camera process, pass `launch_zed:=false` and configure its RGB output to native
resolution for distant poses. Native images require more transport bandwidth.
Live launch explicitly disables simulation and SVO playback. On separate
computers, synchronize clocks: images over `max_frame_age` seconds old or ahead
are rejected (default 0.5 s). Freshness is checked before and after inference and
again immediately before publishing an intent. An expired result cannot complete
a hold, replace the previous command, or produce an intent. Image subscriptions
use best effort with queue depth one to avoid an inference backlog.

The default workload is one 640px pose pass (`long_range: false`). The optional
long-range profile retains the original two-pass algorithm and uses the native
image pixels for target crops. Benchmark it on the deployment GPU before use.
Neither the default profile nor the tests guarantee a processing rate or range
on untested hardware.

## Existing poses and holds

| Pose | Intent | Hold |
| --- | --- | --- |
| One arm up, opposite arm down (either side) | `FOLLOW` | 0.7 s |
| Both arms up | `STAY` (`label: STOP`) | 0.35 s |
| Both arms straight sideways | `COME` | 0.7 s |
| One arm straight sideways, opposite arm down | `FETCH`, `range_mode: long` | 0.7 s |

Start with both arms down for 0.4 s. Lower both arms again before another movement
command; movement commands also have a 1.2 s cooldown. STOP can interrupt without
rearming. Holding a pose emits once. Both shoulders, elbows and wrists must be
visible and confident; hips/legs are optional. Raised arms allow 45° from vertical
and elbows down to 135°. The camera is assumed upright; `shoulder_frame` and
`require_hips` retain the original alternative reference frames.

The previous **confirmed** command stays in `/gesture/status.last_command` and
the debug panel through neutral poses, uncertain detections, target loss,
pause/re-enable and reselection. A new confirmed command replaces it; restarting
the node clears it. This field is historical, not a current observation or a
request to continue movement. Previous keypoints are never substituted for a
missing pose, and intents are never replayed. An incomplete hold is broken by
missing/invalid observations; a source-frame gap over 0.25 s also disarms movement
recognition, including STOP holds. The gap limit and original hold durations are
unchanged. The node warns when processing gaps exceed that limit; inspect
`frame_gap_ms`, `frame_age_ms` and `timing_ok` on `/gesture/status`. Sustained gaps
must remain at or below 250 ms (roughly 4 FPS, with headroom for jitter). Reduce
inference workload or camera transport delay when this fails.

A lone tracked person is selected automatically. Overlap or prolonged target loss
requires explicit reselection. Use the track ID in status/debug output:

```bash
ros2 topic pub --once /zed_gesture_node/select_target std_msgs/msg/Int32 '{data: 7}'
ros2 topic pub --once /zed_gesture_node/select_target std_msgs/msg/Int32 '{data: -1}'
ros2 topic pub --once /zed_gesture_node/enable std_msgs/msg/Bool '{data: false}'
```

## Verify on the camera computer

```bash
ros2 topic hz /zed2/zed_node/rgb/color/rect/image
ros2 topic echo /gesture/status
ros2 topic echo /gesture/intent
ros2 run rqt_image_view rqt_image_view /gesture/debug_image
```

Warm up inference, then check `timing_ok: true`, `frame_gap_ms <= 250`,
`frame_age_ms <= 500`, nonzero input and processing rates, a locked track, and `pose_source`
showing `full frame` or `target crop`. Perform each pose after arms-down release;
check the intent and hold duration, one event per held pose, and that neutral or
brief detection loss retains `last_command` without producing another intent.
Check reselection after target loss and with two overlapping people. Test natural
arm bends, intended camera distance, lighting and actual inference rate on the
deployment computer. Automated geometry checks do not establish model accuracy
or range on real people.

## Regression checks and source

```bash
source /opt/ros/jazzy/setup.bash
python src/zed_gesture/zed_gesture/urc_gesture_test.py --self-test
PYTHONPATH=src/zed_gesture:$PYTHONPATH python -m pytest src/zed_gesture/test -q
```

The bundled `urc_gesture_test.py` is copied byte-for-byte from
[`dngofficial/gesture_test_urc`](https://github.com/dngofficial/gesture_test_urc/blob/5c18aaba263b4e3d6add6c2d106fc3e7da6505fd/urc_gesture_test.py),
commit `5c18aaba263b4e3d6add6c2d106fc3e7da6505fd`. Its classifier, debouncer and
target-lock behavior are preserved, including its 461 deterministic regressions
and keypoints traced from the original real-pose examples. ROS tests exercise
the image callback with controlled inference output; real model/hardware
acceptance remains a separate check. Inference supports both current Ultralytics
`quantize` and older installations' `half` precision options.

Development validation passed: the wrapper and project packages built against
Jazzy/ZED SDK 5.5, all 24 ROS regressions passed (including the 461 original
checks), and the installed launch processed a real photograph delivered over
ROS through CPU YOLO tracking/crop inference into status and debug-image topics.
The local package environment uses CPU PyTorch 2.15.0, torchvision 0.29.2,
Ultralytics 8.4.218, NumPy 1.26.4 and OpenCV 4.11.0. Replace CPU PyTorch with the
matching GPU build on the inference computer. No physical ZED or working GPU was
available for a live-camera acceptance test.

In a CPU replay of a native 1280×720 photograph, 20 samples after warmup measured
59.0 ms median processing for the default profile and 119.8 ms for the two-pass
profile. Maximum source-frame gaps were 69.0 ms and 127.1 ms respectively, within
the 250 ms hold budget on this development computer. These measurements exclude
the physical ZED SDK workload and do not establish deployment-GPU performance.

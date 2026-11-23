# yolo_perception — object detection for the rover

YOLO (Ultralytics YOLOv8) detectors that read the ZED 2 camera and publish what they see.
One ROS 2 node, several **profiles** — one trained model per rover job:

| profile | detects | used in | status |
|---|---|---|---|
| `terrain` | `crater`, `rock` | general / science | trained (YOLOv8n, 30 epochs) — **needs a proper labelled test set** |
| `hammer` | `rock_hammer` | Autonomy mission | pipeline ready, **not trained yet** (needs Roboflow key + GPU) |
| `keyboard` | keys `a`–`z`, `0`–`9` | Equipment Servicing | pipeline ready, **not trained yet** |
| `coco` | stock COCO classes | smoke tests, no custom weights needed | works |

## Datasets used so far

- **Terrain model (`best.pt`, 6 Nov 2026)** — trained in Google Colab on the Roboflow export
  **"Lunabotics-Computer-Vision" v3** (classes `Rock`, `crater`), starting from COCO-pretrained
  `yolov8n.pt`, 30 epochs, 640 px. Also downloaded: Roboflow Universe
  [`dsad-omghq/combined-mgeca` v2](https://universe.roboflow.com/dsad-omghq/combined-mgeca/dataset/2)
  (CC BY 4.0, classes `crater`, `rock`).
  ⚠️ The 30 local test photos had **no label files**, and that dataset orders the classes
  `crater, rock` while the model uses `Rock, crater` — so the earlier confusion-matrix run is not a real
  score. Re-evaluate on a labelled split before trusting numbers.
- **Hammer candidates** (Roboflow Universe, to be checked once downloaded):
  `rover-object-detection/rock-hammer-7alb5`, `timlers-domain/rock-hammer`,
  `mt-ronnj/mt11_rock_pick-dataset`; optional desert scenes `orange-hammer/orange-hammer-detection-desert`.
- **Keyboard candidates**: `1st-518jb/only_umrt_keyboard` (another URC team), `tugraz-gkeui/keyboard-detection-1m02b`.

All sources are listed in [`training/datasets.yaml`](training/datasets.yaml).

## Why one model per job (instead of one model that does everything)

Short answer: **adding letters to the crater model would make the crater model worse, and it buys nothing**,
because the rover never needs craters, the hammer and keys at the same moment.

1. **Training settings conflict.** Letters must be trained with horizontal flip **off** (a mirrored `b` is a `d`,
   `p` becomes `q`) and at a larger image size (keys are small). Craters/rocks/hammers *want* flip on.
   One model gets one set of settings, so somebody loses.
2. **Missing labels.** Each dataset only labels its own objects. Merge a hammer dataset whose desert photos
   contain unlabelled rocks into a rock detector and you are literally teaching it "rocks are background".
   Rock recall drops. (Keyboard + craters don't overlap visually, but hammer + rocks do.)
3. **Imbalance and capacity.** A keyboard photo has ~40–80 key boxes, a terrain photo a handful. The loss gets
   dominated by keys, and `yolov8n` (3 M parameters — small enough for the Orin Nano) has limited capacity
   to share between 2 easy classes and 36 hard ones.
4. **Maintenance.** With one model, every tweak to the keyboard data means re-training *and re-validating*
   craters and the hammer. Separate models can be improved independently.
5. **No speed cost.** Missions run one at a time (Autonomy → hammer, Equipment Servicing → keyboard),
   so we load only the model for the current mission. Running two at once just means two node instances
   with different `topic_prefix`.

When would one model make sense? If two object types must be found **in the same frames at the same time**
and both datasets label both objects. E.g. if Autonomy later needs hammer + water bottle, train one
`autonomy` model on data that labels both.

**TL;DR:** separate small models, one per mission, zero interference. No cap, it's the cleaner play.

## Why letters are harder than craters

- **36 classes vs 2**, and the differences are tiny strokes: `O/Q/D/0`, `I/L/1`, `M/N/W`, `E/F`, `B/8`, `S/5`.
- **Small, packed objects:** keys sit edge to edge, so box overlap/NMS matters (profile uses `iou: 0.30`).
- **Appearance varies a lot:** font, keycap colour, backlighting, glare, worn legends, camera angle from the arm.
- **Needs near-perfect precision:** pressing the wrong key fails the task. Plan to add a sanity check using the
  known QWERTY layout (e.g. `G` must sit between `F` and `H`) before the arm presses anything.
- On the plus side, every keyboard photo contains every letter once, so classes are naturally balanced.

## Hammer: what to test

Competition object is a **rock hammer** (geologist's pick: flat head + pointed pick, e.g. Estwing-style).
Online data has few of these in desert scenes, so:

1. Train on the merged Roboflow hammer sets (`build_dataset.py hammer`).
2. **Take our own photos of the actual hammer** outdoors on sand/gravel at 2–15 m, different angles, partly
   hidden, morning/noon light. Label ~150–300 in Roboflow, add as a `local:` source, retrain.
3. Test generalisation with `training/eval_by_folder.py` on folders `rock_hammer/`, `claw_hammer/`,
   `ball_peen_hammer/`, `sledge_hammer/`, `rubber_mallet/`, plus `negative_*` folders (rocks, sticks,
   other tools) to measure false alarms.

## Run it

```bash
pip install ultralytics                 # YOLO; not available through rosdep/apt

cd rover_ws && colcon build --packages-select yolo_perception --symlink-install
source install/setup.bash

# with the real ZED running (default topic /zed2/zed_node/rgb/color/rect/image, as in rover_bringup):
ros2 launch yolo_perception yolo.launch.py profile:=terrain
# without a ZED — replay a folder of photos (or mock_source:=0 for a webcam):
ros2 launch yolo_perception yolo.launch.py profile:=terrain mock:=true mock_source:=~/test_media mock_fps:=2
# no ROS at all:
ros2 run yolo_perception offline_tester --profile terrain --source ~/test_media
```

Weights: the terrain model ships in `models/terrain_crater_rock.pt`, so it works right after cloning.
New models (hammer, keyboard) go in `~/yolo_weights/` under the names in `config/models.yaml`
(or set `weights_dir:=` / `$YOLO_WEIGHTS_DIR`); a file there overrides the bundled one.
If the node logs "No images on ... yet", run `ros2 topic list | grep image` and pass `image_topic:=...`.

Topics (default prefix `/yolo`):

| topic | type | for |
|---|---|---|
| `/yolo/detections` | `vision_msgs/Detection2DArray` | mission/autonomy code (class name in `results[0].hypothesis.class_id`) |
| `/yolo/annotations` | `foxglove_msgs/ImageAnnotations` | boxes over the camera image in Foxglove |
| `/yolo/annotated_image` | `sensor_msgs/Image` | rendered only when something subscribes |

## Train a model

```bash
cd rover_ws/src/yolo_perception/training
pip install roboflow
export ROBOFLOW_API_KEY=xxxxx          # never commit this
python3 build_dataset.py hammer        # download + merge -> datasets/hammer/data.yaml
python3 train.py hammer                # prints the path of best.pt at the end
python3 eval_by_folder.py --model runs/hammer/yolov8n/weights/best.pt --root ~/hammer_test
mkdir -p ~/yolo_weights && cp runs/hammer/yolov8n/weights/best.pt ~/yolo_weights/rock_hammer.pt
```
(A second run of the same task goes to `runs/<task>/yolov8n2/`; use the path `train.py` prints.)

GPU note: the PyTorch build must match the NVIDIA driver (`python3 -c "import torch; print(torch.cuda.is_available())"`
must print `True`), otherwise training falls back to CPU and the keyboard model will take days.
On the Orin Nano, export to TensorRT **on the Orin itself**: `yolo export model=best.pt format=engine half=True imgsz=<profile imgsz>`,
then run with `model_path:=/path/to/best.engine`. The engine's input size is fixed, so it must equal the profile's `imgsz`.

#!/usr/bin/env python3
"""
Run a model on images/videos without ROS and print what it finds.

  ros2 run yolo_perception offline_tester --profile hammer --source ~/test_media/hammers
  ros2 run yolo_perception offline_tester --model ~/yolo_weights/best.pt --source img.jpg

Annotated copies are saved under --out (default ~/yolo_runs/offline/<profile>).
"""

import argparse
import os

from ultralytics import YOLO

from yolo_perception.profiles import class_ids_from_names, get_profile, resolve_weights


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--profile', default='terrain', help='profile in config/models.yaml')
    p.add_argument('--model', default='', help='weights path; overrides the profile')
    p.add_argument('--source', required=True, help='image, folder, video or webcam index')
    p.add_argument('--conf', type=float, default=None)
    p.add_argument('--imgsz', type=int, default=None)
    p.add_argument('--weights-dir', default=None)
    p.add_argument('--device', default=None, help="'cpu', '0', ... (default: auto)")
    p.add_argument('--out', default='~/yolo_runs/offline')
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    profile = get_profile(args.profile)
    weights = resolve_weights(args.model or profile.weights, args.weights_dir)
    conf = args.conf if args.conf is not None else profile.conf
    imgsz = args.imgsz or profile.imgsz

    print(f'Loading {weights} (profile {profile.name}, conf={conf}, imgsz={imgsz})')
    model = YOLO(weights)
    print(f'Model classes: {list(model.names.values())}')
    classes = None if args.model else class_ids_from_names(model.names, profile.classes)

    results = model.predict(
        source=os.path.expanduser(args.source), conf=conf, iou=profile.iou, imgsz=imgsz,
        classes=classes, device=args.device, save=True, stream=True, verbose=False,
        project=os.path.expanduser(args.out), name=profile.name, exist_ok=True)

    n_images = n_with_det = 0
    save_dir = None
    for r in results:
        n_images += 1
        save_dir = r.save_dir
        total_ms = sum(r.speed.values())
        print(f'\n[{n_images}] {os.path.basename(r.path)}  {total_ms:.1f} ms '
              f'({1000.0 / total_ms if total_ms else 0:.1f} FPS)')
        if len(r.boxes):
            n_with_det += 1
        for box in r.boxes:
            x1, y1, x2, y2 = [round(float(v), 1) for v in box.xyxy[0]]
            print(f'   {model.names[int(box.cls[0])]:<14} conf {float(box.conf[0]):.2f}  '
                  f'box [{x1}, {y1}, {x2}, {y2}]')

    print(f'\n{n_with_det}/{n_images} images had at least one detection.')
    if save_dir:
        print(f'Annotated images: {save_dir}')


if __name__ == '__main__':
    main()

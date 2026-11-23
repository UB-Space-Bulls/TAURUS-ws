#!/usr/bin/env python3
"""
Check a detector on groups of photos (e.g. hammer types) without labelling them.

Put photos in one sub-folder per group:

  hammer_test/
    rock_hammer/        <- the competition object: should be ~100% detected
    claw_hammer/
    ball_peen_hammer/
    sledge_hammer/
    rubber_mallet/
    negative_no_hammer/ <- desert/rocks/tools that are NOT hammers: should be ~0%

  python3 eval_by_folder.py --model runs/hammer/yolov8n/weights/best.pt --root ~/hammer_test

For each group it prints how many photos got at least one detection and the
average best confidence. Folders whose name starts with "negative" or "no_" count
as false-alarm tests. Annotated images go to --out so you can eyeball misses.

This is an image-level check (found / not found), not mAP — use `yolo val` on a
labelled set for mAP. It is the quick way to answer "does it still see a claw
hammer?" or "does it fire on a stick?".
"""

import argparse
import os
from pathlib import Path

IMAGE_EXTS = {'.jpg', '.jpeg', '.png', '.bmp', '.webp'}


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--model', required=True)
    p.add_argument('--root', required=True, help='folder with one sub-folder per group')
    p.add_argument('--conf', type=float, default=0.5)
    p.add_argument('--imgsz', type=int, default=640)
    p.add_argument('--device', default=None)
    p.add_argument('--out', default='~/yolo_runs/eval_by_folder')
    args = p.parse_args()

    from ultralytics import YOLO

    model = YOLO(os.path.expanduser(args.model))
    root = Path(os.path.expanduser(args.root))
    out = Path(os.path.expanduser(args.out))
    groups = sorted(d for d in root.iterdir() if d.is_dir())
    if not groups:
        raise SystemExit(f'No sub-folders in {root}')

    rows = []
    for g in groups:
        imgs = sorted(str(p) for p in g.iterdir() if p.suffix.lower() in IMAGE_EXTS)
        if not imgs:
            continue
        hits, best_confs, misses = 0, [], []
        for r in model.predict(imgs, conf=args.conf, imgsz=args.imgsz, device=args.device,
                               stream=True, verbose=False, save=True,
                               project=str(out), name=g.name, exist_ok=True):
            if len(r.boxes):
                hits += 1
                best_confs.append(float(r.boxes.conf.max()))
            else:
                misses.append(Path(r.path).name)
        negative = g.name.lower().startswith(('negative', 'no_'))
        rows.append((g.name, len(imgs), hits, best_confs, misses, negative))

    print(f'\nmodel: {args.model}   conf >= {args.conf}\n')
    print(f'{"group":<24}{"images":>7}{"detected":>10}{"rate":>8}{"avg best conf":>15}   meaning')
    for name, n, hits, confs, misses, negative in rows:
        rate = hits / n
        avg = sum(confs) / len(confs) if confs else 0.0
        meaning = 'FALSE ALARMS (want 0%)' if negative else 'recall (want high)'
        print(f'{name:<24}{n:>7}{hits:>10}{rate:>8.0%}{avg:>15.2f}   {meaning}')
    print(f'\nAnnotated images: {out}/<group>/')
    for name, n, hits, confs, misses, negative in rows:
        if not negative and misses:
            more = ' ...' if len(misses) > 10 else ''
            print(f'  missed in {name}: {", ".join(misses[:10])}{more}')


if __name__ == '__main__':
    main()

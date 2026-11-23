#!/usr/bin/env python3
"""
Train one model per task with settings that suit that task.

  python3 train.py hammer                     # uses datasets/hammer/data.yaml
  python3 train.py keyboard --epochs 300 --batch 4
  python3 train.py terrain --data /path/to/data.yaml

Best weights end up in runs/<task>/<name>/weights/best.pt — copy that to
~/yolo_weights/<profile weights name> (see config/models.yaml) to use it on the rover.

Why the presets differ:
  keyboard  fliplr=0  -> a mirrored 'b' looks like 'd', 'p' like 'q': flipping corrupts labels.
            imgsz=960 -> keys are small; 640 throws away the detail that separates O/Q/D/0.
  hammer    degrees=15 -> a hammer lying on the ground can point any direction.
  terrain   defaults that best.pt was trained with (640, flips on).
"""

import argparse
from pathlib import Path

HERE = Path(__file__).resolve().parent

PRESETS = {
    'terrain': {'imgsz': 640, 'epochs': 100, 'fliplr': 0.5, 'flipud': 0.0, 'degrees': 0.0},
    'hammer': {'imgsz': 640, 'epochs': 150, 'fliplr': 0.5, 'flipud': 0.0, 'degrees': 15.0,
               'scale': 0.6},
    'keyboard': {'imgsz': 960, 'epochs': 200, 'fliplr': 0.0, 'flipud': 0.0, 'degrees': 5.0,
                 'perspective': 0.0005, 'mixup': 0.0, 'close_mosaic': 20},
}


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('task', choices=sorted(PRESETS))
    p.add_argument('--data', default=None, help='default: datasets/<task>/data.yaml')
    p.add_argument('--model', default='yolov8n.pt',
                   help='start point; yolov8n.pt runs fastest on the Orin Nano, '
                        'yolov8s.pt is more accurate')
    p.add_argument('--epochs', type=int, default=None)
    p.add_argument('--imgsz', type=int, default=None)
    p.add_argument('--batch', type=int, default=-1, help='-1 = AutoBatch (fit GPU memory)')
    p.add_argument('--device', default=None, help="'0' for GPU, 'cpu' (default: auto)")
    p.add_argument('--name', default=None)
    p.add_argument('--workers', type=int, default=4)
    args = p.parse_args()

    import torch
    import yaml
    from ultralytics import YOLO

    cfg = dict(PRESETS[args.task])
    if args.epochs:
        cfg['epochs'] = args.epochs
    if args.imgsz:
        cfg['imgsz'] = args.imgsz
    data = args.data or str(HERE / 'datasets' / args.task / 'data.yaml')
    if not Path(data).is_file():
        raise SystemExit(f'{data} not found — run: python3 build_dataset.py {args.task}')

    batch = args.batch
    if not torch.cuda.is_available():
        print('WARNING: no CUDA GPU visible to PyTorch — training on CPU will be very slow.')
        if batch == -1:
            batch = 8  # AutoBatch only works on GPU

    model = YOLO(args.model)
    model.train(
        data=data, batch=batch, device=args.device, workers=args.workers,
        project=str(HERE / 'runs' / args.task), name=args.name or Path(args.model).stem,
        patience=40, plots=True, **cfg)

    best_path = Path(model.trainer.save_dir) / 'weights' / 'best.pt'
    best = YOLO(str(best_path))
    with open(data) as f:
        # report on the held-out test split when the dataset has one
        split = 'test' if yaml.safe_load(f).get('test') else 'val'
    metrics = best.val(data=data, split=split, imgsz=cfg['imgsz'],
                       batch=batch if batch > 0 else 8, plots=True,
                       project=str(HERE / 'runs' / args.task),
                       name=f'{best_path.parent.parent.name}_{split}')
    print(f'\n{split} mAP50={metrics.box.map50:.3f}  mAP50-95={metrics.box.map:.3f}  '
          f'P={metrics.box.mp:.3f}  R={metrics.box.mr:.3f}')
    print(f'Best weights: {best_path}')


if __name__ == '__main__':
    main()

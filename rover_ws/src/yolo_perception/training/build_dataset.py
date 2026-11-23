#!/usr/bin/env python3
"""
Download the sources listed in datasets.yaml and merge them into one YOLO dataset.

  export ROBOFLOW_API_KEY=xxxxx
  python3 build_dataset.py hammer            # -> datasets/hammer/data.yaml
  python3 build_dataset.py keyboard --skip-download

Output layout (YOLO format, ready for train.py):
  datasets/<task>/{train,valid,test}/{images,labels}/  +  datasets/<task>/data.yaml

Each source's class names are mapped to the output classes (see datasets.yaml);
labels of unmapped classes are dropped. Images are hard-linked when possible
(no extra disk space), copied otherwise.
"""

import argparse
from collections import Counter
import hashlib
import os
from pathlib import Path
import random
import re
import shutil
import sys

import yaml

HERE = Path(__file__).resolve().parent
DEFAULT_ROOT = HERE / 'datasets'
SPLITS = ('train', 'valid', 'test')
IMAGE_EXTS = {'.jpg', '.jpeg', '.png', '.bmp', '.webp'}


# --------------------------------------------------------------------------- download
def source_dir_name(src):
    return f"{src['workspace']}__{src['project']}__v{src['version']}"


def download_sources(task_cfg, raw_root: Path):
    key = os.environ.get('ROBOFLOW_API_KEY')
    if not key:
        sys.exit('ROBOFLOW_API_KEY is not set. Get one at roboflow.com -> Settings -> '
                 'API Keys, then\n  export ROBOFLOW_API_KEY=xxxxx')
    try:
        from roboflow import Roboflow
    except ImportError:
        sys.exit('pip install roboflow   (needed to download datasets)')

    rf = Roboflow(api_key=key)
    for src in task_cfg['sources']:
        if 'local' in src:
            continue
        project = rf.workspace(src['workspace']).project(src['project'])
        version = src['version']
        if str(version) == 'latest':
            numbers = [int(str(v.version).rstrip('/').rsplit('/', 1)[-1])
                       for v in project.versions()]
            version = max(numbers)
        src['version'] = int(version)
        dest = raw_root / source_dir_name(src)
        if (dest / 'data.yaml').is_file():
            print(f'[skip] {dest.name} already downloaded')
            continue
        print(f"[download] {src['workspace']}/{src['project']} v{version} -> {dest}")
        # overwrite=True: a half-finished earlier download (folder without data.yaml)
        # would otherwise be skipped forever.
        project.version(int(version)).download('yolov8', location=str(dest), overwrite=True)


def resolve_source_dirs(task_cfg, raw_root: Path):
    dirs = []
    for src in task_cfg['sources']:
        if 'local' in src:
            d = Path(os.path.expanduser(src['local']))
        elif str(src['version']) == 'latest':
            matches = sorted(raw_root.glob(f"{src['workspace']}__{src['project']}__v*"),
                             key=lambda p: int(p.name.rsplit('__v', 1)[-1]))
            if not matches:
                sys.exit(f"{src['workspace']}/{src['project']} not downloaded yet "
                         '(drop --skip-download)')
            d = matches[-1]
        else:
            d = raw_root / source_dir_name(src)
        if not (d / 'data.yaml').is_file():
            sys.exit(f'No data.yaml in {d}')
        dirs.append(d)
    return dirs


# --------------------------------------------------------------------------- class mapping
# A separator is required: 'key_a' -> 'a', but a generic 'Keys'/'Buttons' class
# must NOT become the letter 's'.
KEY_PATTERNS = [
    re.compile(r'^(?:key|btn|button)[\s_\-]+(\w)$', re.I),
    re.compile(r'^(\w)[\s_\-]+(?:key|btn|button)$', re.I),
]


def normalize_key(name: str) -> str:
    n = name.strip()
    for pat in KEY_PATTERNS:
        m = pat.match(n)
        if m:
            n = m.group(1)
            break
    return n.strip().lower()


def map_source_classes(names, task_cfg):
    """Return {source_id: output_name or None}."""
    mapping = {}
    if task_cfg.get('normalize_keys'):
        allowed = set(task_cfg.get('allowed_keys', ''))
        for i, n in enumerate(names):
            k = normalize_key(str(n))
            mapping[i] = k if (len(k) == 1 and k in allowed) else None
    else:
        rules = [(re.compile(rx, re.I), out) for rx, out in task_cfg['class_map'].items()]
        for i, n in enumerate(names):
            mapping[i] = next((out for rx, out in rules if rx.search(str(n))), None)
    if task_cfg.get('normalize_keys'):
        # Two source classes landing on the same key ('A' and 'key_a') is fine; flag
        # it anyway so a surprising merge is visible in the log.
        seen = {}
        for i, out in mapping.items():
            if out:
                seen.setdefault(out, []).append(names[i])
        for out, srcs in seen.items():
            if len(srcs) > 1:
                print(f'  note: source classes {srcs} all map to key {out!r}')
    return mapping


def read_names(src_dir: Path):
    with open(src_dir / 'data.yaml') as f:
        names = yaml.safe_load(f)['names']
    if isinstance(names, dict):
        names = [names[k] for k in sorted(names)]
    return [str(n) for n in names]


# --------------------------------------------------------------------------- merge
def link_or_copy(src: Path, dst: Path):
    try:
        os.link(src, dst)
    except OSError:
        shutil.copy2(src, dst)


def split_dirs(src_dir: Path):
    """Find (split, images_dir) pairs, carving a valid split from train if missing."""
    found = {}
    for split in SPLITS:
        alt = 'val' if split == 'valid' else split
        for cand in (src_dir / split / 'images', src_dir / alt / 'images'):
            if cand.is_dir():
                found[split] = cand
                break
    return found


def merge(task, task_cfg, src_dirs, out_dir: Path, val_frac: float, seed: int):
    per_source = []
    for d in src_dirs:
        names = read_names(d)
        per_source.append((d, names, map_source_classes(names, task_cfg)))

    classes = task_cfg['classes']
    if classes == 'auto':
        sets = [{v for v in m.values() if v} for _, _, m in per_source]
        common = set.intersection(*sets) if sets else set()
        allowed_order = task_cfg.get('allowed_keys', '')
        classes = [c for c in allowed_order if c in common] or sorted(common)
        dropped = sorted(set.union(*sets) - common) if sets else []
        if dropped:
            print(f'[auto classes] dropped (not labelled by every source): {dropped}')
    if not classes:
        sys.exit('No output classes left. Check class_map / sources.')
    class_idx = {c: i for i, c in enumerate(classes)}

    if out_dir.exists():
        shutil.rmtree(out_dir)
    rng = random.Random(seed)
    counts = {s: Counter() for s in SPLITS}
    n_images = Counter()
    # Roboflow Universe sets are often forks of each other: the same photo can sit in
    # source A's train split and source B's test split, which inflates mAP. Keep the
    # first copy of every identical file.
    seen_hashes = set()
    n_duplicates = 0

    for n_src, (src_dir, names, mapping) in enumerate(per_source):
        # index keeps tags unique even if two sources share a (truncated) dir name
        tag = f's{n_src}_' + re.sub(r'[^A-Za-z0-9]+', '_', src_dir.name)[:40]
        used = {names[i]: m for i, m in mapping.items() if m in class_idx}
        print(f'[{src_dir.name}] using {used}, dropping '
              f'{[names[i] for i, m in mapping.items() if m not in class_idx]}')
        splits = split_dirs(src_dir)
        if not splits:
            print(f'  WARNING: no train/valid/test images folder in {src_dir}, skipped')
        n_emptied = 0
        jobs = []  # (out_split, image_path)
        for split, img_dir in splits.items():
            imgs = sorted(p for p in img_dir.iterdir() if p.suffix.lower() in IMAGE_EXTS)
            if split == 'train' and 'valid' not in splits:
                rng.shuffle(imgs)
                k = max(1, int(len(imgs) * val_frac)) if len(imgs) > 1 else 0
                jobs += [('valid', p) for p in imgs[:k]] + [('train', p) for p in imgs[k:]]
            else:
                jobs += [(split, p) for p in imgs]

        for out_split, img in jobs:
            digest = hashlib.md5(img.read_bytes()).hexdigest()
            if digest in seen_hashes:
                n_duplicates += 1
                continue
            seen_hashes.add(digest)
            lbl = img.parent.parent / 'labels' / (img.stem + '.txt')
            out_lines = []
            had_boxes = False
            if lbl.is_file():
                for line in lbl.read_text().splitlines():
                    parts = line.split()
                    if not parts:
                        continue
                    had_boxes = True
                    name = mapping.get(int(float(parts[0])))
                    if name in class_idx:
                        out_lines.append(' '.join([str(class_idx[name])] + parts[1:]))
                        counts[out_split][name] += 1
            if had_boxes and not out_lines:
                n_emptied += 1
            img_out = out_dir / out_split / 'images' / f'{tag}__{img.name}'
            lbl_out = out_dir / out_split / 'labels' / f'{tag}__{img.stem}.txt'
            img_out.parent.mkdir(parents=True, exist_ok=True)
            lbl_out.parent.mkdir(parents=True, exist_ok=True)
            link_or_copy(img, img_out)
            lbl_out.write_text('\n'.join(out_lines) + ('\n' if out_lines else ''))
            n_images[out_split] += 1
        if n_emptied:
            print(f'  {n_emptied} images lost all their boxes to dropped classes and are kept '
                  'as "nothing here" examples - make sure those dropped classes are not '
                  'actually the target object under another name.')

    data = {
        # absolute: ultralytics resolves a relative path against its own datasets dir
        'path': str(out_dir.resolve()),
        'train': 'train/images',
        'val': 'valid/images',
        'names': dict(enumerate(classes)),
    }
    if n_images['test']:
        data['test'] = 'test/images'
    with open(out_dir / 'data.yaml', 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)

    print(f'\n=== {task}: {out_dir / "data.yaml"}')
    if n_duplicates:
        print(f'  skipped {n_duplicates} duplicate images (identical files across sources/splits)')
    for s in SPLITS:
        if n_images[s]:
            top = ', '.join(f'{c}:{counts[s][c]}' for c in classes[:12])
            more = ' ...' if len(classes) > 12 else ''
            print(f'  {s:<5} {n_images[s]:5d} images | boxes {top}{more}')
    rare = [c for c in classes if counts['train'][c] < 50]
    if rare:
        print(f'  WARNING: fewer than 50 training boxes for {rare} '
              '- expect weak results on those.')


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('task', help='terrain | hammer | keyboard (sections of datasets.yaml)')
    p.add_argument('--config', default=str(HERE / 'datasets.yaml'))
    p.add_argument('--root', default=str(DEFAULT_ROOT), help='where raw/ and merged datasets go')
    p.add_argument('--skip-download', action='store_true')
    p.add_argument('--val-frac', type=float, default=0.15,
                   help='used only if a source has no valid split')
    p.add_argument('--seed', type=int, default=0)
    args = p.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    if args.task not in cfg:
        sys.exit(f'Unknown task {args.task}. Options: {list(cfg)}')
    task_cfg = cfg[args.task]
    root = Path(os.path.expanduser(args.root))
    raw_root = root / 'raw'
    raw_root.mkdir(parents=True, exist_ok=True)

    if not args.skip_download:
        download_sources(task_cfg, raw_root)
    src_dirs = resolve_source_dirs(task_cfg, raw_root)
    merge(args.task, task_cfg, src_dirs, root / args.task, args.val_frac, args.seed)


if __name__ == '__main__':
    main()

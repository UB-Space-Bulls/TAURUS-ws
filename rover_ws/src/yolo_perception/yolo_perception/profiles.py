"""
Model profiles: one YOLO model per rover job (terrain, hammer, keyboard, ...).

A profile bundles the weights file and the inference settings that suit it
(image size, confidence threshold, which classes to keep). Profiles live in
config/models.yaml so tuning a threshold never needs a code change.

Weights are looked up in this order:
  1. an absolute / ~ path given directly,
  2. the `weights_dir` argument,
  3. $YOLO_WEIGHTS_DIR,
  4. ~/yolo_weights
  5. the package's own models/ folder (weights committed to git)
If none exists, the bare name is returned so Ultralytics can auto-download
stock models such as yolov8n.pt.
"""

from dataclasses import dataclass, field
import os
from pathlib import Path
from typing import List, Optional

import yaml

PACKAGE_NAME = 'yolo_perception'
DEFAULT_WEIGHTS_DIR = '~/yolo_weights'


@dataclass
class Profile:
    name: str
    weights: str
    imgsz: int = 640
    conf: float = 0.45
    iou: float = 0.45
    classes: List[str] = field(default_factory=list)  # [] = every class
    description: str = ''


def default_profiles_file() -> Path:
    """Installed config/models.yaml, falling back to the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = Path(get_package_share_directory(PACKAGE_NAME)) / 'config' / 'models.yaml'
        if installed.is_file():
            return installed
    except Exception:  # not sourced / not installed: running from source
        pass
    return Path(__file__).resolve().parent.parent / 'config' / 'models.yaml'


def load_profiles(path: Optional[str] = None) -> dict:
    cfg_path = Path(os.path.expanduser(path)) if path else default_profiles_file()
    with open(cfg_path, 'r') as f:
        raw = yaml.safe_load(f) or {}
    profiles = {}
    for name, p in (raw.get('profiles') or {}).items():
        profiles[name] = Profile(
            name=name,
            weights=str(p['weights']),
            imgsz=int(p.get('imgsz', 640)),
            conf=float(p.get('conf', 0.45)),
            iou=float(p.get('iou', 0.45)),
            classes=[str(c) for c in (p.get('classes') or [])],
            description=str(p.get('description', '')),
        )
    return profiles


def get_profile(name: str, path: Optional[str] = None) -> Profile:
    profiles = load_profiles(path)
    if name not in profiles:
        raise KeyError(f"Unknown profile '{name}'. Known profiles: {sorted(profiles)}")
    return profiles[name]


def bundled_models_dirs() -> List[str]:
    """Weights shipped inside the package (models/), installed and source-tree copies."""
    dirs = []
    try:
        from ament_index_python.packages import get_package_share_directory
        dirs.append(os.path.join(get_package_share_directory(PACKAGE_NAME), 'models'))
    except Exception:  # not sourced / not installed: running from source
        pass
    dirs.append(str(Path(__file__).resolve().parent.parent / 'models'))
    return dirs


def resolve_weights(weights: str, weights_dir: Optional[str] = None) -> str:
    expanded = os.path.expanduser(weights)
    if os.path.isabs(expanded) or os.sep in weights:
        return expanded
    # Local copies first (so a freshly trained model overrides the bundled one).
    search_dirs = [weights_dir, os.environ.get('YOLO_WEIGHTS_DIR'), DEFAULT_WEIGHTS_DIR]
    search_dirs += bundled_models_dirs()
    for d in search_dirs:
        if not d:
            continue
        candidate = os.path.join(os.path.expanduser(d), weights)
        if os.path.isfile(candidate):
            return candidate
    # Not found locally: let Ultralytics try (works for stock names like yolov8n.pt).
    return weights


def class_ids_from_names(model_names: dict, wanted: List[str]) -> Optional[List[int]]:
    """
    Map class names from the profile to model class ids (case-insensitive).

    Returns None (= keep every class) when `wanted` is empty.
    """
    if not wanted:
        return None
    lookup = {str(n).lower(): int(i) for i, n in model_names.items()}
    ids, missing = [], []
    for name in wanted:
        if name.lower() in lookup:
            ids.append(lookup[name.lower()])
        else:
            missing.append(name)
    if missing:
        raise ValueError(
            f'Classes {missing} are not in this model. '
            f'Model classes: {list(model_names.values())}')
    return ids

#!/usr/bin/env python3
"""Validate using geometric ray casting; does NOT launch Gazebo or ROS."""
import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from terrain_geometry_impl.core import Config, process  # noqa: E402
from terrain_geometry_impl.demo_scene import raycast_scene, summarize, acceptance  # noqa: E402
from terrain_geometry_impl.l1_profile import L1Profile  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--plot', type=Path, help='Optional PNG; requires matplotlib.')
    parser.add_argument('--l1-variant', choices=('PM', 'RM'), default='PM')
    args = parser.parse_args()
    profile = L1Profile(variant=args.l1_variant)
    cloud = raycast_scene(profile)
    start = time.perf_counter()
    cfg = Config()
    result = process(cloud, cfg)
    processing_seconds = time.perf_counter()-start
    metrics = summarize(result, cfg)
    checks = acceptance(metrics)
    report = {'source': 'offline L1-envelope raster approximation, NOT Gazebo/RViz or hardware',
              'l1_variant': profile.variant,
              'raster_shape': [profile.vertical_samples, profile.horizontal_samples],
              'simulated_frame_rate_hz': profile.update_rate,
              'range_noise_stddev_m': profile.range_noise_stddev,
              'noise_seed': 42,
              'input_points': len(cloud), 'processing_seconds': processing_seconds,
              'metrics': metrics, 'checks': checks, 'passed': all(checks.values())}
    text = json.dumps(report, indent=2, allow_nan=False) + '\n'
    print(text, end='')
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text)
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap
        fig, axes = plt.subplots(2, 2, figsize=(11, 8), constrained_layout=True)
        extent = (cfg.origin_x, cfg.origin_x+cfg.width*cfg.resolution,
                  cfg.origin_y, cfg.origin_y+cfg.height*cfg.resolution)
        layers = ((result.height, 'Ground height (m)', 'terrain', 0, 1.1),
                  (np.degrees(result.slope), 'Slope (degrees)', 'viridis', 0, 20),
                  (result.roughness, 'Roughness (m RMS)', 'magma', 0, 0.06))
        for ax, (layer, title, cmap, lo, hi) in zip(axes.flat, layers):
            colors = plt.get_cmap(cmap).copy()
            colors.set_bad('#bec4ca')
            im = ax.imshow(layer, origin='lower', extent=extent, cmap=colors, vmin=lo, vmax=hi)
            ax.set_title(title)
            fig.colorbar(im, ax=ax, shrink=0.8)
        ax = axes.flat[3]
        ax.imshow(result.status, origin='lower', extent=extent,
                  cmap=ListedColormap(['#bec4ca', '#35a566', '#d7493f', '#f6ae36']), vmin=0, vmax=3)
        ax.set_title('Gray: unknown | green: ground | red: obstacle\nGold: ground + obstacle')
        for ax in axes.flat:
            ax.set_xlabel('x (m)')
            ax.set_ylabel('y (m)')
        fig.suptitle(f'P3 / Unitree L1 {profile.variant} envelope — offline raster approximation\n'
                     'Not a Gazebo or hardware recording', fontsize=15)
        args.plot.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.plot, dpi=160)
        plt.close(fig)
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()

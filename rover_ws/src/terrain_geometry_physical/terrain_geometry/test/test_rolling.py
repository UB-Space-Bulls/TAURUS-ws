"""Moving geometry must behave like the same terrain at the origin."""
from dataclasses import replace
import math
import numpy as np
import pytest
from terrain_geometry_impl.core import Config, process
from terrain_geometry_impl.rolling import rolling_config


def test_translated_ramp_preserves_geometry_and_missing_cells():
    cfg = Config(min_z=-2.2, resolution=0.5, width=12, height=12, origin_x=-3., origin_y=-3.)
    x, y = np.meshgrid(np.arange(-2.96, 3, 0.08), np.arange(-2.96, 3, 0.08))
    pts = np.column_stack((x.ravel(), y.ravel(), (x * math.tan(math.radians(15))).ravel()))
    pts = pts[~((pts[:, 0] > 1) & (pts[:, 1] > 1))]
    original = process(pts, cfg)
    moved_cfg = rolling_config(cfg, (100., -200., 11.2), 1.2)
    moved = process(pts + [100., -200., 10.], moved_cfg)
    np.testing.assert_array_equal(original.status, moved.status)
    np.testing.assert_allclose(original.slope, moved.slope, atol=1e-6, equal_nan=True)
    np.testing.assert_allclose(original.roughness, moved.roughness, atol=1e-6, equal_nan=True)
    np.testing.assert_allclose(original.height + 10, moved.height, atol=1e-5, equal_nan=True)
    assert cfg.seed_height == 0 and cfg.origin_x == -3
    assert np.any(moved.status == 0)


def test_origin_snaps_down_and_seed_tracks_actual_body():
    cfg = rolling_config(Config(resolution=0.5), (-0.01, 3.24, 5), .6)
    assert cfg.origin_x == -6.5 and cfg.origin_y == -3.
    assert cfg.seed_x == -.01 and cfg.seed_y == 3.24
    assert cfg.seed_height == 4.4
    assert cfg.min_z == 4 and cfg.max_z == 8


@pytest.mark.parametrize('position,clearance', [((0,0,0),0), ((0,0,0),-1),
    ((0,0,0),float('nan')), ((0,0,float('inf')),1), ((0,0),1)])
def test_invalid_pose_and_clearance_rejected(position, clearance):
    with pytest.raises(ValueError):
        rolling_config(Config(), position, clearance)


def test_moving_to_unobserved_area_does_not_retain_ground():
    cfg = rolling_config(Config(), (100., 200., 1.), 1.)
    r = process(np.empty((0,3)), cfg)
    assert not np.any(r.status)


def test_ground_must_be_inside_body_relative_height_band():
    with pytest.raises(ValueError, match='min_z/max_z'):
        rolling_config(Config(min_z=-1.), (0, 0, 0), 1.2)

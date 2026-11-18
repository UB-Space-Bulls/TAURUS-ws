"""The MDRS viewer must center translated patches and reject invalid fixtures."""
from pathlib import Path
import runpy

from launch import LaunchContext
from launch.actions import TimerAction
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest
import yaml


@pytest.fixture
def fixture_config(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    data = tmp_path / 'points.npz'
    data.touch()
    document = {
        'mdrs_patch_publisher': {'ros__parameters': {
            'data': data.name, 'center_x': 125.0, 'center_y': 150.0,
            'half_width': 12.0, 'frame': 'mdrs_map'}},
        'terrain_geometry': {'ros__parameters': {
            'target_frame': 'mdrs_map', 'seed_height': 8.5}},
    }
    path = tmp_path / 'patch.yaml'
    path.write_text(yaml.safe_dump(document))
    return path, document


def actions_for(path, **overrides):
    source = Path(__file__).parents[1] / 'launch/site_view.launch.py'
    context = LaunchContext()
    context.launch_configurations.update(
        config=str(path), rviz='true', processing_rate='1.0', test_duration='0.0')
    context.launch_configurations.update(overrides)
    return runpy.run_path(str(source))['setup'](context), context


def command(node, context):
    result = []
    for part in node.cmd:
        value = perform_substitutions(context, part)
        if value == '--ros-args':
            break
        result.append(value)
    return result


def test_translated_patch_has_known_view_origin_and_rviz(fixture_config):
    path, _ = fixture_config
    actions, context = actions_for(path)
    nodes = [action for action in actions if isinstance(action, Node)]
    origin = next(node for node in nodes if node.node_package == 'tf2_ros')
    arguments = command(origin, context)
    for flag, value in (('--x', '125.0'), ('--y', '150.0'), ('--z', '8.5'),
                        ('--frame-id', 'mdrs_map'),
                        ('--child-frame-id', 'mdrs_patch_origin')):
        assert arguments[arguments.index(flag) + 1] == value
    viewer = next(node for node in nodes if node.node_package == 'rviz2')
    assert any(value.endswith('mdrs_site.rviz') for value in command(viewer, context))
    assert not any(isinstance(action, TimerAction) for action in actions)


def test_finite_headless_mode_keeps_pipeline_without_viewer(fixture_config):
    path, _ = fixture_config
    actions, _ = actions_for(path, rviz='false', test_duration='15')
    assert not any(isinstance(action, Node) and action.node_package == 'rviz2'
                   for action in actions)
    assert any(isinstance(action, TimerAction) for action in actions)
    assert sum(isinstance(action, Node) and action.node_package == 'terrain_geometry'
               for action in actions) == 2


@pytest.mark.parametrize('key,value', [
    ('processing_rate', '0'), ('processing_rate', 'nan'),
    ('test_duration', '-1'), ('test_duration', 'inf')])
def test_invalid_rate_or_duration_is_rejected(fixture_config, key, value):
    path, _ = fixture_config
    with pytest.raises(ValueError, match=key):
        actions_for(path, **{key: value})


def test_mismatched_frames_are_rejected(fixture_config):
    path, document = fixture_config
    document['terrain_geometry']['ros__parameters']['target_frame'] = 'odom'
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match='must match'):
        actions_for(path)


def test_missing_point_data_is_rejected(fixture_config):
    path, document = fixture_config
    document['mdrs_patch_publisher']['ros__parameters']['data'] = 'missing.npz'
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match='point data does not exist'):
        actions_for(path)

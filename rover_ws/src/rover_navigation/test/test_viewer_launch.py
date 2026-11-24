"""Nested Nav2 viewer settings must not disable the combined launch's RViz."""
from pathlib import Path
import runpy

from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import pytest


@pytest.mark.parametrize('enabled', ['true', 'false'])
@pytest.mark.parametrize('launch_name,mesh', [
    ('terrain_rover.launch.py', True), ('fastlio_obstacle.launch.py', False)])
def test_outer_viewer_setting_survives_child_launch(enabled, launch_name, mesh, tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    path = Path(__file__).parents[1] / 'launch' / launch_name
    launch = runpy.run_path(str(path))['generate_launch_description']()
    context = LaunchContext()
    context.launch_configurations.update(rviz=enabled, gui='false', headless_rendering='true')
    nodes = []

    def visit(entity):
        condition = getattr(entity, 'condition', None)
        if condition is not None and not condition.evaluate(context):
            return
        if isinstance(entity, Node):
            nodes.append((entity.node_package, entity.node_executable))
            return
        if isinstance(entity, ExecuteProcess):
            return
        for child in entity.visit(context) or []:
            visit(child)

    visit(launch)
    assert context.launch_configurations['rviz'] == enabled
    assert nodes.count(('rviz2', 'rviz2')) == (1 if enabled == 'true' else 0)
    assert nodes.count(('lidar_terrain_description', 'mesh_marker')) == (
        1 if enabled == 'true' and mesh else 0)

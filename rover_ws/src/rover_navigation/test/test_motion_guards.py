"""Command-loss and stale-terrain regressions without a running simulator."""
from pathlib import Path
import runpy
from types import SimpleNamespace
from unittest.mock import patch

from geometry_msgs.msg import Twist, Vector3
from rover_swerve_controller.swerve_kinematics_node import MODULES, SwerveKinematicsNode


def test_missing_commands_stop_wheels_and_hold_steering():
    node = object.__new__(SwerveKinematicsNode)
    node._last_steer_angles = {name: 0.3 for name in MODULES}
    node._command_timeout = 0.5
    node._last_command_time = 10.0
    wheels, steering = [], []
    node._wheel_pub = SimpleNamespace(publish=wheels.append)
    node._steer_pub = SimpleNamespace(publish=steering.append)
    with patch('rover_swerve_controller.swerve_kinematics_node.time.monotonic', return_value=10.4):
        node._check_command_timeout()
        assert not wheels
    with patch('rover_swerve_controller.swerve_kinematics_node.time.monotonic', return_value=10.6):
        node._check_command_timeout()
    assert list(wheels[-1].data) == [0.0] * 4
    assert list(steering[-1].data) == [0.3] * 4


def gate():
    script = Path(__file__).parents[1] / 'scripts/terrain_velocity_gate.py'
    cls = runpy.run_path(str(script))['TerrainVelocityGate']
    node = object.__new__(cls)
    node.valid = True
    node.require_localization = False
    node.health_time = node.command_time = 10.0
    node.command = Twist(linear=Vector3(x=0.2))
    output = []
    node.publisher = SimpleNamespace(publish=output.append)
    return node, output


def test_fresh_terrain_and_command_pass_velocity():
    node, output = gate()
    with patch('time.monotonic', return_value=10.1):
        node.tick()
    assert output[-1].linear.x == 0.2


def test_false_or_missing_validity_stops_even_with_new_commands():
    node, output = gate()
    node.valid = False
    with patch('time.monotonic', return_value=10.1):
        node.tick()
    assert output[-1] == Twist()
    node.valid = True
    node.command_time = 12.0
    with patch('time.monotonic', return_value=12.1):
        node.tick()
    assert output[-1] == Twist()


def test_missing_command_stops_even_with_healthy_terrain():
    node, output = gate()
    node.health_time = 11.0
    with patch('time.monotonic', return_value=11.1):
        node.tick()
    assert output[-1] == Twist()


def test_localization_loss_stops_even_with_fresh_terrain():
    node, output = gate()
    node.require_localization = True
    node.localization_valid = False
    node.localization_time = 10.0
    with patch('time.monotonic', return_value=10.1):
        node.tick()
    assert output[-1] == Twist()
    node.localization_valid = True
    with patch('time.monotonic', return_value=10.1):
        node.tick()
    assert output[-1].linear.x == 0.2
    node.command_time = node.health_time = 12.0
    with patch('time.monotonic', return_value=12.1):
        node.tick()
    assert output[-1] == Twist()

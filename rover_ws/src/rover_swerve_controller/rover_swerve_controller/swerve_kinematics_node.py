import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Order must match the `joints:` lists in
# rover_description/config/rover_controllers.yaml.
MODULES = ("front_left", "front_right", "rear_left", "rear_right")

# Derived from the CAD-based rover_description/urdf/rover.urdf.xacro, not
# from a symmetric WHEEL_BASE/WHEEL_SEPARATION pair -- the real chassis is
# NOT symmetric front-to-back (front modules sit 0.3175m from center, rear
# modules sit 0.2921m from center). Values below are each *_steer_joint's
# <origin> in rover.urdf.xacro, expressed relative to chassis_link -- the
# raw CAD body frame, where +x = left and -y = front, swapped and
# sign-flipped from this file's own "x: forward, y: left" convention. That
# conversion (forward = -chassis_y, left = +chassis_x) is baked into the
# numbers below. (base_link itself is now REP-103 aligned via a fixed +90
# deg yaw joint in the URDF, but the steer joints still hang off
# chassis_link, so these stay in chassis_link terms.)
# Re-derive these from the URDF again if the chassis geometry changes.
MODULE_POSITIONS = {
    "front_left": (0.3175, 0.3556),
    "front_right": (0.3175, -0.3556),
    "rear_left": (-0.2921, 0.3556),
    "rear_right": (-0.2921, -0.3556),
}

# Measured directly off rover_description/meshes/wheel1_1.stl's bounding box
# (diameter ~203.2mm across the two axes perpendicular to the rolling axis)
# -- i.e. an 8in-diameter wheel, not the 0.15m (300mm-diameter) placeholder.
WHEEL_RADIUS = 0.1016

# Below this module speed, hold the last commanded steer angle instead of
# snapping to atan2(0, 0) == 0 -- avoids the wheels jittering back to
# "forward" every time the rover comes to a stop.
MIN_SPEED_FOR_STEERING = 1e-3


def _shortest_angle_diff(a: float, b: float) -> float:
    """Signed difference a - b, wrapped to (-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class SwerveKinematicsNode(Node):
    """Converts /cmd_vel into per-module steer-angle and wheel-speed commands.

    Includes wheel-angle-flip optimization: if the target steer angle is
    more than 90 deg from the module's current angle, it's cheaper to steer
    to the opposite angle instead (always <=90 deg away) and reverse the
    wheel direction, than to physically rotate the module the long way
    around (up to 180 deg) for the same net wheel velocity vector.
    """

    def __init__(self):
        super().__init__("swerve_kinematics_node")

        self._last_steer_angles = {name: 0.0 for name in MODULES}
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self._command_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        if not math.isfinite(self._command_timeout) or self._command_timeout <= 0:
            raise ValueError('cmd_vel_timeout must be finite and positive')
        self._last_command_time = time.monotonic()

        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_callback, 10)
        self._steer_pub = self.create_publisher(
            Float64MultiArray, "/steer_controller/commands", 10
        )
        self._wheel_pub = self.create_publisher(
            Float64MultiArray, "/wheel_controller/commands", 10
        )
        self.create_timer(0.1, self._check_command_timeout)

    def _check_command_timeout(self):
        if time.monotonic() - self._last_command_time > self._command_timeout:
            self._cmd_vel_callback(Twist())

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._last_command_time = time.monotonic()
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        steer_angles = []
        wheel_speeds = []

        for name in MODULES:
            x, y = MODULE_POSITIONS[name]
            module_vx = vx - omega * y
            module_vy = vy + omega * x
            speed = math.hypot(module_vx, module_vy)

            if speed > MIN_SPEED_FOR_STEERING:
                target_angle = math.atan2(module_vy, module_vx)
                last_angle = self._last_steer_angles[name]
                wheel_direction = 1.0

                if abs(_shortest_angle_diff(target_angle, last_angle)) > math.pi / 2:
                    target_angle = _shortest_angle_diff(target_angle + math.pi, 0.0)
                    wheel_direction = -1.0

                angle = target_angle
                self._last_steer_angles[name] = angle
            else:
                angle = self._last_steer_angles[name]
                wheel_direction = 1.0

            steer_angles.append(angle)
            wheel_speeds.append(wheel_direction * speed / WHEEL_RADIUS)

        self._steer_pub.publish(Float64MultiArray(data=steer_angles))
        self._wheel_pub.publish(Float64MultiArray(data=wheel_speeds))


def main(args=None):
    rclpy.init(args=args)
    node = SwerveKinematicsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

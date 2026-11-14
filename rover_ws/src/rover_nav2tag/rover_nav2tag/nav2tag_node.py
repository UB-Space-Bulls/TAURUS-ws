#!/usr/bin/env python3
"""
nav2tag_node.py
Autonomous AprilTag Navigation Node for the DEIMOS Mars Rover.

Migrated from Lyra's bringup/nav2tag.py to ROS 2 Jazzy.
Features:
- Configurable requested Tag ID via ROS 2 parameters and dynamic topic subscription.
- Strict tag filtering: detects and ignores non-matching tags; locks onto requested tag.
- TF-based 3D/2D pose lookup relative to the Nav2 map frame.
- Calculates a 0.5m standoff navigation goal directly in front of the tag, facing it.
- Communicates with Nav2's NavigateToPose action server.
- Supports runtime goal cancellation when requested tag changes.
"""

import math
import re
from typing import Optional, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    import rclpy.time
    from rcl_interfaces.msg import SetParametersResult

    from nav2_msgs.action import NavigateToPose
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Int32
    from action_msgs.msg import GoalStatus
    import tf2_ros
    ROS2_AVAILABLE = True
except ImportError:
    # Fallback placeholders for environments without ROS 2 installed (e.g. macOS host dev)
    ROS2_AVAILABLE = False
    class Node:  # type: ignore
        def __init__(self, name: str):
            self._name = name
    class GoalStatus:  # type: ignore
        STATUS_UNKNOWN = 0
        STATUS_ACCEPTED = 1
        STATUS_EXECUTING = 2
        STATUS_CANCELING = 3
        STATUS_SUCCEEDED = 4
        STATUS_CANCELED = 5
        STATUS_ABORTED = 6
    class SetParametersResult:  # type: ignore
        def __init__(self, successful=True):
            self.successful = successful
    class PoseStamped:  # type: ignore
        pass
    class Int32:  # type: ignore
        pass


def parse_tag_frame_id(frame_name: str, tag_family: str = "tag36h11") -> Optional[int]:
    """
    Extract the numeric Tag ID from a TF frame name.
    Supports formats like:
      - tag36h11:5, tag36h11_5, tag36h11/5, tag36h11-5
      - tag:5, tag_5, tag5
    Returns the integer tag ID, or None if the frame does not match.
    """
    # Pattern 1: Specified tag family prefix, e.g. tag36h11:5 or tag36h11_5
    pattern_family = rf"^{re.escape(tag_family)}[:_/\-]?(\d+)$"
    match = re.match(pattern_family, frame_name)
    if match:
        return int(match.group(1))

    # Pattern 2: Generic tag prefix, e.g. tag:5 or tag_5 or tag5
    pattern_generic = r"^tag[:_/\-]?(\d+)$"
    match = re.match(pattern_generic, frame_name)
    if match:
        return int(match.group(1))

    return None


def tag_approach_heading(
    qx: float, qy: float, qz: float, qw: float, min_horizontal: float = 0.3
) -> Optional[float]:
    """
    Return the map-frame heading (radians) a rover should drive to approach
    the tag head-on, from the tag's orientation quaternion (map -> tag).

    AprilTag frame convention: x right along the tag face, y down,
    z pointing INTO the tag (away from whoever is looking at it). So the
    tag's z-axis, flattened onto the ground plane, is exactly the direction
    "from the viewer toward the tag". (Using the x-axis here would be wrong:
    it lies along the tag face, which sends the rover off to the side.)

    Returns None if the z-axis is near-vertical (tag lying flat / facing
    up or down), since there's no meaningful horizontal approach direction.
    """
    # Third column of the rotation matrix = tag z-axis expressed in map frame
    zx = 2.0 * (qx * qz + qw * qy)
    zy = 2.0 * (qy * qz - qw * qx)
    if math.hypot(zx, zy) < min_horizontal:
        return None
    return math.atan2(zy, zx)


def calculate_standoff_pose(
    tx: float,
    ty: float,
    yaw: float,
    stop_distance: float = 0.5,
    approach_angle_offset: float = 0.0,
) -> Tuple[float, float, float, float]:
    """
    Compute the (x, y, qz, qw) 2D goal pose in front of the tag.

    `yaw` is the approach heading (see `tag_approach_heading`): the direction
    pointing from the open side of the tag INTO the tag face. The goal sits
    `stop_distance` back from the tag along that direction, and the rover
    faces along it — i.e. straight at the tag.
    """
    # Back up stop_distance from the tag, against the approach direction
    goal_x = tx - stop_distance * math.cos(yaw)
    goal_y = ty - stop_distance * math.sin(yaw)

    # Rover faces along the approach direction (toward the tag) + fine-tuning offset
    facing_yaw = yaw + approach_angle_offset
    # Normalize facing_yaw to [-pi, pi]
    facing_yaw = math.atan2(math.sin(facing_yaw), math.cos(facing_yaw))

    qz = math.sin(facing_yaw / 2.0)
    qw = math.cos(facing_yaw / 2.0)

    return goal_x, goal_y, qz, qw


class NavigateToTag(Node):
    """
    DEIMOS Autonomous Tag Navigation Node.
    """

    def __init__(self):
        super().__init__("navigate_to_tag")

        if not ROS2_AVAILABLE:
            raise RuntimeError("rclpy is not available in the current environment.")

        # -------------------------------------------------------------
        # 1. ROS 2 Parameters
        # -------------------------------------------------------------
        self.declare_parameter("target_tag_id", 5)  # Default: target Tag 5 (-1 = any tag)
        self.declare_parameter("tag_family", "tag36h11")
        self.declare_parameter("stop_distance", 0.5)  # 0.5 m standoff
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("check_map_odom", True)
        self.declare_parameter("nav2_action_name", "navigate_to_pose")
        self.declare_parameter("timer_period", 1.0)
        self.declare_parameter("approach_angle_offset", 0.0)

        self.target_tag_id = self.get_parameter("target_tag_id").get_parameter_value().integer_value
        self.tag_family = self.get_parameter("tag_family").get_parameter_value().string_value
        self.stop_distance = self.get_parameter("stop_distance").get_parameter_value().double_value
        self.map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self.odom_frame = self.get_parameter("odom_frame").get_parameter_value().string_value
        self.check_map_odom = self.get_parameter("check_map_odom").get_parameter_value().bool_value
        self.nav2_action_name = self.get_parameter("nav2_action_name").get_parameter_value().string_value
        timer_period = self.get_parameter("timer_period").get_parameter_value().double_value
        self.approach_angle_offset = (
            self.get_parameter("approach_angle_offset").get_parameter_value().double_value
        )

        self.add_on_set_parameters_callback(self.parameters_callback)

        # -------------------------------------------------------------
        # 2. Dynamic Tag Selection Topic
        # -------------------------------------------------------------
        # Mission logic or CLI can publish: ros2 topic pub /target_tag_id std_msgs/msg/Int32 "{data: 5}"
        self.tag_sub = self.create_subscription(
            Int32,
            "target_tag_id",
            self.target_tag_callback,
            10,
        )

        # Visual debug topic: displays computed goal pose in RViz2
        self.goal_pub = self.create_publisher(PoseStamped, "nav2tag/current_goal", 10)

        # -------------------------------------------------------------
        # 3. Action Client & TF Listener
        # -------------------------------------------------------------
        self.nav_client = ActionClient(self, NavigateToPose, self.nav2_action_name)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State tracking
        self.goal_active = False
        self.current_goal_handle = None
        self.active_tag_frame = None

        # Periodic navigation loop
        self.timer = self.create_timer(timer_period, self.try_navigate)

        self.get_logger().info(
            f"DEIMOS nav2tag initialized! Looking for target Tag ID: "
            f"{self.target_tag_id if self.target_tag_id >= 0 else 'ANY'}, "
            f"tag family: '{self.tag_family}', stop distance: {self.stop_distance:.2f}m"
        )

    def parameters_callback(self, params):
        """Handle runtime parameter updates (e.g. via ros2 param set)."""
        for param in params:
            if param.name == "target_tag_id":
                new_id = param.value
                if new_id != self.target_tag_id:
                    self.get_logger().info(
                        f"Target Tag ID updated via parameter: {self.target_tag_id} -> {new_id}"
                    )
                    self.set_target_tag(new_id)
            elif param.name == "stop_distance":
                self.stop_distance = float(param.value)
                self.get_logger().info(f"Stop distance updated to: {self.stop_distance:.2f}m")
            elif param.name == "tag_family":
                self.tag_family = str(param.value)
            elif param.name == "approach_angle_offset":
                self.approach_angle_offset = float(param.value)

        return SetParametersResult(successful=True)

    def target_tag_callback(self, msg: Int32):
        """Handle runtime tag target change from topic."""
        new_id = msg.data
        if new_id != self.target_tag_id:
            self.get_logger().info(
                f"Received new target Tag ID via /target_tag_id topic: {new_id}"
            )
            self.set_target_tag(new_id)

    def set_target_tag(self, new_id: int):
        """Update target tag and cancel existing navigation goal if target changed."""
        self.target_tag_id = new_id
        if self.goal_active and self.current_goal_handle is not None:
            self.get_logger().info("Cancelling active Nav2 goal for previous tag...")
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"Failed to cancel active goal: {e}")
        self.goal_active = False
        self.current_goal_handle = None
        self.active_tag_frame = None

    def find_target_tag_in_tf(self) -> Optional[str]:
        """
        Scan all frames in the TF tree.
        Extract tag IDs and filter against self.target_tag_id.
        - If target_tag_id is specified (>= 0):
          Ignores any tag whose ID does not match.
          Returns the frame name of the matching tag when detected.
        - If target_tag_id < 0:
          Wildcard mode: returns the first detected tag.
        """
        try:
            frames_yaml = self.tf_buffer.all_frames_as_yaml()
        except Exception as e:
            self.get_logger().warn(f"Could not query TF frames: {e}", throttle_duration_sec=3.0)
            return None

        detected_tags = []

        for line in frames_yaml.splitlines():
            line = line.strip()
            # In all_frames_as_yaml, frame keys appear as "frame_name:"
            if line.endswith(":"):
                frame_name = line[:-1]
                tag_id = parse_tag_frame_id(frame_name, self.tag_family)
                if tag_id is not None:
                    detected_tags.append((frame_name, tag_id))

        if not detected_tags:
            return None

        # Check each detected tag against requested ID
        for frame_name, tag_id in detected_tags:
            if self.target_tag_id >= 0:
                if tag_id == self.target_tag_id:
                    self.get_logger().info(
                        f"Target Tag MATCH: found '{frame_name}' (ID {tag_id})!",
                        throttle_duration_sec=2.0,
                    )
                    return frame_name
                else:
                    self.get_logger().info(
                        f"Ignoring detected tag '{frame_name}' (ID {tag_id}) — "
                        f"actively seeking target Tag ID {self.target_tag_id}",
                        throttle_duration_sec=3.0,
                    )
            else:
                # Wildcard mode
                self.get_logger().info(
                    f"Wildcard mode: accepting detected tag '{frame_name}' (ID {tag_id})",
                    throttle_duration_sec=2.0,
                )
                return frame_name

        return None

    def try_navigate(self):
        """Periodic loop to check TF tree, filter tags, and dispatch Nav2 goal."""
        if self.goal_active:
            return

        # 1. Verify SLAM / map->odom transform is available
        if self.check_map_odom:
            try:
                self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.odom_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5),
                )
            except Exception as e:
                self.get_logger().warn(
                    f"{self.map_frame}→{self.odom_frame} not available — "
                    f"is SLAM (slam_toolbox) running? ({e})",
                    throttle_duration_sec=3.0,
                )
                return

        # 2. Find requested tag in TF
        tag_frame = self.find_target_tag_in_tf()
        if tag_frame is None:
            self.get_logger().warn(
                f"Waiting for target Tag ID "
                f"{self.target_tag_id if self.target_tag_id >= 0 else 'ANY'} "
                f"in TF tree...",
                throttle_duration_sec=3.0,
            )
            return

        # 3. Lookup tag transform in map frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                tag_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
        except tf2_ros.LookupException:
            self.get_logger().warn(f"{tag_frame} disappeared from TF", throttle_duration_sec=2.0)
            return
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f"TF extrapolation for {tag_frame}: {e}", throttle_duration_sec=2.0)
            return
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warn(f"TF connectivity error: {e}", throttle_duration_sec=2.0)
            return
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed for {tag_frame}: {e}", throttle_duration_sec=2.0)
            return

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y

        q = transform.transform.rotation
        yaw = tag_approach_heading(q.x, q.y, q.z, q.w)
        if yaw is None:
            self.get_logger().warn(
                f"{tag_frame} is facing up/down (z-axis near vertical) — "
                f"no horizontal approach direction, skipping",
                throttle_duration_sec=3.0,
            )
            return

        # 4. Calculate standoff goal pose
        goal_x, goal_y, qz, qw = calculate_standoff_pose(
            tx=tx,
            ty=ty,
            yaw=yaw,
            stop_distance=self.stop_distance,
            approach_angle_offset=self.approach_angle_offset,
        )

        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        self.get_logger().info(
            f"Target {tag_frame} at ({tx:.2f}, {ty:.2f}, approach heading={math.degrees(yaw):.1f}°) → "
            f"Commanding Nav2 goal at ({goal_x:.2f}, {goal_y:.2f}) "
            f"with standoff {self.stop_distance:.2f}m"
        )

        # Publish for RViz debugging
        self.goal_pub.publish(goal)

        # 5. Send goal to Nav2
        self.active_tag_frame = tag_frame
        self.send_goal(goal)

    def send_goal(self, pose: PoseStamped):
        """Send NavigateToPose action goal to Nav2."""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"Nav2 action server '{self.nav2_action_name}' not available!"
            )
            self.goal_active = False
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f"Dispatching goal to Nav2 '{self.nav2_action_name}'...")
        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_cb,
        )
        future.add_done_callback(self.goal_response_cb)
        self.goal_active = True

    def goal_response_cb(self, future):
        """Callback when Nav2 accepts or rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2 — will retry on next cycle")
            self.goal_active = False
            self.current_goal_handle = None
            return

        self.get_logger().info("Nav2 goal accepted! Driving toward tag...")
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        """Callback when Nav2 completes navigation."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"Successfully reached target {self.active_tag_frame}! "
                f"Rover is parked {self.stop_distance:.2f}m in front of the tag."
            )
            # Stay stopped; remain active until target is changed
            self.goal_active = True
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Navigation goal was cancelled.")
            self.goal_active = False
            self.current_goal_handle = None
        else:
            self.get_logger().warn(
                f"Navigation ended with status code {status} — will retry"
            )
            self.goal_active = False
            self.current_goal_handle = None

    def feedback_cb(self, feedback_msg):
        """Nav2 periodic distance feedback."""
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f"Navigating: {dist:.2f}m remaining to target tag",
            throttle_duration_sec=1.5,
        )


def main(args=None):
    if not ROS2_AVAILABLE:
        print("Error: rclpy is required to run nav2tag_node.")
        return
    rclpy.init(args=args)
    node = NavigateToTag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

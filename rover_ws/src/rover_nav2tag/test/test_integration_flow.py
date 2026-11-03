#!/usr/bin/env python3
"""
test_integration_flow.py
Simulates the complete end-to-end mission flow for nav2tag:

Flow:
    Requested Tag (Tag 5)
          ↓
    Detect Tag 3 → IGNORE
    Detect Tag 5 → MATCH
          ↓
    Create Nav2 Goal (0.5m standoff facing tag)
          ↓
    Send to Nav2 (navigate_to_pose action)
          ↓
    Receive feedback & reach goal
"""

import math
import sys
import os
import unittest
from typing import Dict, Any

# Ensure rover_nav2tag is importable
current_dir = os.path.dirname(os.path.abspath(__file__))
pkg_dir = os.path.dirname(current_dir)
sys.path.insert(0, pkg_dir)

from rover_nav2tag.nav2tag_node import parse_tag_frame_id, calculate_standoff_pose


class MockTransform:
    def __init__(self, x: float, y: float, z: float, yaw: float):
        self.translation_x = x
        self.translation_y = y
        self.translation_z = z
        self.yaw = yaw

    @property
    def rotation_qz(self):
        return math.sin(self.yaw / 2.0)

    @property
    def rotation_qw(self):
        return math.cos(self.yaw / 2.0)


class MockTFBuffer:
    def __init__(self):
        self.frames: Dict[str, MockTransform] = {}

    def add_transform(self, child_frame: str, x: float, y: float, z: float, yaw: float):
        self.frames[child_frame] = MockTransform(x, y, z, yaw)

    def all_frames_as_yaml(self) -> str:
        lines = []
        for name in self.frames.keys():
            lines.append(f"  {name}:")
        return "\n".join(lines)

    def lookup_transform(self, parent: str, child: str):
        if child not in self.frames:
            raise KeyError(f"Frame {child} not found in mock TF tree")
        return self.frames[child]


class MockNav2ActionServer:
    def __init__(self):
        self.goals_received = []
        self.active_goal = None
        self.cancelled_goals = []

    def send_goal(self, goal_data: Dict[str, Any]):
        self.goals_received.append(goal_data)
        self.active_goal = goal_data

    def cancel_active_goal(self):
        if self.active_goal:
            self.cancelled_goals.append(self.active_goal)
            self.active_goal = None


class MockNav2TagWorkflow:
    """Simulates the state machine and decisions of NavigateToTag without live ROS network."""

    def __init__(self, tf_buffer: MockTFBuffer, nav2_server: MockNav2ActionServer, target_tag_id: int = 5, stop_distance: float = 0.5):
        self.tf_buffer = tf_buffer
        self.nav2_server = nav2_server
        self.target_tag_id = target_tag_id
        self.stop_distance = stop_distance
        self.goal_active = False
        self.log_history = []

    def log(self, msg: str):
        self.log_history.append(msg)

    def scan_for_tag(self):
        frames_yaml = self.tf_buffer.all_frames_as_yaml()
        detected_tags = []
        for line in frames_yaml.splitlines():
            line = line.strip()
            if line.endswith(":"):
                frame_name = line[:-1]
                tag_id = parse_tag_frame_id(frame_name)
                if tag_id is not None:
                    detected_tags.append((frame_name, tag_id))

        for frame_name, tag_id in detected_tags:
            if self.target_tag_id >= 0:
                if tag_id == self.target_tag_id:
                    self.log(f"MATCH: Tag {tag_id} found in frame {frame_name}")
                    return frame_name
                else:
                    self.log(f"IGNORE: Tag {tag_id} (frame {frame_name}) != requested {self.target_tag_id}")
            else:
                self.log(f"WILDCARD: Tag {tag_id} accepted")
                return frame_name
        return None

    def execute_cycle(self):
        if self.goal_active:
            return "ALREADY_ACTIVE"

        tag_frame = self.scan_for_tag()
        if not tag_frame:
            return "NO_MATCHING_TAG"

        tf = self.tf_buffer.lookup_transform("map", tag_frame)
        gx, gy, qz, qw = calculate_standoff_pose(
            tf.translation_x,
            tf.translation_y,
            tf.yaw,
            stop_distance=self.stop_distance,
        )

        goal = {
            "tag_frame": tag_frame,
            "target_tag_id": self.target_tag_id,
            "goal_x": gx,
            "goal_y": gy,
            "qz": qz,
            "qw": qw,
            "standoff": self.stop_distance,
        }
        self.nav2_server.send_goal(goal)
        self.goal_active = True
        return "GOAL_DISPATCHED"

    def update_target_tag(self, new_tag_id: int):
        self.target_tag_id = new_tag_id
        if self.goal_active:
            self.nav2_server.cancel_active_goal()
            self.goal_active = False


class TestNav2TagIntegration(unittest.TestCase):
    def setUp(self):
        self.tf_buffer = MockTFBuffer()
        # Decoy Tag 3
        self.tf_buffer.add_transform("tag36h11:3", x=2.0, y=1.0, z=0.0, yaw=0.0)
        # Target Tag 5
        self.tf_buffer.add_transform("tag36h11:5", x=6.0, y=3.0, z=0.0, yaw=0.0)

        self.nav2_server = MockNav2ActionServer()

    def test_end_to_end_tag_5_targeted_and_tag_3_ignored(self):
        workflow = MockNav2TagWorkflow(
            tf_buffer=self.tf_buffer,
            nav2_server=self.nav2_server,
            target_tag_id=5,
            stop_distance=0.5,
        )

        status = workflow.execute_cycle()
        self.assertEqual(status, "GOAL_DISPATCHED")

        # Verify logs show Tag 3 was ignored and Tag 5 was matched
        logs = "\n".join(workflow.log_history)
        self.assertIn("IGNORE: Tag 3 (frame tag36h11:3) != requested 5", logs)
        self.assertIn("MATCH: Tag 5 found in frame tag36h11:5", logs)

        # Verify goal dispatched to Nav2
        self.assertEqual(len(self.nav2_server.goals_received), 1)
        goal = self.nav2_server.goals_received[0]
        self.assertEqual(goal["tag_frame"], "tag36h11:5")
        self.assertEqual(goal["target_tag_id"], 5)
        self.assertAlmostEqual(goal["goal_x"], 5.5)  # 6.0 - 0.5 = 5.5
        self.assertAlmostEqual(goal["goal_y"], 3.0)

    def test_dynamic_retargeting(self):
        workflow = MockNav2TagWorkflow(
            tf_buffer=self.tf_buffer,
            nav2_server=self.nav2_server,
            target_tag_id=5,
            stop_distance=0.5,
        )
        workflow.execute_cycle()
        self.assertEqual(len(self.nav2_server.goals_received), 1)

        # Operator changes target tag to 3
        workflow.update_target_tag(3)
        self.assertEqual(len(self.nav2_server.cancelled_goals), 1)

        # Next cycle targets Tag 3
        status = workflow.execute_cycle()
        self.assertEqual(status, "GOAL_DISPATCHED")
        self.assertEqual(len(self.nav2_server.goals_received), 2)
        goal2 = self.nav2_server.goals_received[1]
        self.assertEqual(goal2["tag_frame"], "tag36h11:3")
        self.assertAlmostEqual(goal2["goal_x"], 1.5)  # 2.0 - 0.5 = 1.5


if __name__ == "__main__":
    print("=" * 60)
    print("Running DEIMOS nav2tag integration flow test...")
    print("=" * 60)
    suite = unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    if result.wasSuccessful():
        print("\nAll integration flow tests PASSED!")
        sys.exit(0)
    else:
        print("\nIntegration tests FAILED.")
        sys.exit(1)

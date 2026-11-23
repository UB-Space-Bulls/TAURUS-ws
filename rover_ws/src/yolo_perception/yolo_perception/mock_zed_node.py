#!/usr/bin/env python3
"""
Stand-in for the ZED camera: publishes frames from a webcam, video, image or image folder.

Lets you test the detector on a laptop with no ZED attached, e.g. feed a folder of
hammer photos and watch the boxes in Foxglove.

  ros2 run yolo_perception mock_zed_node --ros-args -p source:=~/test_media/hammers -p fps:=2.0

source: webcam index ('0'), video file, single image, or a directory of images (cycled).
"""

import os
from pathlib import Path

import cv2
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

IMAGE_EXTS = ('.jpg', '.jpeg', '.png', '.bmp', '.webp')


class MockZedNode(Node):
    def __init__(self):
        super().__init__('mock_zed_node')
        # dynamic_typing: `source:=0` and `fps:=2` arrive as ints after YAML parsing
        loose = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('source', '0', loose)
        self.declare_parameter('topic', '/zed2/zed_node/rgb/color/rect/image')
        self.declare_parameter('fps', 30.0, loose)
        self.declare_parameter('frame_id', 'zed_left_camera_optical_frame')

        source = os.path.expanduser(str(self.get_parameter('source').value))
        topic = self.get_parameter('topic').value
        fps = float(self.get_parameter('fps').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.bridge = CvBridge()
        self.cap = None
        self.images = []
        self.image_idx = 0

        if source.isdigit():
            self.cap = cv2.VideoCapture(int(source))
        elif os.path.isdir(source):
            self.images = sorted(
                str(p) for p in Path(source).iterdir() if p.suffix.lower() in IMAGE_EXTS)
        elif source.lower().endswith(IMAGE_EXTS):
            self.images = [source]
        else:
            self.cap = cv2.VideoCapture(source)

        if self.cap is not None and not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera/video '{source}'")
        if self.cap is None and not self.images:
            raise RuntimeError(f"No images found at '{source}'")

        self.pub = self.create_publisher(Image, topic, 10)
        self.timer = self.create_timer(1.0 / fps, self.publish_frame)
        what = f'{len(self.images)} image(s)' if self.images else 'video stream'
        self.get_logger().info(f'Publishing {what} from {source} to {topic} at {fps} FPS')

    def next_frame(self):
        if self.images:
            path = self.images[self.image_idx % len(self.images)]
            self.image_idx += 1
            frame = cv2.imread(path)
            if frame is None:
                self.get_logger().warn(f'Could not read {path}, skipping')
            return frame
        ret, frame = self.cap.read()
        if not ret:  # end of a video file: loop
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()
        return frame if ret else None

    def publish_frame(self):
        frame = self.next_frame()
        if frame is None:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockZedNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Run a YOLO model on the ZED camera stream and publish detections.

Subscribes:
  image_topic (sensor_msgs/Image)          default /zed2/zed_node/rgb/color/rect/image
                                           (ZED wrapper 5.x name used by rover_bringup)
Publishes (prefix = `topic_prefix`, default /yolo):
  <prefix>/detections     vision_msgs/Detection2DArray  -> for mission/autonomy logic
  <prefix>/annotations    foxglove_msgs/ImageAnnotations -> boxes drawn over the image in Foxglove
  <prefix>/annotated_image sensor_msgs/Image            -> only rendered when someone subscribes

Which model runs is picked by the `profile` parameter (terrain / hammer /
keyboard / coco, see config/models.yaml). Any of model_path, conf_thres,
iou_thres, imgsz, target_classes set explicitly overrides the profile.
"""

import time

from cv_bridge import CvBridge
from foxglove_msgs.msg import Color, ImageAnnotations, Point2, PointsAnnotation, TextAnnotation
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from yolo_perception.profiles import class_ids_from_names, get_profile, resolve_weights

# Distinct colours so neighbouring classes (e.g. keys) are easy to tell apart.
PALETTE = [
    (0.00, 1.00, 0.25), (1.00, 0.60, 0.00), (0.20, 0.60, 1.00), (1.00, 0.20, 0.60),
    (0.60, 0.30, 1.00), (1.00, 1.00, 0.00), (0.00, 0.90, 0.90), (1.00, 0.35, 0.35),
]


class ZedYoloNode(Node):
    def __init__(self):
        super().__init__('zed_yolo_node')

        self.declare_parameter('profile', 'terrain')
        self.declare_parameter('profiles_file', '')  # '' = installed config/models.yaml
        # '' = $YOLO_WEIGHTS_DIR or ~/yolo_weights
        self.declare_parameter('weights_dir', '')
        self.declare_parameter('image_topic', '/zed2/zed_node/rgb/color/rect/image')
        self.declare_parameter('topic_prefix', '/yolo')
        # dynamic_typing: accept `device:=0` / `conf_thres:=0` (YAML turns them into ints)
        loose = ParameterDescriptor(dynamic_typing=True)
        # '' = auto (GPU if available), 'cpu', '0'
        self.declare_parameter('device', '', loose)
        # Overrides — leave at the sentinel value to use the profile's setting.
        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_thres', -1.0, loose)
        self.declare_parameter('iou_thres', -1.0, loose)
        self.declare_parameter('imgsz', 0)
        self.declare_parameter('target_classes', [-1])    # [-1] = profile's classes

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        profile = get_profile(gp('profile'), gp('profiles_file') or None)
        self.profile_name = profile.name

        weights = gp('model_path') or profile.weights
        self.model_path = resolve_weights(weights, gp('weights_dir') or None)
        conf, iou = float(gp('conf_thres')), float(gp('iou_thres'))
        self.conf_thres = conf if conf >= 0 else profile.conf
        self.iou_thres = iou if iou >= 0 else profile.iou
        self.imgsz = gp('imgsz') if gp('imgsz') > 0 else profile.imgsz
        self.device = str(gp('device')) or None

        self.get_logger().info(f"Profile '{profile.name}': loading {self.model_path}")
        try:
            self.model = YOLO(self.model_path)
        except Exception as e:
            raise RuntimeError(
                f"Could not load YOLO weights '{self.model_path}'. Put the .pt file in "
                f'~/yolo_weights (or set weights_dir / $YOLO_WEIGHTS_DIR / model_path). '
                f'Cause: {e}')

        raw_classes = [int(c) for c in gp('target_classes')]
        if raw_classes and min(raw_classes) >= 0:
            unknown = [c for c in raw_classes if c not in self.model.names]
            if unknown:
                raise ValueError(
                    f'target_classes {unknown} not in model classes {self.model.names}')
            self.target_classes = raw_classes
        else:
            self.target_classes = class_ids_from_names(self.model.names, profile.classes)
        kept = ('all' if self.target_classes is None
                else [self.model.names[i] for i in self.target_classes])
        self.get_logger().info(
            f'Classes: {kept} | conf={self.conf_thres} iou={self.iou_thres} imgsz={self.imgsz}')

        self.bridge = CvBridge()
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        image_topic = gp('image_topic')
        self.sub = self.create_subscription(Image, image_topic, self.image_callback, sensor_qos)

        prefix = gp('topic_prefix').rstrip('/')
        self.det_pub = self.create_publisher(Detection2DArray, f'{prefix}/detections', 10)
        self.ann_pub = self.create_publisher(ImageAnnotations, f'{prefix}/annotations', 10)
        self.img_pub = self.create_publisher(Image, f'{prefix}/annotated_image', 10)
        self.get_logger().info(f'Listening on {image_topic}, publishing under {prefix}/')

        self.frame_count = 0
        self.delay_sum_ms = 0.0
        self.last_report_time = time.time()
        self.got_image = False
        self.image_topic = image_topic
        self.watchdog = self.create_timer(5.0, self._warn_if_no_images)

    def _warn_if_no_images(self):
        if self.got_image:
            self.watchdog.cancel()
            return
        self.get_logger().warn(
            f'No images on {self.image_topic} yet. Check `ros2 topic list | grep image` '
            'and set image_topic:=... if the camera uses another name.')

    def image_callback(self, msg: Image):
        self.got_image = True
        t0 = time.perf_counter()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        result = self.model.predict(
            source=frame, conf=self.conf_thres, iou=self.iou_thres, imgsz=self.imgsz,
            classes=self.target_classes, device=self.device, verbose=False)[0]

        boxes = result.boxes
        xyxy = boxes.xyxy.cpu().numpy() if len(boxes) else []
        cls_ids = boxes.cls.cpu().numpy().astype(int) if len(boxes) else []
        confs = boxes.conf.cpu().numpy() if len(boxes) else []

        det_array = Detection2DArray(header=msg.header)
        ann_msg = ImageAnnotations()
        for (x1, y1, x2, y2), cls_id, conf in zip(xyxy, cls_ids, confs):
            label = str(self.model.names[int(cls_id)])
            det_array.detections.append(self._detection(msg, label, conf, x1, y1, x2, y2))
            self._annotate(ann_msg, msg, int(cls_id), label, conf, x1, y1, x2, y2)

        self.det_pub.publish(det_array)
        self.ann_pub.publish(ann_msg)

        if self.img_pub.get_subscription_count() > 0:
            out = self.bridge.cv2_to_imgmsg(result.plot(), encoding='bgr8')
            out.header = msg.header
            self.img_pub.publish(out)

        self._report_speed(t0, len(det_array.detections))

    @staticmethod
    def _detection(msg, label, conf, x1, y1, x2, y2) -> Detection2D:
        det = Detection2D(header=msg.header)
        det.bbox.center.position.x = float(x1 + x2) / 2.0
        det.bbox.center.position.y = float(y1 + y2) / 2.0
        det.bbox.size_x = float(x2 - x1)
        det.bbox.size_y = float(y2 - y1)
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = label
        hyp.hypothesis.score = float(conf)
        det.results.append(hyp)
        return det

    @staticmethod
    def _annotate(ann_msg, msg, cls_id, label, conf, x1, y1, x2, y2):
        r, g, b = PALETTE[cls_id % len(PALETTE)]
        x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)
        rect = PointsAnnotation()
        rect.timestamp = msg.header.stamp
        rect.type = PointsAnnotation.LINE_LOOP
        rect.points = [Point2(x=x1, y=y1), Point2(x=x2, y=y1),
                       Point2(x=x2, y=y2), Point2(x=x1, y=y2)]
        rect.outline_color = Color(r=r, g=g, b=b, a=1.0)
        rect.fill_color = Color(r=r, g=g, b=b, a=0.12)
        rect.thickness = 3.0
        ann_msg.points.append(rect)

        txt = TextAnnotation()
        txt.timestamp = msg.header.stamp
        txt.position = Point2(x=x1, y=max(18.0, y1 - 6.0))
        txt.text = f'{label} ({conf:.2f})'
        txt.font_size = 16.0
        txt.text_color = Color(r=1.0, g=1.0, b=1.0, a=1.0)
        txt.background_color = Color(r=r * 0.5, g=g * 0.5, b=b * 0.5, a=0.85)
        ann_msg.texts.append(txt)

    def _report_speed(self, t0, n_objects):
        self.delay_sum_ms += (time.perf_counter() - t0) * 1000.0
        self.frame_count += 1
        now = time.time()
        if now - self.last_report_time >= 2.0:
            fps = self.frame_count / (now - self.last_report_time)
            avg_delay = self.delay_sum_ms / self.frame_count
            self.get_logger().info(
                f'[{self.profile_name}] {fps:5.1f} FPS | {avg_delay:5.1f} ms/frame '
                f'| objects: {n_objects}')
            self.frame_count = 0
            self.delay_sum_ms = 0.0
            self.last_report_time = now


def main(args=None):
    rclpy.init(args=args)
    node = ZedYoloNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

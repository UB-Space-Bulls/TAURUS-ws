#!/usr/bin/env python3
"""ZED wrapper image topic -> URC gesture pipeline -> intent JSON.

Replaces the cv2.VideoCapture loop in urc_gesture_test.py with a ROS 2 subscriber.
All gesture logic (classify / GestureDebouncer / TargetLock / crop refinement) is the
unmodified code from urc_gesture_test.py, which must sit next to this file.

Publishes
  ~intent_topic        std_msgs/String  JSON intent, only when a gesture is confirmed
                                        (same payload as send_command(): command, label,
                                        sequence, track_id, source_time, range_mode for FETCH)
  ~status_topic        std_msgs/String  JSON, every processed frame
  ~debug_image_topic   sensor_msgs/Image annotated frame (only rendered if subscribed)

Subscribes
  image_topic          sensor_msgs/Image   ZED left rectified colour image
  ~select_target       std_msgs/Int32      lock track id (-1 clears, like the 'r' key)
  ~enable              std_msgs/Bool       pause/resume inference
"""
import json
import math
import time
import uuid

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String

from .image_utils import fill_bgr_image, image_to_bgr

try:
    from .urc_gesture_test import (
        COMMANDS, L_HIP, R_HIP, REQUIRED, GeometryConfig, GestureDebouncer, TargetLock,
        classify, crop_bounds, draw_status_panel, match_crop_pose, pose_arrays,
        pose_quality, send_command, status_rows)
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "zed_gesture/urc_gesture_test.py is missing. Copy it unmodified from "
        "https://github.com/dngofficial/gesture_test_urc into this package directory.") from exc


class ZedGestureNode(Node):
    def __init__(self):
        super().__init__("zed_gesture_node")
        P = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731

        image_topic = P("image_topic", "/zed/zed_node/rgb/image_rect_color")
        self.publish_debug = P("publish_debug", True)
        self.debug_scale = float(P("debug_scale", 0.5))

        long_range, crop_pose = P("long_range", True), P("crop_pose", False)
        imgsz = int(P("imgsz", 0))
        self.imgsz = imgsz if imgsz > 0 else (960 if long_range else 640)
        self.pose_imgsz = int(P("pose_imgsz", 640))
        self.tracker = P("tracker", "botsort.yaml")
        self.refine_pose = bool(crop_pose or long_range)

        self.geometry = GeometryConfig(
            kp_conf=float(P("kp_conf", 0.5)), min_torso_px=float(P("min_torso_px", 12.0)),
            require_hips=bool(P("require_hips", False)), up_deg=float(P("up_angle", 45.0)),
            up_straight_deg=float(P("up_elbow", 135.0)),
            camera_upright=not bool(P("shoulder_frame", False)))
        self.required_joints = REQUIRED + [L_HIP, R_HIP] if self.geometry.require_hips else REQUIRED

        self.deb = GestureDebouncer(float(P("hold", 0.7)), float(P("release", 0.4)),
                                    float(P("cooldown", 1.2)), float(P("stop_hold", 0.35)),
                                    float(P("max_gap", 0.25)))
        self.auto_target = bool(P("auto_target", True))
        initial_id = int(P("target_id", -1))
        self.target = TargetLock(initial_id if initial_id >= 0 else None, auto=self.auto_target)

        # --- models (same construction as urc_gesture_test.main) ---
        import cv2
        from ultralytics import YOLO
        self.cv2 = cv2
        model_name = P("model", "yolo11n-pose.pt")
        self.pose_model = YOLO(model_name, task="pose")
        # Keep crop prediction separate from tracking so tracker state advances once per frame.
        if crop_pose:
            self.detector = YOLO(P("detector", "yolo11n.pt"))
        elif long_range:
            self.detector = YOLO(model_name, task="pose")
        else:
            self.detector = self.pose_model
        self.opts = dict(conf=float(P("conf", 0.3)), verbose=False,
                         quantize=16 if P("half", False) else 32)
        device = P("device", "0")
        if device:
            self.opts["device"] = device

        # --- state ---
        self.enabled = True
        self.session_id = uuid.uuid4().hex
        self.sequence = 0
        self.frame_index = 0
        self.last_ts = None
        self.previous_start = None
        self.fps = 0.0
        self._pose_source = "none"

        # --- ROS I/O ---
        # Sensor QoS (best-effort, depth 1): connects to reliable or best-effort publishers and
        # always hands us the newest frame instead of queueing a backlog behind slow inference.
        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(Int32, "~/select_target", self.on_select, 10)
        self.create_subscription(Bool, "~/enable", self.on_enable, 10)
        self.intent_pub = self.create_publisher(String, P("intent_topic", "/gesture/intent"), 10)
        self.status_pub = self.create_publisher(String, P("status_topic", "/gesture/status"), 10)
        self.debug_pub = self.create_publisher(Image, P("debug_image_topic", "/gesture/debug_image"), 1)

        self.get_logger().info(
            f"Listening on {image_topic} | imgsz {self.imgsz} | long_range={long_range} "
            f"crop_pose={crop_pose} | start with arms down")

    # ------------------------------------------------------------------ control
    def on_select(self, msg):
        if msg.data < 0:
            self.target = TargetLock(None, auto=self.auto_target)
            self.get_logger().info("target cleared")
        else:
            self.target.select(int(msg.data))
            self.get_logger().info(f"target locked: {msg.data}")
        self.deb.reset()

    def on_enable(self, msg):
        self.enabled = bool(msg.data)
        self.deb.reset()
        self.get_logger().info(f"gesture detection {'enabled' if self.enabled else 'paused'}")

    # ------------------------------------------------------------------ pipeline
    def on_image(self, msg):
        if not self.enabled:
            return
        start = time.monotonic()

        stamp = msg.header.stamp
        ts = stamp.sec + stamp.nanosec * 1e-9
        if ts <= 0:
            ts = self.get_clock().now().nanoseconds * 1e-9
        if self.last_ts is not None and ts <= self.last_ts:
            if self.last_ts - ts > 1.0:  # clock jumped back (bag loop / sim reset)
                self.deb.reset()
                self.target = TargetLock(None, auto=self.auto_target)
            else:
                return  # duplicate / out-of-order frame; debouncer needs increasing time
        self.last_ts = ts

        try:
            frame = image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return

        if self.frame_index == 0:
            self.get_logger().info(f"Actual source frames: {frame.shape[1]}x{frame.shape[0]} ({msg.encoding})")

        result = self.detector.track(frame, persist=True, tracker=self.tracker, classes=[0],
                                     imgsz=self.imgsz, **self.opts)[0]
        if self.frame_index == 0:
            self.get_logger().info(f"Inference device: {self.detector.predictor.device}")

        has_boxes = result.boxes is not None
        boxes = result.boxes.xyxy.cpu().numpy() if has_boxes else np.empty((0, 4))
        ids = (result.boxes.id.cpu().numpy().astype(int)
               if has_boxes and result.boxes.id is not None else np.empty(0, dtype=int))

        old_id = self.target.target_id
        idx, target_status = self.target.update(ids, boxes, ts)
        if old_id != self.target.target_id or self.target.blocked:
            self.deb.reset()

        raw, detail = "UNKNOWN", {"reason": target_status}
        selected_kp = None
        pose_source = "none"
        if idx is not None:
            selected_kp, selected_cf = self._target_pose(frame, result, boxes, idx)
            if selected_kp is not None:
                pose_source = self._pose_source
                raw, detail = classify(selected_kp, selected_cf, self.geometry)
                if raw == "NONE" and self.deb.candidate in COMMANDS:
                    loose, loose_detail = classify(selected_kp, selected_cf, self.geometry, relaxed=True)
                    if loose == self.deb.candidate:
                        raw, detail = loose, loose_detail
                h, w = frame.shape[:2]
                joints = selected_kp[self.required_joints]
                if raw != "UNKNOWN" and (np.any(joints <= 2) or np.any(joints >= [w - 2, h - 2])):
                    raw, detail = "UNKNOWN", {
                        "reason": "Required joints at image edge",
                        "hint": "Step back: keep both shoulders, elbows and wrists inside the picture."
                                + (" Include hips in this mode." if self.geometry.require_hips else "")}
            else:
                detail = {"reason": "missing/ambiguous pose or confidence"}
        detail.setdefault("body_frame", "shoulders + hips" if self.geometry.require_hips else
                          "shoulders (arms only)" if not self.geometry.camera_upright else
                          "upright camera (arms only)")
        detail["pose_source"] = pose_source

        try:
            confirmed, progress, fired = self.deb.update(raw, ts)
        except ValueError as exc:  # should not happen after the guard above
            self.get_logger().warn(f"debouncer reset: {exc}")
            self.deb.reset()
            confirmed, progress, fired = "NONE", 0.0, False

        if fired:
            self.sequence += 1
            send_command(confirmed, {"session_id": self.session_id, "sequence": self.sequence,
                                     "track_id": int(self.target.target_id), "source_time": ts,
                                     "frame_id": msg.header.frame_id}, self.intent_pub)

        # FPS over consecutive callback starts (includes inference, publishing, drawing)
        fps_now = 1 / max(start - self.previous_start, 1e-6) if self.previous_start is not None else 0
        self.previous_start = start
        self.fps = fps_now if self.fps == 0 else 0.9 * self.fps + 0.1 * fps_now
        processing_ms = (time.monotonic() - start) * 1000

        self.status_pub.publish(String(data=json.dumps({
            "source_time": ts, "raw": raw, "last_command": confirmed, "armed": self.deb.armed,
            "progress": round(progress, 3), "track_id": self.target.target_id,
            "target_status": target_status, "reason": detail.get("reason"),
            "hint": detail.get("hint"), "pose_source": pose_source,
            "loop_fps": round(self.fps, 2), "processing_ms": round(processing_ms, 1)},
            default=str)))

        if self.publish_debug and self.debug_pub.get_subscription_count() > 0:
            rows = status_rows(raw, confirmed, progress, "GESTURE", self.target.target_id,
                               self.fps, self.deb.armed, detail, target_status)
            self._publish_debug(msg.header, frame, boxes, ids, idx, selected_kp, rows)

        self.frame_index += 1

    def _target_pose(self, frame, result, boxes, idx):
        """Full-frame pose from the tracker, optionally replaced by a better target-crop pose."""
        kp = cf = None
        self._pose_source = "none"
        kps, confs = pose_arrays(result)
        if kps is not None:
            kp, cf = kps[idx], confs[idx]
            self._pose_source = "full frame"
        if self.refine_pose:
            x1, y1, x2, y2 = crop_bounds(boxes[idx], frame.shape)
            crop_result = self.pose_model.predict(frame[y1:y2, x1:x2], imgsz=self.pose_imgsz,
                                                  classes=[0], **self.opts)[0]
            kps, confs = pose_arrays(crop_result)
            if kps is not None and crop_result.boxes is not None:
                mapped = crop_result.boxes.xyxy.cpu().numpy() + [x1, y1, x1, y1]
                matched = match_crop_pose(mapped, boxes[idx])
                if matched is not None:
                    crop_kp, crop_cf = kps[matched] + [x1, y1], confs[matched]
                    if kp is None or (pose_quality(crop_kp, crop_cf, self.required_joints) >
                                      pose_quality(kp, cf, self.required_joints)):
                        kp, cf = crop_kp, crop_cf
                        self._pose_source = "target crop"
        return kp, cf

    def _publish_debug(self, header, frame, boxes, ids, idx, kp, rows):
        cv2 = self.cv2
        out = frame.copy()
        for i, (box, tid) in enumerate(zip(boxes, ids)):
            color = (0, 255, 0) if i == idx else (200, 200, 200)
            x1, y1, x2, y2 = (int(v) for v in box)
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
            cv2.putText(out, f"id {tid}", (x1, max(y1 - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        if kp is not None:
            for joint in self.required_joints:
                x, y = kp[joint]
                if math.isfinite(x) and math.isfinite(y) and (x or y):
                    cv2.circle(out, (int(x), int(y)), 5, (0, 0, 255), -1)
        out = draw_status_panel(out, rows)
        if 0 < self.debug_scale < 1:
            out = cv2.resize(out, None, fx=self.debug_scale, fy=self.debug_scale,
                             interpolation=cv2.INTER_AREA)
        self.debug_pub.publish(fill_bgr_image(Image(), out, header))


def main(args=None):
    rclpy.init(args=args)
    node = ZedGestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""Exercise real ROS images and the callback; replace model output deterministically."""
import json
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32

from zed_gesture.image_utils import fill_bgr_image, image_to_bgr
from zed_gesture.urc_gesture_test import (
    L_SH, R_SH, L_EL, R_EL, L_WR, R_WR, REQUIRED, self_test,
)
from zed_gesture.zed_gesture_node import ZedGestureNode


class Tensor:
    def __init__(self, values):
        self.values = np.asarray(values)

    def cpu(self):
        return self

    def numpy(self):
        return self.values


def pose(label):
    """Original photo-traced FOLLOW/STOP plus neutral and sideways arm poses."""
    points = np.zeros((17, 2))
    if label in ("FOLLOW", "STOP"):
        points[REQUIRED] = {
            "FOLLOW": [[262, 326], [125, 384], [314, 242], [110, 494], [337, 120], [109, 639]],
            "STOP": [[263, 373], [110, 376], [288, 227], [89, 239], [279, 77], [123, 97]],
        }[label]
    else:
        points[[L_SH, R_SH]] = [[250, 300], [350, 300]]
        for sh, el, wr, sign in [(L_SH, L_EL, L_WR, -1), (R_SH, R_EL, R_WR, 1)]:
            outward = label == "COME" or (label == "FETCH" and sh == L_SH)
            delta = np.array([sign * 60, 0] if outward else [0, 60])
            points[el], points[wr] = points[sh] + delta, points[sh] + 2 * delta
    confidence = np.ones(17)
    if label == "UNKNOWN":
        confidence[R_WR] = 0.0
    return SimpleNamespace(
        boxes=SimpleNamespace(xyxy=Tensor([[50, 50, 600, 700]]), id=Tensor([7])),
        keypoints=SimpleNamespace(xy=Tensor([points]), conf=Tensor([confidence])),
    )


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)

    def get_subscription_count(self):
        return 0

    def payloads(self):
        return [json.loads(msg.data) for msg in self.messages]


@pytest.fixture
def pipeline(monkeypatch, request):
    model = SimpleNamespace(result=pose("NEUTRAL"), predictor=SimpleNamespace(device="cpu"))
    ultralytics = ModuleType("ultralytics")
    ultralytics.YOLO = lambda *args, **kwargs: model
    cfg = ModuleType("ultralytics.cfg")
    cfg.DEFAULT_CFG_DICT = {"quantize": None}
    monkeypatch.setitem(sys.modules, "ultralytics", ultralytics)
    monkeypatch.setitem(sys.modules, "ultralytics.cfg", cfg)
    rclpy.init(args=[])
    request.addfinalizer(rclpy.shutdown)
    node = ZedGestureNode(parameter_overrides=[
        Parameter("device", value="cpu"),
    ])
    request.addfinalizer(node.destroy_node)
    node.intent_pub, node.status_pub, node.debug_pub = Publisher(), Publisher(), Publisher()
    clock = SimpleNamespace(ns=10_000_000_000)
    monkeypatch.setattr(node, "get_clock", lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=clock.ns)))
    model.advance_clock = lambda seconds: setattr(clock, "ns", clock.ns + round(seconds * 1e9))
    model.track_calls = model.crop_calls = 0

    def track(*args, **kwargs):
        model.track_calls += 1
        model.advance_clock(model.track_delay)
        return [model.result]

    def predict(*args, **kwargs):
        model.crop_calls += 1
        model.advance_clock(model.crop_delay)
        return [SimpleNamespace(keypoints=None, boxes=None)]

    model.track, model.predict = track, predict
    blank = np.zeros((720, 640, 3), dtype=np.uint8).tobytes()

    def frame(label="NEUTRAL", advance=0.05, delay=0.0, track_id=7, invalid=False,
              track_delay=0.0, crop_delay=0.0):
        clock.ns += round(advance * 1e9)
        model.result = pose(label)
        model.track_delay, model.crop_delay = track_delay, crop_delay
        model.result.boxes.id = Tensor([track_id])
        msg = Image(height=720, width=640, encoding="bgr8", step=640 * 3, data=blank)
        timestamp = clock.ns - round(delay * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(timestamp, 1_000_000_000)
        msg.header.frame_id = "zed2_left_camera_optical_frame"
        if invalid:
            msg.step = 1
        node.on_image(msg)
        return msg

    def feed(label, count, **kwargs):
        for _ in range(count):
            frame(label, **kwargs)

    yield node, model, frame, feed


def test_original_pose_regressions():
    self_test()


@pytest.mark.parametrize("label,command", [
    ("FOLLOW", "FOLLOW"), ("STOP", "STAY"), ("COME", "COME"), ("FETCH", "FETCH"),
])
def test_live_callback_holds_once_then_keeps_previous_command(pipeline, label, command):
    node, _, _, feed = pipeline
    feed("NEUTRAL", 12)
    feed(label, 6)
    assert not node.intent_pub.messages  # even STOP still needs its full hold
    feed(label, 40)
    events = node.intent_pub.payloads()
    assert len(events) == 1
    assert events[0]["command"] == command
    assert events[0]["track_id"] == 7
    assert events[0]["frame_id"] == "zed2_left_camera_optical_frame"
    assert events[0]["sequence"] == 1
    if label == "FETCH":
        assert events[0]["range_mode"] == "long"
    feed("NEUTRAL", 12)
    feed("UNKNOWN", 14)
    assert node.status_pub.payloads()[-1]["last_command"] == label
    assert len(node.intent_pub.messages) == 1


def test_missing_pose_breaks_hold_and_never_reuses_keypoints(pipeline):
    node, _, _, feed = pipeline
    feed("NEUTRAL", 12)
    feed("FOLLOW", 9)
    feed("UNKNOWN", 1)
    feed("FOLLOW", 9)
    assert not node.intent_pub.messages
    feed("FOLLOW", 10)
    assert [event["command"] for event in node.intent_pub.payloads()] == ["FOLLOW"]


@pytest.mark.parametrize("bad_frame", [
    {"delay": 1.0}, {"delay": -1.0}, {"invalid": True}, {"advance": 0.5},
])
def test_stale_malformed_or_gapped_frames_disarm(pipeline, bad_frame):
    node, _, frame, feed = pipeline
    feed("NEUTRAL", 12)
    feed("FOLLOW", 9)
    frame("FOLLOW", **bad_frame)
    feed("FOLLOW", 30)
    assert not node.intent_pub.messages
    feed("NEUTRAL", 12)
    feed("FOLLOW", 20)
    assert len(node.intent_pub.messages) == 1


def test_duplicate_images_do_not_complete_hold(pipeline):
    node, _, frame, feed = pipeline
    feed("NEUTRAL", 12)
    frame("FOLLOW")
    feed("FOLLOW", 40, advance=0)
    assert not node.intent_pub.messages
    assert node.deb.samples == 1


def test_previous_command_survives_controls_loss_and_reselection(pipeline):
    node, _, frame, feed = pipeline
    feed("NEUTRAL", 12)
    feed("FOLLOW", 20)
    node.on_enable(Bool(data=False))
    feed("STOP", 20)
    assert len(node.intent_pub.messages) == 1
    node.on_enable(Bool(data=True))
    node.on_select(Int32(data=8))
    frame("NEUTRAL", track_id=8)
    assert node.status_pub.payloads()[-1]["last_command"] == "FOLLOW"
    feed("FOLLOW", 25, track_id=7)  # previous person cannot command the new lock
    assert node.target.blocked
    assert node.status_pub.payloads()[-1]["last_command"] == "FOLLOW"
    node.on_select(Int32(data=-1))
    feed("COME", 20)
    assert len(node.intent_pub.messages) == 1  # changing targets never rearms
    feed("NEUTRAL", 12)
    feed("COME", 20)
    assert [event["command"] for event in node.intent_pub.payloads()] == ["FOLLOW", "COME"]
    assert node.status_pub.payloads()[-1]["last_command"] == "COME"


def test_default_workload_uses_one_pose_pass(pipeline):
    node, model, _, feed = pipeline
    feed("NEUTRAL", 12)
    feed("FOLLOW", 20)
    assert node.imgsz == 640
    assert model.track_calls == 32
    assert model.crop_calls == 0
    assert len(node.intent_pub.messages) == 1


@pytest.mark.parametrize("interval", [0.05, 0.2])
def test_holds_and_priority_stop_at_supported_cadences(pipeline, interval):
    node, _, _, feed = pipeline
    feed("NEUTRAL", 12, advance=interval)
    feed("FOLLOW", 20, advance=interval)
    feed("STOP", 20, advance=interval)
    assert [event["command"] for event in node.intent_pub.payloads()] == ["FOLLOW", "STAY"]
    assert node.status_pub.payloads()[-1]["timing_ok"]


def test_slow_processing_is_reported_and_does_not_complete_holds(pipeline):
    node, _, _, feed = pipeline
    feed("NEUTRAL", 12)
    feed("FOLLOW", 20)
    feed("STOP", 15, advance=0.3)
    status = node.status_pub.payloads()[-1]
    assert not status["timing_ok"]
    assert status["frame_gap_ms"] == 300.0
    assert status["last_command"] == "FOLLOW"
    assert len(node.intent_pub.messages) == 1


@pytest.mark.parametrize("stage", ["track", "crop"])
def test_image_expiring_during_inference_breaks_hold_and_preserves_history(pipeline, stage):
    node, _, frame, feed = pipeline
    feed("NEUTRAL", 12)
    feed("FOLLOW", 20)
    feed("NEUTRAL", 12)
    feed("COME", 9)
    node.refine_pose = stage == "crop"
    frame("COME", **{f"{stage}_delay": 0.6})
    status = node.status_pub.payloads()[-1]
    assert status["raw"] == "UNKNOWN"
    assert status["last_command"] == "FOLLOW"
    assert status["frame_age_ms"] == 600.0
    assert not status["timing_ok"]
    assert "expired" in status["reason"]
    assert not node.deb.armed
    feed("COME", 30)
    assert len(node.intent_pub.messages) == 1
    feed("NEUTRAL", 12)
    feed("COME", 20)
    assert [event["command"] for event in node.intent_pub.payloads()] == ["FOLLOW", "COME"]


def test_last_moment_expiry_cannot_publish_or_replace_previous_command(pipeline, monkeypatch):
    node, model, frame, feed = pipeline
    feed("NEUTRAL", 12)
    feed("FOLLOW", 20)
    previous_fire = node.deb.last_fire

    def confirm_then_delay(raw, ts):
        assert raw == "COME"
        node.deb.last_fire = ts
        model.advance_clock(0.1)
        return "COME", 1.0, True

    monkeypatch.setattr(node.deb, "update", confirm_then_delay)
    frame("COME", track_delay=0.45)
    status = node.status_pub.payloads()[-1]
    assert not status["timing_ok"]
    assert status["last_command"] == "FOLLOW"
    assert node.sequence == 1
    assert node.deb.last_fire == previous_fire
    assert len(node.intent_pub.messages) == 1


@pytest.mark.parametrize("encoding,pixel", [
    ("bgr8", [1, 2, 3]), ("bgra8", [1, 2, 3, 255]),
    ("rgb8", [3, 2, 1]), ("rgba8", [3, 2, 1, 255]), ("mono8", [9]),
])
def test_camera_encodings_with_row_padding(encoding, pixel):
    row = pixel + pixel + [0, 0]
    msg = Image(height=2, width=2, encoding=encoding, step=len(row), data=bytes(row * 2))
    frame = image_to_bgr(msg)
    assert frame.shape == (2, 2, 3)
    assert frame.flags.c_contiguous
    np.testing.assert_array_equal(frame[0, 0], [9, 9, 9] if encoding == "mono8" else [1, 2, 3])
    restored = fill_bgr_image(Image(), frame, msg.header)
    np.testing.assert_array_equal(image_to_bgr(restored), frame)

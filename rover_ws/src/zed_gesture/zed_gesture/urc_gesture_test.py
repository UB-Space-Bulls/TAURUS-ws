#!/usr/bin/env python3
"""URC arm commands: YOLO COCO pose -> geometry -> temporal filter -> intent.

pip install torch==2.14.1 torchvision==0.29.1 --index-url https://download.pytorch.org/whl/cu130
pip install ultralytics opencv-python numpy
python urc_gesture_test.py
python urc_gesture_test.py --device 0 --half
python urc_gesture_test.py --video test_20m.mp4 --log results.jsonl
python urc_gesture_test.py --crop-pose --imgsz 960 --pose-imgsz 640
python urc_gesture_test.py --self-test

FOLLOW: one arm raised, opposite arm resting down. STOP/STAY: both arms raised.
Raised arms may angle outward and bend slightly; exact vertical alignment is unnecessary.
COME: both straight outward. Start/rearm with both arms down for 0.4 s.
FETCH (long range): one arm straight sideways, either side, opposite arm down.
FETCH emits range_mode="long"; the rover controller owns the travel behavior.
Long-range vision defaults to 960px tracking plus a closer pose pass on the target.
Use --no-long-range for the lighter 640px single-pass mode.
Only shoulders, elbows and wrists are required; hips/legs can be out of frame.
The camera is assumed upright. Use --shoulder-frame for a tilted camera.
Use --require-hips for the stricter full-torso reference frame.
Click a person's box to lock their track; r clears it, g/v select gesture/voice
input mode, q quits. A lone person is selected automatically at startup unless
--manual-target is set. Target loss/overlap requires explicit reselection.
VOICE is an external-input handoff indicator, not a speech recognizer.
Optional --ros-topic publishes JSON intents as std_msgs/String (source ROS first).
"""

import argparse
from dataclasses import dataclass
import json
import math
import time
import uuid

import numpy as np

NOSE = 0
L_SH, R_SH, L_EL, R_EL, L_WR, R_WR = 5, 6, 7, 8, 9, 10
L_HIP, R_HIP = 11, 12
REQUIRED = [L_SH, R_SH, L_EL, R_EL, L_WR, R_WR]
JOINT_NAMES = {L_SH: "L shoulder", R_SH: "R shoulder", L_EL: "L elbow",
               R_EL: "R elbow", L_WR: "L wrist", R_WR: "R wrist",
               L_HIP: "L hip", R_HIP: "R hip"}
COMMANDS = {"FOLLOW": "FOLLOW", "STOP": "STAY", "COME": "COME", "FETCH": "FETCH"}


@dataclass
class GeometryConfig:
    kp_conf: float = 0.5
    min_torso_px: float = 12.0
    min_shoulder_ratio: float = 0.45
    straight_deg: float = 155.0
    up_deg: float = 45.0
    up_straight_deg: float = 135.0
    out_deg: float = 20.0
    down_deg: float = 55.0
    require_hips: bool = False
    camera_upright: bool = True


def angle_deg(a, b, c):
    ba, bc = a - b, c - b
    denom = np.linalg.norm(ba) * np.linalg.norm(bc)
    if denom < 1e-6:
        return 0.0
    return math.degrees(math.acos(float(np.clip(np.dot(ba, bc) / denom, -1, 1))))


def classify(kp, cf, config=None, relaxed=False):
    """Return FOLLOW/STOP/COME/FETCH/NEUTRAL/NONE/UNKNOWN and diagnostic geometry.

    UNKNOWN means untrusted geometry; NONE means visible but not a command.
    Neither is evidence of arms-down release. Default directions use the upright
    image axes so lifting a shoulder does not rotate the gesture reference.
    --shoulder-frame and --require-hips select body-relative reference frames.
    """
    cfg = config or GeometryConfig()
    kp, cf = np.asarray(kp, dtype=float), np.asarray(cf, dtype=float)
    if kp.shape != (17, 2) or cf.shape != (17,):
        return "UNKNOWN", {"reason": "expected COCO 17 keypoints with confidence"}
    required = REQUIRED + [L_HIP, R_HIP] if cfg.require_hips else REQUIRED
    missing = [joint for joint in required if
               not np.isfinite(kp[joint]).all() or not np.isfinite(cf[joint]) or
               cf[joint] < cfg.kp_conf or np.all(kp[joint] == 0)]
    if missing:
        names = [JOINT_NAMES[joint] for joint in missing]
        summary = ", ".join(names[:3]) + (", ..." if len(names) > 3 else "")
        return "UNKNOWN", {"reason": f"Weak/missing: {summary}", "missing_joints": names,
                           "hint": "Keep both shoulders, elbows and wrists in view."
                                   + (" Hips are required in this mode." if cfg.require_hips else " Hips are optional.")}

    shoulders = (kp[L_SH] + kp[R_SH]) / 2
    shoulder_w = float(np.linalg.norm(kp[L_SH] - kp[R_SH]))
    # Use shoulder width only as a plausibility/size scale, never as a required
    # wrist reach. Arm directions and extension use each arm's own segment lengths.
    torso = shoulder_w / 0.8
    body_frame = "shoulders (arms only)"
    if cfg.require_hips:
        hips = (kp[L_HIP] + kp[R_HIP]) / 2
        torso = float(np.linalg.norm(hips - shoulders))
        body_frame = "shoulders + hips"
    if torso < cfg.min_torso_px:
        return "UNKNOWN", {"reason": "body reference too small", "reference_px": torso}
    if cfg.require_hips:
        if not cfg.min_shoulder_ratio <= shoulder_w / torso <= 2.0:
            return "UNKNOWN", {"reason": "profile/implausible torso", "torso_px": torso}
        down = (hips - shoulders) / torso
        across = np.array([down[1], -down[0]])
        if abs(float(np.dot((kp[L_SH] - kp[R_SH]) / shoulder_w, down))) > 0.45:
            return "UNKNOWN", {"reason": "distorted torso"}
    else:
        across = (kp[R_SH] - kp[L_SH]) / shoulder_w
        if across[0] < 0:
            across = -across
        if across[0] < 0.3:
            return "UNKNOWN", {"reason": "shoulder line too steep; face the camera upright"}
        down = np.array([-across[1], across[0]])
        if cfg.camera_upright:
            # A raised arm naturally raises its shoulder. Treating that slope
            # as camera roll makes FOLLOW unnecessarily close to the UP limit.
            across, down = np.array([1.0, 0.0]), np.array([0.0, 1.0])
            body_frame = "upright camera (arms only)"

    margin = 5.0 if relaxed else 0.0
    extension = 0.6 if relaxed else 0.7
    arms = {}
    nose_valid = (cfg.require_hips and np.isfinite(kp[NOSE]).all() and np.isfinite(cf[NOSE]) and
                  cf[NOSE] >= cfg.kp_conf and not np.all(kp[NOSE] == 0))
    for side, (sh, el, wr) in {"L": (L_SH, L_EL, L_WR),
                               "R": (R_SH, R_EL, R_WR)}.items():
        arm = (kp[wr] - kp[sh]) / torso
        upper = (kp[el] - kp[sh]) / torso
        fore = (kp[wr] - kp[el]) / torso
        length = float(np.linalg.norm(arm))
        upper_length, fore_length = float(np.linalg.norm(upper)), float(np.linalg.norm(fore))
        reach = upper_length + fore_length
        # Reject degenerate and implausibly long projected limbs.
        if (not 0.15 <= upper_length <= 1.8 or not 0.15 <= fore_length <= 1.8 or
                not 0.25 <= upper_length / fore_length <= 4 or length < 1e-6):
            return "UNKNOWN", {"reason": f"Implausible {side} arm geometry",
                               "hint": "Keep the whole arm visible and face the camera."}
        vertical = float(np.dot(arm, down))
        outward_sign = 1 if np.dot(kp[sh] - shoulders, across) > 0 else -1
        outward = float(np.dot(arm, across) * outward_sign)
        elbow = angle_deg(kp[sh], kp[el], kp[wr])
        straight = elbow >= cfg.straight_deg - margin
        above_head = vertical < -(0.45 if relaxed else 0.5) * reach
        if nose_valid:
            above_head = above_head and np.dot(kp[wr] - kp[NOSE], down) < -0.12 * torso
        up_tilt = math.degrees(math.acos(float(np.clip(-vertical / length, -1, 1))))
        up_checks = {
            "raise wrist higher": above_head,
            "open elbow a little": elbow >= cfg.up_straight_deg - margin,
            "point arm further upward": up_tilt <= cfg.up_deg + margin,
            "raise elbow above shoulder": np.dot(upper, down) < -0.05 * reach,
            "raise wrist above elbow": np.dot(fore, down) < -0.05 * reach}
        up = all(up_checks.values())
        out = (straight and outward > extension * reach and
               outward / length >= math.cos(math.radians(cfg.out_deg + margin)) and
               np.dot(upper, across) * outward_sign > 0.1 * reach and
               np.dot(fore, across) * outward_sign > 0.1 * reach)
        # A resting arm may angle away from the body or bend at the elbow.
        # Still require the wrist well below its shoulder and the elbow below
        # its shoulder; a forearm pointing back up is not arms-down release.
        down_tilt = math.degrees(math.acos(float(np.clip(vertical / length, -1, 1))))
        down_checks = {
            "wrist below shoulder": vertical > 0.45 * reach,
            "elbow below shoulder": np.dot(upper, down) > 0.1 * reach,
            "forearm not pointing up": np.dot(fore, down) >= -0.05 * reach,
            "arm near downward": down_tilt <= cfg.down_deg + margin}
        lowered = all(down_checks.values())
        arms[side] = {"up": bool(up), "out": bool(out), "down": bool(lowered),
                      "state": "UP" if up else "OUT" if out else "DOWN" if lowered else "OTHER",
                      "elbow_deg": round(elbow, 1), "down_tilt_deg": round(down_tilt, 1),
                      "up_tilt_deg": round(up_tilt, 1),
                      "up_rejected": [name for name, passed in up_checks.items() if not passed],
                      "down_rejected": [name for name, passed in down_checks.items() if not passed]}

    left, right = arms["L"], arms["R"]
    if left["up"] and right["up"]:
        label = "STOP"
    elif (left["up"] and right["down"]) or (right["up"] and left["down"]):
        label = "FOLLOW"
    elif left["out"] and right["out"]:
        label = "COME"
    elif (left["out"] and right["down"]) or (right["out"] and left["down"]):
        label = "FETCH"
    elif left["down"] and right["down"]:
        label = "NEUTRAL"
    else:
        label = "NONE"
    detail = {"reason": "ok" if label != "NONE" else "Arms visible; no matching gesture",
              "reference_px": round(torso, 1), "body_frame": body_frame, "arms": arms}
    if label == "NONE":
        raised = [side for side in arms if arms[side]["up"]]
        if len(raised) == 1:
            other = "R" if raised[0] == "L" else "L"
            detail["reason"] = f"FOLLOW blocked: {other} arm not down"
            detail["hint"] = f"{other} arm: " + "; ".join(arms[other]["down_rejected"])
        else:
            sideways = [side for side in arms if arms[side]["out"]]
            near_up = [side for side, arm in arms.items() if arm["up_tilt_deg"] < 75]
            if len(sideways) == 1:
                other = "R" if sideways[0] == "L" else "L"
                detail["reason"] = f"FETCH blocked: {other} arm not down"
                detail["hint"] = f"{other} arm: " + "; ".join(arms[other]["down_rejected"])
            elif near_up:
                detail["hint"] = " | ".join(
                    f"{side}: " + "; ".join(arms[side]["up_rejected"]) for side in near_up)
            else:
                detail["hint"] = "FETCH: one arm sideways, other down | COME: both sideways."
    return label, detail


class GestureDebouncer:
    """Neutral release, continuous holds, cooldown and priority STOP.

    Pass capture/source time, never inference elapsed time for recorded clips.
    Missing observations break holds. Long gaps also disarm motion commands.
    """
    def __init__(self, hold_sec=0.7, release_sec=0.4, cooldown_sec=1.2,
                 stop_sec=0.35, max_gap_sec=0.25, loss_sec=0.4):
        self.hold_sec, self.release_sec = hold_sec, release_sec
        self.cooldown_sec, self.stop_sec = cooldown_sec, stop_sec
        self.max_gap_sec, self.loss_sec = max_gap_sec, loss_sec
        self.last_fire = -math.inf
        self.reset()

    def reset(self):
        self.armed = False
        self.stop_latched = False
        self.candidate = "NONE"
        self.cand_since = None
        self.neutral_since = None
        self.unknown_since = None
        self.last_time = None
        self.samples = 0
        self.confirmed = "NONE"

    def update(self, raw, now=None):
        now = time.monotonic() if now is None else float(now)
        if not math.isfinite(now):
            raise ValueError("timestamp must be finite")
        if self.last_time is not None:
            if now <= self.last_time:
                raise ValueError("timestamps must increase")
            if now - self.last_time > self.max_gap_sec:
                self.armed = False
                self.candidate, self.cand_since = "NONE", None
                self.neutral_since = None
        self.last_time = now

        if raw == "UNKNOWN":
            if self.unknown_since is None:
                self.unknown_since = now
            if now - self.unknown_since >= self.loss_sec:
                self.armed = False
        else:
            self.unknown_since = None
        if raw == "NEUTRAL":
            if self.neutral_since is None:
                self.neutral_since = now
            if now - self.neutral_since >= self.release_sec:
                self.armed = True
                self.stop_latched = False
        else:
            self.neutral_since = None

        if raw not in COMMANDS:
            self.candidate, self.cand_since, self.samples = "NONE", None, 0
            return self.confirmed, 0.0, False
        if raw != self.candidate or self.cand_since is None:
            self.candidate, self.cand_since, self.samples = raw, now, 1
        else:
            self.samples += 1
        required = self.stop_sec if raw == "STOP" else self.hold_sec
        progress = min((now - self.cand_since) / required, 1.0)
        allowed = ((raw == "STOP" and not self.stop_latched) or
                   (raw != "STOP" and self.armed and
                    now - self.last_fire >= self.cooldown_sec))
        fired = allowed and progress >= 1.0 and self.samples >= 3
        if fired:
            self.confirmed, self.last_fire, self.armed = raw, now, False
            if raw == "STOP":
                self.stop_latched = True
        return self.confirmed, progress, bool(fired)


def box_iou(a, b):
    lo, hi = np.maximum(a[:2], b[:2]), np.minimum(a[2:], b[2:])
    intersection = float(np.prod(np.maximum(hi - lo, 0)))
    union = float(np.prod(np.maximum(a[2:] - a[:2], 0)) +
                  np.prod(np.maximum(b[2:] - b[:2], 0)) - intersection)
    return intersection / max(union, 1e-6)


class TargetLock:
    def __init__(self, target_id=None, auto=True, loss_sec=0.6):
        self.target_id, self.auto, self.loss_sec = target_id, auto, loss_sec
        self.last_seen = None
        self.missing = False
        self.blocked = False

    def select(self, target_id):
        self.target_id, self.last_seen, self.blocked = target_id, None, False
        self.missing = False

    def update(self, ids, boxes, now):
        if self.blocked:
            return None, "reselect target"
        if self.target_id is None and self.auto and len(ids) == 1:
            self.target_id = int(ids[0])
        matches = np.flatnonzero(ids == self.target_id) if self.target_id is not None else []
        if len(matches) != 1:
            self.missing = True
            if self.last_seen is not None and now - self.last_seen > self.loss_sec:
                self.blocked = True
            return None, "target missing / select astronaut"
        idx = int(matches[0])
        # Slow inference alone is not evidence that the tracked person vanished.
        if self.missing and self.last_seen is not None and now - self.last_seen > self.loss_sec:
            self.blocked = True
            return None, "target lost; reselect"
        if any(box_iou(boxes[idx], box) > 0.25 for j, box in enumerate(boxes) if j != idx):
            self.blocked = True
            return None, "overlapping people; reselect"
        self.last_seen = now
        self.missing = False
        return idx, "locked"


class MediaClock:
    def __init__(self, fps):
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("video FPS missing; use --video-fps")
        self.fps, self.previous = fps, None

    def update(self, position_ms):
        timestamp = position_ms / 1000.0
        if not math.isfinite(timestamp) or timestamp < 0:
            timestamp = 0.0 if self.previous is None else self.previous + 1 / self.fps
        if self.previous is not None and timestamp <= self.previous:
            timestamp = self.previous + 1 / self.fps
        self.previous = timestamp
        return timestamp


def crop_bounds(box, shape, padding=0.4):
    x1, y1, x2, y2 = box
    dx, dy = (x2 - x1) * padding, (y2 - y1) * padding
    h, w = shape[:2]
    return (max(0, int(x1 - dx)), max(0, int(y1 - dy)),
            min(w, int(math.ceil(x2 + dx))), min(h, int(math.ceil(y2 + dy))))


def match_crop_pose(boxes, target_box):
    scores = np.array([box_iou(box, target_box) for box in boxes])
    if len(scores) == 0 or scores.max() < 0.3:
        return None
    order = np.argsort(scores)[::-1]
    if len(order) > 1 and scores[order[0]] - scores[order[1]] < 0.15:
        return None
    return int(order[0])


def pose_arrays(result):
    if result.keypoints is None or result.keypoints.conf is None:
        return None, None
    return result.keypoints.xy.cpu().numpy(), result.keypoints.conf.cpu().numpy()


def pose_quality(kp, cf, required):
    """Prefer refinement only when it improves confidence in required joints."""
    scores = np.asarray(cf[required], dtype=float).copy()
    valid = (np.isfinite(kp[required]).all(axis=1) & np.isfinite(scores) &
             ~np.all(kp[required] == 0, axis=1))
    scores[~valid] = 0
    return float(scores.min()), float(scores.mean())


def status_rows(raw, confirmed, progress, mode, target_id, fps, armed, detail, target_status):
    arms = detail.get("arms", {})
    arm_text = " | ".join(
        f"{side} arm: {arm['state']} (elbow {arm['elbow_deg']:.0f} deg"
        + (f", down tilt {arm['down_tilt_deg']:.0f} deg" if arm['state'] == "OTHER" else "") + ")"
        for side, arm in arms.items())
    hint = detail.get("hint", "FOLLOW: one up | STAY: both up | FETCH: one sideways | COME: both sideways")
    if mode == "GESTURE" and raw in COMMANDS and raw != "STOP" and not armed:
        hint = "Gesture recognized. Lower both arms briefly to rearm, then repeat it."
    current = "FETCH (LONG RANGE)" if raw == "FETCH" else raw
    return [f"GESTURE NOW: {current} | last command: {confirmed} | hold {progress:.0%}",
            f"MODE {mode} | target {target_id} | {fps:.1f} FPS | {target_status}",
            f"{'ARMED' if armed else 'NEED ARMS DOWN'} | {detail['reason']}",
            f"{detail.get('body_frame', 'Arms')}: " + (arm_text or
                "Keep both wrists, elbows and shoulders visible."),
            hint]


def draw_status_panel(frame, rows):
    """Append feedback below the image so it never covers the hands or pose."""
    import cv2

    height, width = frame.shape[:2]
    panel = cv2.copyMakeBorder(frame, 0, 32 * len(rows) + 12, 0, 0,
                               cv2.BORDER_CONSTANT, value=(0, 0, 0))
    for row, text in enumerate(rows):
        text_width = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0][0]
        scale = 0.7 * min(1.0, max(width - 20, 1) / max(text_width, 1))
        color = (0, 255, 255) if row == 0 else (255, 255, 255)
        cv2.putText(panel, text, (10, height + 26 + 32 * row),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2)
    return panel


def send_command(label, event=None, publisher=None):
    """Only accepted command intents reach this hook; supervisor owns movement."""
    payload = dict(event or {})
    payload.update(command=COMMANDS[label], label=label, source="gesture",
                   emitted_unix=time.time())
    if label == "FETCH":
        payload["range_mode"] = "long"
    message = json.dumps(payload, separators=(",", ":"), allow_nan=False)
    print(message, flush=True)
    if publisher is not None:
        from std_msgs.msg import String
        publisher.publish(String(data=message))
    return payload


def self_test():
    """Deterministic geometry, time, identity and crop regressions; no models."""
    kp = np.zeros((17, 2), dtype=float)
    kp[[NOSE, L_SH, R_SH, L_HIP, R_HIP]] = [
        [200, 70], [150, 110], [250, 110], [160, 210], [240, 210]]
    cf = np.ones(17)

    def pose(left, right):
        p = kp.copy()
        for sh, el, wr, state in [(L_SH, L_EL, L_WR, left), (R_SH, R_EL, R_WR, right)]:
            sign = -1 if sh == L_SH else 1
            delta = {"up": [0, -60], "down": [0, 60], "out": [sign * 60, 0]}[state]
            p[el] = p[sh] + delta
            p[wr] = p[sh] + np.array(delta) * 2
        return p

    count = 0

    def check(condition, name):
        nonlocal count
        if not condition:
            raise AssertionError(name)
        count += 1

    for left, right, expected in [("up", "down", "FOLLOW"), ("down", "up", "FOLLOW"),
                                   ("up", "up", "STOP"), ("out", "out", "COME"),
                                   ("out", "down", "FETCH"), ("down", "out", "FETCH"),
                                   ("down", "down", "NEUTRAL"), ("up", "out", "NONE")]:
        p = pose(left, right)
        for scale in [0.15, 1.0, 3.0]:
            for rotation in [-20, 0, 20]:
                theta = math.radians(rotation)
                rot = np.array([[math.cos(theta), -math.sin(theta)],
                                [math.sin(theta), math.cos(theta)]])
                check(classify(p @ rot.T * scale + 500, cf,
                               GeometryConfig(camera_upright=False))[0] == expected,
                      f"{expected}, scale {scale}, roll {rotation}")
    p = pose("up", "down")
    weak = cf.copy()
    weak[R_WR] = 0.1
    check(classify(p, weak)[0] == "UNKNOWN", "opposite wrist required")
    check(classify(p, np.full(17, np.nan))[0] == "UNKNOWN", "NaN confidence")
    check(classify(np.full((17, 2), np.nan), cf)[0] == "UNKNOWN", "NaN coordinates")
    check(classify(p * 0.05 + 500, cf)[0] == "UNKNOWN", "tiny torso")
    profile = p.copy()
    profile[:, 0] = 200 + (profile[:, 0] - 200) * 0.2
    check(classify(profile, cf)[0] == "UNKNOWN", "profile")
    hidden_nose = cf.copy()
    hidden_nose[NOSE] = 0
    check(classify(p, hidden_nose)[0] == "FOLLOW", "head fallback")
    bent = p.copy()
    bent[L_EL] += [65, 30]
    check(classify(bent, cf)[0] != "FOLLOW", "bent arm/head scratch")
    boundary = p.copy()
    tilt = math.radians(GeometryConfig().up_deg + 2)
    boundary[L_EL] = p[L_SH] + [60 * math.sin(tilt), -60 * math.cos(tilt)]
    boundary[L_WR] = p[L_SH] + [120 * math.sin(tilt), -120 * math.cos(tilt)]
    check(classify(boundary, cf)[0] == "NONE" and
          classify(boundary, cf, relaxed=True)[0] == "FOLLOW", "angular hysteresis")

    # Approximate visible shoulder/elbow/wrist positions traced from the user's
    # two example images. Confidence is assumed here to isolate geometry; this
    # does not test model accuracy or estimate confidence from the box label.
    examples = {
        "FOLLOW": [[262, 326], [125, 384], [314, 242], [110, 494], [337, 120], [109, 639]],
        "STOP": [[263, 373], [110, 376], [288, 227], [89, 239], [279, 77], [123, 97]]}
    example_conf = np.zeros(17)
    example_conf[REQUIRED] = 1
    example_rng = np.random.default_rng(11)
    for expected, points in examples.items():
        example = np.full((17, 2), np.nan)
        example[REQUIRED] = points
        for mirror in [-1, 1]:
            transformed = example * [mirror, 1] + [500, 0]
            check(classify(transformed, example_conf)[0] == expected,
                  f"user image geometry: {expected}, mirror {mirror}")
            observed = [classify(transformed + example_rng.normal(0, 3, transformed.shape), example_conf)[0]
                        for _ in range(30)]
            check(all(label == expected for label in observed),
                  f"user image geometry tolerates 3px joint jitter: {expected}, mirror {mirror}")
        check(classify(example, example_conf, GeometryConfig(camera_upright=False))[0] == expected,
              f"shoulder-relative mode remains available for {expected}")

    def raised_arm(p, side, tilt, bend):
        sh, el, wr = ((L_SH, L_EL, L_WR) if side == "L" else (R_SH, R_EL, R_WR))
        sign = -1 if side == "L" else 1
        upper_angle, fore_angle = math.radians(tilt + bend / 2), math.radians(tilt - bend / 2)
        p[el] = p[sh] + [sign * 60 * math.sin(upper_angle), -60 * math.cos(upper_angle)]
        p[wr] = p[el] + [sign * 60 * math.sin(fore_angle), -60 * math.cos(fore_angle)]
        return p

    for command in ["FOLLOW_L", "FOLLOW_R", "STOP"]:
        for tilt, bend in [(35, 30), (44, 40), (15, 40)]:
            natural = pose("down", "down")
            if command != "FOLLOW_R":
                raised_arm(natural, "L", tilt, bend)
            if command != "FOLLOW_L":
                raised_arm(natural, "R", max(0, tilt - 8), max(0, bend - 10))
            expected = "STOP" if command == "STOP" else "FOLLOW"
            for roll in [-20, 0, 20]:
                theta = math.radians(roll)
                rot = np.array([[math.cos(theta), -math.sin(theta)],
                                [math.sin(theta), math.cos(theta)]])
                for mirror in [-1, 1]:
                    transformed = (natural * [mirror, 1]) @ rot.T + 500
                    configs = [GeometryConfig(camera_upright=False), GeometryConfig(require_hips=True)]
                    if roll == 0:
                        configs.append(GeometryConfig())
                    for config in configs:
                        check(classify(transformed, cf, config)[0] == expected,
                              f"comfortable {command}, tilt {tilt}, bend {bend}, roll {roll}, mirror {mirror}, config {config}")
    for tilt, bend, name in [(65, 0, "too sideways"), (0, 75, "sharply folded"),
                              (30, 90, "head scratch")]:
        rejected = raised_arm(pose("down", "down"), "L", tilt, bend)
        check(classify(rejected, cf)[0] not in ("FOLLOW", "STOP"), f"raised arm rejects {name}")
        raised_arm(rejected, "R", tilt, bend)
        check(classify(rejected, cf)[0] not in ("FOLLOW", "STOP"), f"both raised arms reject {name}")
    bent_out = raised_arm(raised_arm(pose("down", "down"), "L", 90, 40), "R", 90, 40)
    check(classify(bent_out, cf)[0] == "NONE", "raised-arm elbow tolerance does not loosen COME")

    # Small keypoint noise should not require re-aligning a comfortable raised pose.
    rng = np.random.default_rng(7)
    for command in ["FOLLOW_L", "FOLLOW_R", "STOP"]:
        natural = pose("down", "down")
        if command != "FOLLOW_R":
            raised_arm(natural, "L", 35, 30)
        if command != "FOLLOW_L":
            raised_arm(natural, "R", 30, 35)
        replay, events = GestureDebouncer(), []
        for frame_no, sample in enumerate([pose("down", "down")] * 12 + [natural] * 30):
            observed, _ = classify(sample + rng.normal(0, 1, sample.shape), cf)
            accepted, _, fired = replay.update(observed, (frame_no + 1) * 0.05)
            if fired:
                events.append(accepted)
        check(events == ["STOP" if command == "STOP" else "FOLLOW"],
              f"comfortable {command} confirms once despite small keypoint noise")

    arms_conf = np.zeros(17)
    arms_conf[REQUIRED] = 1
    for left, right, expected in [("up", "down", "FOLLOW"), ("down", "up", "FOLLOW"),
                                   ("up", "up", "STOP"), ("out", "out", "COME"),
                                   ("out", "down", "FETCH"), ("down", "out", "FETCH"),
                                   ("down", "down", "NEUTRAL")]:
        for arm_scale in [0.6, 1.0, 1.5]:
            upper_body = pose(left, right)
            for sh, el, wr in [(L_SH, L_EL, L_WR), (R_SH, R_EL, R_WR)]:
                upper_body[[el, wr]] = upper_body[sh] + (
                    upper_body[[el, wr]] - upper_body[sh]) * arm_scale
            # Everything except the arm chains is absent, including face and hips.
            upper_body[arms_conf == 0] = np.nan
            for mirror in [1, -1]:
                theta = math.radians(15)
                rot = np.array([[math.cos(theta), -math.sin(theta)],
                                [math.sin(theta), math.cos(theta)]])
                transformed = (upper_body * [mirror, 1]) @ rot.T + 500
                label, detail = classify(transformed, arms_conf)
                check(label == expected,
                      f"arms only {expected}, arm scale {arm_scale}, mirror {mirror}, roll 15")
                check(detail.get("body_frame") == "upright camera (arms only)", "uses upright camera frame")
    upper_body = pose("up", "down")
    upper_body[[L_HIP, R_HIP]] = np.nan
    check(classify(upper_body, arms_conf, GeometryConfig(require_hips=True))[0] == "UNKNOWN",
          "full-torso mode still requires hips")
    missing_wrist = arms_conf.copy()
    missing_wrist[R_WR] = 0
    label, detail = classify(upper_body, missing_wrist)
    check(label == "UNKNOWN" and detail.get("missing_joints") == ["R wrist"],
          "missing wrist identified; absent hips do not veto arm-only poses")
    check("R wrist" in status_rows(label, "NONE", 0, "GESTURE", 1, 30, False,
                                   detail, "locked")[2], "missing wrist visible in feedback")
    check(classify(pose("up", "up"), cf, GeometryConfig(require_hips=True))[0] == "STOP",
          "full-torso mode recognizes STOP")
    unrelated_head = pose("up", "down")
    unrelated_head[NOSE] = [200, -500]
    check(classify(unrelated_head, cf)[0] == "FOLLOW",
          "arm-only classification is independent of face estimates")

    # FOLLOW regressions: real resting-arm shapes, not only vertical sticks.
    resting_shapes = {
        "diagonal 50 degrees": ([46, 39], [92, 78]),
        "bent beside waist": ([80, 60], [40, 70]),
        "short projected forearm": ([35, 50], [55, 65])}
    for lowered_side in ["L", "R"]:
        sh, el, wr = ((L_SH, L_EL, L_WR) if lowered_side == "L" else (R_SH, R_EL, R_WR))
        sign = -1 if lowered_side == "L" else 1
        for name, (elbow, wrist) in resting_shapes.items():
            resting = pose("down", "up") if lowered_side == "L" else pose("up", "down")
            resting[el] = resting[sh] + np.array(elbow) * [sign, 1]
            resting[wr] = resting[sh] + np.array(wrist) * [sign, 1]
            for roll in [-20, 0, 20]:
                theta = math.radians(roll)
                rot = np.array([[math.cos(theta), -math.sin(theta)],
                                [math.sin(theta), math.cos(theta)]])
                for mirror in [-1, 1]:
                    transformed = (resting * [mirror, 1]) @ rot.T + 500
                    configs = [GeometryConfig(camera_upright=False), GeometryConfig(require_hips=True)]
                    if roll == 0:
                        configs.append(GeometryConfig())
                    for config in configs:
                        label, detail = classify(transformed, cf, config)
                        check(label == "FOLLOW" and detail["arms"][lowered_side]["down"],
                              f"FOLLOW: {lowered_side} {name}, roll {roll}, mirror {mirror}, config {config}")
    not_down_shapes = {
        "sideways": ([60, 0], [120, 0]),
        "too far sideways": ([52, 30], [104, 60]),
        "forearm back up": ([60, 90], [40, 40]),
        "wrist above shoulder": ([60, 30], [40, -40])}
    for name, (elbow, wrist) in not_down_shapes.items():
        other = pose("up", "down")
        other[R_EL], other[R_WR] = other[R_SH] + elbow, other[R_SH] + wrist
        label, detail = classify(other, cf)
        check(label == "NONE" and not detail["arms"]["R"]["down"], f"reject FOLLOW: {name}")
        check(detail["reason"] == "FOLLOW blocked: R arm not down" and bool(detail["hint"]),
              f"explain rejected lowered arm: {name}")
    check("rearm" in status_rows("FOLLOW", "STOP", 1, "GESTURE", 1, 30, False,
                                  {"reason": "ok"}, "locked")[-1],
          "explain recognized FOLLOW that needs arms-down release")
    for name, (elbow, wrist) in resting_shapes.items():
        neutral = pose("down", "down")
        for sh, el, wr, sign in [(L_SH, L_EL, L_WR, -1), (R_SH, R_EL, R_WR, 1)]:
            neutral[el] = neutral[sh] + np.array(elbow) * [sign, 1]
            neutral[wr] = neutral[sh] + np.array(wrist) * [sign, 1]
        for raised_side in ["L", "R"]:
            follow = neutral.copy()
            sh, el, wr = ((L_SH, L_EL, L_WR) if raised_side == "L" else (R_SH, R_EL, R_WR))
            follow[el], follow[wr] = follow[sh] + [0, -60], follow[sh] + [0, -120]
            replay, emitted, timestamp = GestureDebouncer(), [], 0.0
            for sample in [neutral] * 12 + [follow] * 30:
                timestamp = round(timestamp + 0.05, 6)
                observed, _ = classify(sample, arms_conf)
                accepted, _, fired = replay.update(observed, timestamp)
                if fired:
                    emitted.append(accepted)
            check(emitted == ["FOLLOW"], f"natural neutral -> held FOLLOW once: {name}, {raised_side}")

    deb = GestureDebouncer()
    tick = 0.0

    def feed(label, seconds):
        nonlocal tick
        events = []
        for _ in range(int(round(seconds / 0.05))):
            tick = round(tick + 0.05, 6)
            accepted, _, fired = deb.update(label, tick)
            if fired:
                events.append(accepted)
        return events

    check(not feed("FOLLOW", 1.0), "startup needs neutral")
    feed("NEUTRAL", 0.5)
    check(feed("FOLLOW", 0.85) == ["FOLLOW"], "held FOLLOW")
    check(not feed("FOLLOW", 2), "no repeated hold")
    check(not feed("UNKNOWN", 0.6) and not feed("FOLLOW", 1), "loss is not release")
    check(feed("STOP", 0.5) == ["STOP"], "STOP bypasses disarmed state")
    check(not feed("STOP", 1), "STOP does not repeat")
    check(not feed("COME", 1), "new motion command needs neutral")
    feed("NEUTRAL", 0.5)
    check(feed("COME", 0.85) == ["COME"], "rearmed COME")
    feed("NEUTRAL", 0.5)
    check(feed("FOLLOW", 0.85) == ["FOLLOW"], "different command after release")
    check(feed("STOP", 0.5) == ["STOP"], "STOP bypasses cooldown")
    feed("NEUTRAL", 0.5)
    feed("FOLLOW", 0.4)
    feed("UNKNOWN", 0.05)
    check(not feed("FOLLOW", 0.4), "missing sample breaks hold")
    deb.update("NEUTRAL", tick + 1)
    tick += 1
    check(not feed("COME", 1), "long interval disarms")
    try:
        deb.update("NEUTRAL", tick)
    except ValueError:
        check(True, "reject repeated timestamp")
    else:
        check(False, "reject repeated timestamp")

    deb, tick = GestureDebouncer(cooldown_sec=3), 0.0
    feed("NEUTRAL", 0.5)
    feed("FOLLOW", 0.85)
    feed("NEUTRAL", 0.5)
    check(not feed("COME", 0.85), "motion cooldown applies across commands")
    check(feed("COME", 1.7) == ["COME"], "motion accepted after cooldown")
    check(feed("STOP", 0.5) == ["STOP"], "priority STOP during motion cooldown")
    feed("UNKNOWN", 0.6)
    check(not feed("STOP", 0.6), "loss does not unlatch STOP")

    sparse = GestureDebouncer(stop_sec=0.1)
    sparse.update("STOP", 0)
    check(not sparse.update("STOP", 0.2)[2], "two sparse samples cannot confirm")

    for side in ["L", "R"]:
        fetch_pose = pose("out", "down") if side == "L" else pose("down", "out")
        replay, events = GestureDebouncer(), []
        for frame_no, sample in enumerate([pose("down", "down")] * 12 + [fetch_pose] * 45):
            observed, _ = classify(sample, arms_conf)
            accepted, _, fired = replay.update(observed, (frame_no + 1) * 0.05)
            if fired:
                events.append(accepted)
        check(events == ["FETCH"], f"held {side}-arm FETCH emits once after neutral")
    deb, tick = GestureDebouncer(), 0.0
    check(not feed("FETCH", 1), "FETCH needs neutral at startup")
    feed("NEUTRAL", 0.5)
    check(feed("FETCH", 0.85) == ["FETCH"], "held FETCH")
    check(not feed("FETCH", 1), "FETCH does not repeat")
    check(feed("STOP", 0.5) == ["STOP"], "STOP interrupts FETCH")
    check(not feed("FETCH", 1), "FETCH after STOP needs neutral")
    feed("NEUTRAL", 0.5)
    check(feed("FETCH", 0.85) == ["FETCH"], "FETCH rearmed after STOP")
    feed("UNKNOWN", 0.6)
    check(not feed("FETCH", 1), "missing pose does not rearm FETCH")
    inward = pose("out", "down")
    inward[L_EL], inward[L_WR] = inward[L_SH] + [60, 0], inward[L_SH] + [120, 0]
    check(classify(inward, cf)[0] == "NONE", "cross-body arm does not trigger FETCH")
    bent_fetch = raised_arm(pose("down", "down"), "L", 90, 60)
    check(classify(bent_fetch, cf)[0] == "NONE", "folded sideways arm does not trigger FETCH")
    fetch_boundary = raised_arm(pose("down", "down"), "L", 68, 0)
    check(classify(fetch_boundary, cf)[0] == "NONE" and
          classify(fetch_boundary, cf, relaxed=True)[0] == "FETCH", "FETCH angular hysteresis")
    weak_fetch = cf.copy()
    weak_fetch[L_WR] = 0
    check(classify(pose("out", "down"), weak_fetch)[0] == "UNKNOWN", "FETCH still requires its wrist")
    fetch_rows = status_rows("FETCH", "NONE", 0.5, "GESTURE", 1, 30, False,
                             {"reason": "ok"}, "locked")
    check("LONG RANGE" in fetch_rows[0] and "rearm" in fetch_rows[-1], "FETCH range and release in UI")
    import io
    from contextlib import redirect_stdout
    output = io.StringIO()
    with redirect_stdout(output):
        payload = send_command("FETCH", {"track_id": 5, "sequence": 1})
    check(json.loads(output.getvalue()) == payload and payload["command"] == "FETCH" and
          payload["range_mode"] == "long" and payload["track_id"] == 5,
          "FETCH JSON intent declares long-range rover behavior")
    weak_pose = cf.copy()
    weak_pose[R_WR] = 0.2
    check(pose_quality(p, cf, REQUIRED) > pose_quality(p, weak_pose, REQUIRED),
          "refinement must improve weakest required joint")
    invalid_pose = p.copy()
    invalid_pose[R_WR] = np.nan
    check(pose_quality(p, weak_pose, REQUIRED) > pose_quality(invalid_pose, cf, REQUIRED),
          "a missing refined wrist cannot displace a visible wrist")

    clock = MediaClock(30)
    check(np.allclose([clock.update(x) for x in [0, 0, 0]], [0, 1/30, 2/30]),
          "video timestamp fallback")
    boxes = np.array([[0, 0, 100, 200], [300, 0, 500, 400]], dtype=float)
    lock = TargetLock()
    check(lock.update(np.array([1, 2]), boxes, 0)[0] is None, "multiple people need selection")
    lock.select(1)
    check(lock.update(np.array([1, 2]), boxes, 0.1)[0] == 0, "selected person not biggest")
    check(lock.update(np.array([2]), boxes[1:], 0.2)[0] is None, "never switch to bystander")
    check(lock.update(np.array([1]), boxes[:1], 1)[0] is None and lock.blocked,
          "reappearance after loss needs reselection")
    lock.select(1)
    check(lock.update(np.array([1, 2]), np.array([boxes[0], boxes[0]]), 2)[0] is None,
          "overlap blocks identity")
    lock = TargetLock()
    lock.update(np.array([1]), boxes[:1], 0)
    check(lock.update(np.array([1]), boxes[:1], 3)[0] == 0 and not lock.blocked,
          "slow first inference does not permanently block a continuously visible target")
    lock.update(np.empty(0, dtype=int), np.empty((0, 4)), 3.1)
    check(lock.update(np.array([1]), boxes[:1], 3.2)[0] == 0,
          "short observed dropout preserves selected identity")
    lock.update(np.empty(0, dtype=int), np.empty((0, 4)), 3.3)
    check(lock.update(np.array([1]), boxes[:1], 4)[0] is None and lock.blocked,
          "actual target loss still requires reselection")
    check(crop_bounds(boxes[0], (480, 640)) == (0, 0, 140, 280), "padded crop clipped")
    check(match_crop_pose(boxes, boxes[0]) == 0, "crop matches target")
    check(match_crop_pose(np.array([boxes[0], boxes[0]]), boxes[0]) is None,
          "ambiguous crop abstains")
    print(f"PASS: {count} deterministic checks (no camera/model benchmark).")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default="yolo11n-pose.pt")
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--video")
    ap.add_argument("--video-fps", type=float, help="fallback FPS if video metadata is absent")
    ap.add_argument("--imgsz", type=int,
                    help="tracking input size (default: 960 long-range; 640 with --no-long-range)")
    ap.add_argument("--long-range", action=argparse.BooleanOptionalAction, default=True,
                    help="use higher-resolution tracking and target-crop pose refinement (default: enabled)")
    ap.add_argument("--conf", type=float, default=0.3)
    ap.add_argument("--kp-conf", type=float, default=0.5)
    ap.add_argument("--up-angle", type=float, default=GeometryConfig.up_deg,
                    help="maximum raised-arm tilt from vertical in degrees (default: 45)")
    ap.add_argument("--up-elbow", type=float, default=GeometryConfig.up_straight_deg,
                    help="minimum raised-arm elbow angle in degrees (default: 135; 180 is straight)")
    ap.add_argument("--min-torso-px", type=float, default=12)
    ap.add_argument("--require-hips", action="store_true",
                    help="require visible hips and use the full-torso body frame (default: arms only)")
    ap.add_argument("--shoulder-frame", action="store_true",
                    help="measure directions relative to the shoulder line for a tilted camera")
    ap.add_argument("--hold", type=float, default=0.7)
    ap.add_argument("--release", type=float, default=0.4)
    ap.add_argument("--cooldown", type=float, default=1.2)
    ap.add_argument("--stop-hold", type=float, default=0.35)
    ap.add_argument("--max-gap", type=float, default=0.25)
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--device", default="0",
                    help="Ultralytics device (default: 0 for CUDA GPU 0; cpu for CPU)")
    ap.add_argument("--half", action="store_true",
                    help="request FP16 on supported GPU/backend (Ultralytics quantize=16)")
    ap.add_argument("--target-id", type=int)
    ap.add_argument("--manual-target", action="store_true")
    ap.add_argument("--tracker", default="botsort.yaml")
    ap.add_argument("--crop-pose", action="store_true")
    ap.add_argument("--detector", default="yolo11n.pt")
    ap.add_argument("--pose-imgsz", type=int, default=640)
    ap.add_argument("--mode", choices=["gesture", "voice"], default="gesture")
    ap.add_argument("--ros-topic", help="publish accepted intents as std_msgs/String JSON")
    ap.add_argument("--log", help="per-frame JSONL results for labeled video evaluation")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return
    if args.imgsz is None:
        args.imgsz = 960 if args.long_range else 640
    if (not 0 < args.kp_conf <= 1 or not 0 < args.conf <= 1 or
            not all(math.isfinite(v) and v > 0 for v in
                    [args.hold, args.release, args.cooldown, args.stop_hold, args.max_gap,
                     args.min_torso_px, args.imgsz, args.pose_imgsz, args.width, args.height])):
        ap.error("confidence must be in (0,1]; thresholds/sizes must be positive and finite")
    if not 0 < args.up_angle <= 60 or not 100 <= args.up_elbow <= 180:
        ap.error("--up-angle must be in (0,60]; --up-elbow must be in [100,180]")

    import cv2
    from ultralytics import YOLO

    pose_model = YOLO(args.model, task="pose")
    refine_pose = args.crop_pose or args.long_range
    # Keep crop prediction separate from tracking: the tracker's callbacks/state
    # must not be advanced a second time for the same source frame.
    detector = (YOLO(args.detector) if args.crop_pose else
                YOLO(args.model, task="pose") if args.long_range else pose_model)
    cap = cv2.VideoCapture(args.video if args.video else args.cam)
    if not cap.isOpened():
        raise SystemExit("Could not open video source")
    if not args.video:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    node, publisher, log_file, ros = None, None, None, None
    try:
        if args.ros_topic:
            import rclpy
            from std_msgs.msg import String
            ros = rclpy
            ros.init()
            node = ros.create_node("urc_gesture_input")
            publisher = node.create_publisher(String, args.ros_topic, 10)
        if args.log:
            log_file = open(args.log, "w", encoding="utf-8")
        clock = MediaClock(args.video_fps or cap.get(cv2.CAP_PROP_FPS)) if args.video else None
        geometry = GeometryConfig(kp_conf=args.kp_conf, min_torso_px=args.min_torso_px,
                                  require_hips=args.require_hips, up_deg=args.up_angle,
                                  up_straight_deg=args.up_elbow, camera_upright=not args.shoulder_frame)
        required_joints = REQUIRED + [L_HIP, R_HIP] if args.require_hips else REQUIRED
        deb = GestureDebouncer(args.hold, args.release, args.cooldown, args.stop_hold, args.max_gap)
        target = TargetLock(args.target_id, auto=not args.manual_target)
        mode, sequence, frame_index, fps = args.mode.upper(), 0, 0, 0.0
        session_id, previous_start = uuid.uuid4().hex, None
        ui = {"click": None, "ids": np.empty(0), "boxes": np.empty((0, 4))}
        window = "URC gestures: click astronaut | r reset | g gesture | v voice | q quit"
        if not args.headless:
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)

            def on_mouse(event, x, y, flags, param):
                if event == cv2.EVENT_LBUTTONDOWN:
                    ui["click"] = (x, y)

            cv2.setMouseCallback(window, on_mouse)
        # Even half=False triggers a deprecation warning on every inference call.
        options = dict(conf=args.conf, verbose=False, quantize=16 if args.half else 32)
        if args.device is not None:
            options["device"] = args.device
        print("Start with arms down. Commands are intents; VOICE requires an external recognizer.", flush=True)
        while True:
            start = time.monotonic()
            ok, frame = cap.read()
            if not ok:
                break
            if frame_index == 0:
                print(f"Actual source frames: {frame.shape[1]}x{frame.shape[0]}", flush=True)
            timestamp = clock.update(cap.get(cv2.CAP_PROP_POS_MSEC)) if clock else time.monotonic()
            result = detector.track(frame, persist=True, tracker=args.tracker, classes=[0],
                                    imgsz=args.imgsz, **options)[0]
            if frame_index == 0:
                print(f"Inference device: {detector.predictor.device}", flush=True)
            boxes = result.boxes.xyxy.cpu().numpy() if result.boxes is not None else np.empty((0, 4))
            ids = (result.boxes.id.cpu().numpy().astype(int)
                   if result.boxes is not None and result.boxes.id is not None else np.empty(0, dtype=int))
            old_id = target.target_id
            if ui["click"] is not None:
                x, y = ui.pop("click")
                ui["click"] = None
                hits = [int(track) for track, box in zip(ui["ids"], ui["boxes"])
                        if box[0] <= x <= box[2] and box[1] <= y <= box[3]]
                if len(hits) == 1:
                    target.select(hits[0])
                    deb.reset()
            idx, target_status = target.update(ids, boxes, timestamp)
            if old_id != target.target_id:
                deb.reset()
            if target.blocked:
                deb.reset()
            raw, detail = "UNKNOWN", {"reason": target_status}
            selected_kp, selected_cf = None, None
            pose_source = "none"
            if idx is not None:
                kps, confs = pose_arrays(result)
                if kps is not None:
                    selected_kp, selected_cf = kps[idx], confs[idx]
                    pose_source = "full frame"
                if refine_pose:
                    x1, y1, x2, y2 = crop_bounds(boxes[idx], frame.shape)
                    crop_result = pose_model.predict(frame[y1:y2, x1:x2], imgsz=args.pose_imgsz,
                                                     classes=[0], **options)[0]
                    kps, confs = pose_arrays(crop_result)
                    if kps is not None:
                        mapped_boxes = crop_result.boxes.xyxy.cpu().numpy() + [x1, y1, x1, y1]
                        matched = match_crop_pose(mapped_boxes, boxes[idx])
                        if matched is not None:
                            crop_kp, crop_cf = kps[matched] + [x1, y1], confs[matched]
                            if (selected_kp is None or
                                    pose_quality(crop_kp, crop_cf, required_joints) >
                                    pose_quality(selected_kp, selected_cf, required_joints)):
                                selected_kp, selected_cf = crop_kp, crop_cf
                                pose_source = "target crop"
                if selected_kp is not None:
                    raw, detail = classify(selected_kp, selected_cf, geometry)
                    if raw == "NONE" and deb.candidate in COMMANDS:
                        loose, loose_detail = classify(selected_kp, selected_cf, geometry, relaxed=True)
                        if loose == deb.candidate:
                            raw, detail = loose, loose_detail
                    h, w = frame.shape[:2]
                    joints = selected_kp[required_joints]
                    if raw != "UNKNOWN" and (np.any(joints <= 2) or
                                              np.any(joints >= [w - 2, h - 2])):
                        raw, detail = "UNKNOWN", {"reason": "Required joints at image edge",
                            "hint": "Step back: keep both shoulders, elbows and wrists inside the picture."
                                    + (" Include hips in this mode." if args.require_hips else "")}
                else:
                    detail = {"reason": "missing/ambiguous pose or confidence"}
            detail.setdefault("body_frame", "shoulders + hips" if args.require_hips else
                              "shoulders (arms only)" if args.shoulder_frame else "upright camera (arms only)")
            detail["pose_source"] = pose_source
            confirmed, progress, fired = deb.update(raw if mode == "GESTURE" else "UNKNOWN", timestamp)
            event = None
            if fired:
                sequence += 1
                event = send_command(confirmed, {"session_id": session_id, "sequence": sequence,
                                                "track_id": int(target.target_id),
                                                "source_time": timestamp}, publisher)
            if node is not None:
                ros.spin_once(node, timeout_sec=0)
            processing_ms = (time.monotonic() - start) * 1000
            # Consecutive loop starts include display, logging and UI overhead.
            fps_now = 1 / max(start - previous_start, 1e-6) if previous_start is not None else 0
            previous_start = start
            fps = fps_now if fps == 0 else 0.9 * fps + 0.1 * fps_now
            if log_file:
                log_file.write(json.dumps(dict(frame=frame_index, source_time=timestamp, mode=mode,
                    track_id=target.target_id, raw=raw, last_command=confirmed, armed=deb.armed,
                    progress=progress, detail=detail, event=event, loop_fps=round(fps, 2),
                    processing_ms=round(processing_ms, 2)),
                    allow_nan=False) + "\n")
            frame_index += 1
            if not args.headless:
                annotated = result.plot()
                if idx is not None:
                    bx1, by1, bx2, by2 = boxes[idx].astype(int)
                    cv2.rectangle(annotated, (bx1, by1), (bx2, by2), (0, 255, 255), 3)
                if selected_kp is not None:
                    for joint in required_joints:
                        if selected_cf[joint] >= args.kp_conf and np.isfinite(selected_kp[joint]).all():
                            cv2.circle(annotated, tuple(selected_kp[joint].astype(int)), 3, (0, 255, 255), -1)
                rows = status_rows(raw, confirmed, progress, mode, target.target_id,
                                   fps, deb.armed, detail, target_status)
                annotated = draw_status_panel(annotated, rows)
                ui["ids"], ui["boxes"] = ids.copy(), boxes.copy()
                cv2.imshow(window, annotated)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("r"):
                    target.select(None)
                    deb.reset()
                if key in (ord("g"), ord("v")):
                    mode = "GESTURE" if key == ord("g") else "VOICE"
                    deb.reset()
    finally:
        cap.release()
        if log_file:
            log_file.close()
        if node is not None:
            node.destroy_node()
        if ros is not None and ros.ok():
            ros.shutdown()
        if not args.headless:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user.", flush=True)

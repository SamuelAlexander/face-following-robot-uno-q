# SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL (http://www.arduino.cc)
#
# SPDX-License-Identifier: MPL-2.0

# -----------------------------------------------------------------------
# MPU Python App: Object detection -> wheel pulse controller
#
# Uses the VideoObjectDetection brick to detect objects on a USB camera.
# For TARGET_CLASS, the largest bounding box is used as the active target.
# The controller computes heading error from bbox center vs frame center
# and converts it into differential left/right wheel turn commands.
# -----------------------------------------------------------------------

from arduino.app_utils import App, Bridge
from arduino.app_bricks.web_ui import WebUI
from arduino.app_bricks.video_objectdetection import VideoObjectDetection
from datetime import datetime, UTC
import math
import threading
import time

# -----------------------------------------------------------------------
# Configuration — tweak these to change behaviour
# -----------------------------------------------------------------------

# Object class to track. Must match the model label exactly.
# Built-in face-detection model outputs "face".
TARGET_CLASS = "face"

# Minimum detection confidence (0.0-1.0).
DETECTION_CONFIDENCE = 0.5

# Seconds without any detection callback before the watchdog forces stop.
# At 3-4 FPS this must be comfortably above the frame interval (~300 ms).
NO_DETECTION_TIMEOUT = 1.5

# --- Follow controller tuning -------------------------------------------
# Servo pulse range for continuous-rotation wheels.
STOP_PULSE_US = 1500
MIN_PULSE_US = 1000
MAX_PULSE_US = 2000
PULSE_RANGE_US = 500

# Proportional gain: how aggressively turn speed scales with heading error.
# Higher = snappier but risks overshooting. Start low for desk testing.
STEER_GAIN = 0.15
# Hard cap on wheel speed used for turning [0-1]. 0.25 ≈ ±125 us from stop.
# Must exceed the servo dead zone (~±25-30 us for Parallax CR servos).
MAX_TURN_SPEED = 0.25
# Response curve exponent (1.0 = linear, <1.0 = compress large errors).
# Lower values make edge-of-frame speed closer to slight-off-center speed.
STEER_CURVE = 0.4
# Ignore heading errors smaller than this (suppresses jitter).
CENTER_DEADBAND = 0.05
# Seconds to coast on the last known target position before declaring lost.
TRACKING_TIMEOUT = 0.5

# --- Manual drive bias ----------------------------------------------------
# Fixed forward/backward bias independent of object size.
# Negative = backward, positive = forward, 0.0 = turn-only.
DRIVE_BIAS = 0.0
# Hard cap for manual drive bias [0-1]. 0.08 ≈ ±40 us from stop.
MAX_DRIVE_BIAS = 0.08

# Fallback frame dimensions when detector payload does not include frame size.
# Adjust to match your camera resolution (common: 320, 480, 640, 1280).
FRAME_WIDTH_FALLBACK = 640.0
FRAME_HEIGHT_FALLBACK = 480.0

# If left/right tracking is mirrored, set STEER_SIGN to -1.0.
STEER_SIGN = 1.0

# Match MCU wheel inversion so debug status reflects physical motion.
LEFT_WHEEL_INVERTED = False
RIGHT_WHEEL_INVERTED = True

# Lost-target behavior: default stop, optionally rotate slowly to search.
SEARCH_WHEN_LOST = False
SEARCH_SPEED = 0.2

# -----------------------------------------------------------------------
# Bricks initialisation
# -----------------------------------------------------------------------

ui = WebUI()
detection_stream = VideoObjectDetection(
    confidence=DETECTION_CONFIDENCE, debounce_sec=0.0
)

# Let the web UI adjust the confidence threshold live via a slider.
ui.on_message(
    "override_th",
    lambda sid, threshold: detection_stream.override_threshold(threshold),
)


def _set_steer_gain(sid, value):
    global STEER_GAIN
    STEER_GAIN = _clamp(_safe_float(value, STEER_GAIN), 0.0, 0.30)
    print(f"steer_gain overridden to {STEER_GAIN:.3f}")


ui.on_message("override_steer_gain", _set_steer_gain)


def _set_drive_bias(sid, value):
    global DRIVE_BIAS
    DRIVE_BIAS = _clamp(_safe_float(value, DRIVE_BIAS), -MAX_DRIVE_BIAS, MAX_DRIVE_BIAS)
    print(f"drive_bias overridden to {DRIVE_BIAS:.3f}")


ui.on_message("override_drive_bias", _set_drive_bias)

_emergency_stopped = False


def _emergency_stop(sid, value=None):
    global _emergency_stopped
    _emergency_stopped = True
    _send_wheel_cmd(STOP_PULSE_US, STOP_PULSE_US, "emergency_stop", urgent=True)
    print("EMERGENCY STOP activated")


def _emergency_release(sid, value=None):
    global _emergency_stopped
    _emergency_stopped = False
    print("Emergency stop released")


ui.on_message("emergency_stop", _emergency_stop)
ui.on_message("emergency_release", _emergency_release)


def _motor_test(sid, value=None):
    """Run a left-right-stop test sequence to verify servo wiring.

    Uses large pulse offsets (1300/1700) and sends directly via Bridge.notify
    to bypass all control logic and rate limiting — pure hardware test.
    """
    global _emergency_stopped
    _emergency_stopped = True  # Pause tracking during test.
    print("MOTOR TEST: starting")

    steps = [
        ("left",  1300, 1700, 1.5),
        ("stop",  1500, 1500, 0.5),
        ("right", 1700, 1300, 1.5),
        ("stop",  1500, 1500, 0.0),
    ]
    for label, left, right, pause in steps:
        print(f"MOTOR TEST: {label} L={left} R={right}")
        Bridge.notify("set_wheel_pwm", left, right)
        if pause > 0:
            time.sleep(pause)

    _emergency_stopped = False
    ui.send_message("motor_test_done", message={})
    print("MOTOR TEST: done")


ui.on_message("motor_test", _motor_test)

# -----------------------------------------------------------------------
# Wheel command tracking
# -----------------------------------------------------------------------

_last_wheel = {"left": STOP_PULSE_US, "right": STOP_PULSE_US}
_last_detection_ts = 0.0
_watchdog_started = False
_logged_sample = False
_last_state_publish_ts = 0.0
_last_bridge_ts = 0.0
# Minimum interval between Bridge.notify() calls.
# The UNO Q Bridge has no queue — spamming notify() crashes the serial link.
MIN_BRIDGE_INTERVAL = 0.05  # 50 ms → max 20 cmd/s

# Cached target for coasting when detection is momentarily lost.
_last_target = None
_last_target_ts = 0.0


# -----------------------------------------------------------------------
# Bounding-box helpers
# -----------------------------------------------------------------------

def _bbox_from_entry(entry):
    """Extract (xmin, ymin, xmax, ymax) from a detection dict, or None.

    The VideoObjectDetection brick produces:
        {"confidence": float, "bounding_box_xyxy": (xmin, ymin, xmax, ymax)}
    """
    if not isinstance(entry, dict):
        return None
    bb = entry.get("bounding_box_xyxy")
    if isinstance(bb, (list, tuple)) and len(bb) >= 4:
        return bb[0], bb[1], bb[2], bb[3]
    return None


def _box_center_x(entry):
    """Return the horizontal centre of a detection's bounding box (pixels)."""
    bb = _bbox_from_entry(entry)
    if bb is None:
        return None
    return (bb[0] + bb[2]) / 2


def _box_area(entry):
    """Return bbox area in pixels."""
    bb = _bbox_from_entry(entry)
    if bb is None:
        return None
    w = max(0.0, bb[2] - bb[0])
    h = max(0.0, bb[3] - bb[1])
    return w * h


# -----------------------------------------------------------------------
# Detection → left/right decision
# -----------------------------------------------------------------------

def _collect_target_boxes(detections):
    """Return boxes for TARGET_CLASS from the brick payload.

    The VideoObjectDetection brick wraps detections in a list per label:
        {label: [{"confidence": float, "bounding_box_xyxy": (x0,y0,x1,y1)}, ...]}
    """
    boxes = []
    if not isinstance(detections, dict):
        return boxes

    entry = detections.get(TARGET_CLASS)
    if isinstance(entry, list):
        for det in entry:
            if isinstance(det, dict) and "bounding_box_xyxy" in det:
                boxes.append(det)
    elif isinstance(entry, dict) and "bounding_box_xyxy" in entry:
        boxes.append(entry)

    return boxes


def _clamp(value, low, high):
    return max(low, min(high, value))


def _safe_float(value, fallback):
    try:
        return float(value)
    except (TypeError, ValueError):
        return fallback


def _speed_to_pulse(speed):
    speed = _clamp(speed, -1.0, 1.0)
    pulse = STOP_PULSE_US + int(speed * PULSE_RANGE_US)
    return _clamp(pulse, MIN_PULSE_US, MAX_PULSE_US)


def _pulse_to_speed(pulse):
    return _clamp((pulse - STOP_PULSE_US) / PULSE_RANGE_US, -1.0, 1.0)


def _movement_status(left_us, right_us, mode):
    left_speed = _pulse_to_speed(left_us)
    right_speed = _pulse_to_speed(right_us)
    forward = (left_speed + right_speed) * 0.5
    turn = (left_speed - right_speed) * 0.5

    if abs(left_speed) < 0.05 and abs(right_speed) < 0.05:
        status = "stopped"
    elif abs(turn) > abs(forward) + 0.05:
        status = "turning right" if turn > 0 else "turning left"
    elif forward > 0:
        status = "going forward"
    else:
        status = "going backward"

    if mode == "search":
        status = f"searching ({status})"
    elif mode == "lost_stop":
        status = "lost target - stopped"

    return status, round(forward, 2), round(turn, 2)


def _best_target(boxes):
    """Return (center_x_norm, area_frac) for the largest valid target, or None."""
    best_cx = None
    best_area = -1.0
    best_bbox = None
    for box in boxes:
        cx = _box_center_x(box)
        if cx is None:
            continue

        area = _box_area(box) or 0.0

        # bbox coordinates are in pixels; normalise cx to 0-1.
        center_x_norm = cx / FRAME_WIDTH_FALLBACK

        if area > best_area:
            best_area = area
            best_cx = center_x_norm
            best_bbox = _bbox_from_entry(box)

    if best_cx is None:
        return None

    frame_area = FRAME_WIDTH_FALLBACK * FRAME_HEIGHT_FALLBACK
    area_frac = best_area / frame_area if frame_area > 0 else 0.0

    return best_cx, area_frac, best_bbox


def _compute_wheel_command(detections: dict):
    """Map detections to wheel command tuple, or None when target not found."""
    global _last_target, _last_target_ts

    boxes = _collect_target_boxes(detections)
    result = _best_target(boxes)

    if result is not None:
        target_cx, area_frac, bbox = result
        _last_target = (target_cx, area_frac, bbox)
        _last_target_ts = time.monotonic()
        mode = "tracking"
    elif _last_target is not None and (time.monotonic() - _last_target_ts) < TRACKING_TIMEOUT:
        target_cx, area_frac, bbox = _last_target
        mode = "coasting"
    else:
        _last_target = None
        return None

    # --- Steering (left/right) ---
    heading_error = (target_cx - 0.5) * 2.0
    if abs(heading_error) < CENTER_DEADBAND:
        heading_error = 0.0

    shaped = math.copysign(abs(heading_error) ** STEER_CURVE, heading_error)
    turn = _clamp(STEER_SIGN * STEER_GAIN * shaped, -MAX_TURN_SPEED, MAX_TURN_SPEED)

    # --- Manual drive bias (forward/backward) ---
    forward = DRIVE_BIAS

    # Turn pulses first, then layer forward/backward as an equal pulse shift
    # on both wheels.  This works correctly with MCU-side inversion: adding
    # +N us to both wheels here becomes +N on left and -N (physical) on right
    # after the MCU mirrors the right pulse around 1500 us → net forward.
    left_us = _speed_to_pulse(turn)
    right_us = _speed_to_pulse(-turn)
    fwd_offset = int(forward * PULSE_RANGE_US)
    left_us = _clamp(left_us + fwd_offset, MIN_PULSE_US, MAX_PULSE_US)
    right_us = _clamp(right_us + fwd_offset, MIN_PULSE_US, MAX_PULSE_US)
    print(f"controller cx={target_cx:.3f} heading={heading_error:.3f} turn={turn:.3f} area={area_frac:.3f} fwd={forward:.3f} mode={mode}")
    return left_us, right_us, mode, area_frac


def _lost_target_command():
    if not SEARCH_WHEN_LOST:
        return STOP_PULSE_US, STOP_PULSE_US, "lost_stop", None
    rotate = _clamp(SEARCH_SPEED, 0.0, 1.0)
    left_us = _speed_to_pulse(rotate)
    right_us = _speed_to_pulse(-rotate)
    return left_us, right_us, "search", None


# -----------------------------------------------------------------------
# Bridge communication
# -----------------------------------------------------------------------

def _send_wheel_cmd(left_us: int, right_us: int, mode: str, area_frac=None,
                    urgent: bool = False):
    """Send wheel pulse command and publish debug state to the UI.

    Set urgent=True for safety-critical commands (e.g. emergency stop) to
    bypass rate limiting and the same-command cache.
    """
    global _last_state_publish_ts, _last_bridge_ts
    same_cmd = left_us == _last_wheel["left"] and right_us == _last_wheel["right"]
    status, forward, turn = _movement_status(left_us, right_us, mode)
    now = time.monotonic()

    if urgent or (not same_cmd) or (now - _last_state_publish_ts > 0.5):
        print(
            f"wheel_cmd left_us={left_us} right_us={right_us} "
            f"status='{status}' mode={mode} fwd={forward} turn={turn}"
        )
        msg = {
            "status": status,
            "mode": mode,
            "left_us": left_us,
            "right_us": right_us,
            "forward": forward,
            "turn": turn,
            "left_speed": round(_pulse_to_speed(left_us), 2),
            "right_speed": round(_pulse_to_speed(right_us), 2),
            "timestamp": datetime.now(UTC).isoformat(),
        }
        if area_frac is not None:
            msg["area_frac"] = round(area_frac, 4)
        ui.send_message("robot_state", message=msg)
        _last_state_publish_ts = now

    if same_cmd and not urgent:
        return

    # Rate-limit Bridge calls to avoid crashing the serial link.
    if not urgent and now - _last_bridge_ts < MIN_BRIDGE_INTERVAL:
        return

    _last_wheel["left"] = left_us
    _last_wheel["right"] = right_us
    _last_bridge_ts = now
    Bridge.notify("set_wheel_pwm", left_us, right_us)


# -----------------------------------------------------------------------
# Watchdog — stop servos when detections dry up
# -----------------------------------------------------------------------

def _watchdog_loop():
    """Background thread: fallback command if detection callbacks stop."""
    while True:
        time.sleep(0.25)
        if _emergency_stopped:
            continue
        if time.monotonic() - _last_detection_ts > NO_DETECTION_TIMEOUT:
            _send_wheel_cmd(*_lost_target_command())


# -----------------------------------------------------------------------
# Main detection callback
# -----------------------------------------------------------------------

def on_detections(detections):
    """Called by the VideoObjectDetection brick whenever objects are found."""
    global _logged_sample, _last_detection_ts, _watchdog_started

    # Log one sample so you can inspect the payload format.
    if not _logged_sample:
        print(f"sample_detections {detections}")
        _logged_sample = True
        if not _watchdog_started:
            _watchdog_started = True
            threading.Thread(target=_watchdog_loop, daemon=True).start()

    _last_detection_ts = time.monotonic()

    if _emergency_stopped:
        return

    # Forward every detection to the web UI.
    # The brick wraps detections as {label: [det, ...]} (list per label).
    if isinstance(detections, dict):
        for key, value in detections.items():
            dets = value if isinstance(value, list) else [value]
            for det in dets:
                if not isinstance(det, dict):
                    continue
                entry = {
                    "content": key,
                    "confidence": det.get("confidence"),
                    "timestamp": datetime.now(UTC).isoformat(),
                }
                bb = det.get("bounding_box_xyxy")
                if bb is not None:
                    entry["bbox"] = list(bb)
                ui.send_message("detection", message=entry)

    # Compute and send wheel command; fallback if target class is missing.
    cmd = _compute_wheel_command(detections)
    if cmd is None:
        cmd = _lost_target_command()
    _send_wheel_cmd(*cmd)


detection_stream.on_detect_all(on_detections)

App.run()

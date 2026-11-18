"""The one place that reads the camera.

A background thread grabs frames, runs lane detection, draws the overlay and
JPEG-encodes it for the web page. Lane following reads the latest result
with latest(). Only one thing may open the camera, so everything else goes
through here.
"""

import json
import tempfile
import threading
import time
from pathlib import Path

import config

_lock = threading.Lock()
_state = {"id": 0, "time": 0.0, "lane": None, "jpeg": None, "frame": None}
_started = False
_reset_track = False
_calibration_lock = threading.Lock()
_calibration_path = Path(__file__).resolve().parents[1] / "hue-calibration.json"


def _validate_calibration(enabled, offset):
    if type(enabled) is not bool or type(offset) is not int or not -90 <= offset <= 90:
        raise ValueError("Expected a toggle and a hue offset from -90 to 90.")
    return {"enabled": enabled, "offset": offset}


def _load_calibration():
    try:
        saved = json.loads(_calibration_path.read_text(encoding="utf-8"))
        return _validate_calibration(saved["enabled"], saved["offset"])
    except FileNotFoundError:
        pass
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"Could not load hue calibration: {exc}")
    return {"enabled": False, "offset": 0}


_calibration = _load_calibration()


def hue_calibration():
    with _calibration_lock:
        return _calibration.copy()


def set_hue_calibration(enabled, offset):
    global _calibration
    settings = _validate_calibration(enabled, offset)
    with _calibration_lock:
        if settings != _calibration:
            temp_path = None
            try:
                with tempfile.NamedTemporaryFile(
                    mode="w", encoding="utf-8", dir=_calibration_path.parent,
                    prefix=".hue-calibration-", delete=False,
                ) as stream:
                    temp_path = Path(stream.name)
                    json.dump(settings, stream)
                temp_path.replace(_calibration_path)
            finally:
                if temp_path is not None:
                    temp_path.unlink(missing_ok=True)
            _calibration = settings
        return _calibration.copy()


def reset_lane_track():
    """Forget the lines followed last frame. The next frame locks onto the lane ahead."""
    global _reset_track
    with _lock:
        _reset_track = True


def latest():
    """Return (frame_id, frame_time, lane_info_or_None, jpeg_bytes_or_None)."""
    with _lock:
        return _state["id"], _state["time"], _state["lane"], _state["jpeg"]


def latest_frame():
    """The raw camera frame (no overlay) with its id, for recording."""
    with _lock:
        return _state["id"], _state["frame"]


def camera_ok():
    with _lock:
        return _state["jpeg"] is not None


def _loop():
    global _reset_track
    from camera import Camera
    from lane_detector import LaneDetector
    camera = Camera()
    if not camera.ok:
        return
    import cv2
    detector = LaneDetector()
    previous_offset = 0
    while True:
        try:
            frame = camera.read()
            if frame is None:
                time.sleep(0.1)
                continue
            with _lock:
                do_reset = _reset_track
                _reset_track = False
            calibration = hue_calibration()
            offset = calibration["offset"] if calibration["enabled"] else 0
            if offset != previous_offset:
                detector = LaneDetector()
                previous_offset = offset
            elif do_reset:
                detector.reset_track()
            lane = detector.detect(frame, hue_offset=offset)
            shown = detector.draw(frame) if config.SHOW_LANES else frame
            ok, buf = cv2.imencode(".jpg", shown, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            with _lock:
                _state["id"] += 1
                _state["time"] = time.monotonic()
                _state["lane"] = lane
                _state["frame"] = frame
                if ok:
                    _state["jpeg"] = buf.tobytes()
        except Exception as exc:
            print(f"Camera read failed: {exc}")
            time.sleep(0.5)


def start():
    global _started
    if _started:
        return
    _started = True
    threading.Thread(target=_loop, name="vision", daemon=True).start()

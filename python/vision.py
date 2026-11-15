"""The one place that reads the camera.

A background thread grabs frames, runs lane detection, draws the overlay and
JPEG-encodes it for the web page. Lane following reads the latest result
with latest(). Only one thing may open the webcam, so everything else goes
through here.
"""

import threading
import time

import config

_lock = threading.Lock()
_state = {"id": 0, "time": 0.0, "lane": None, "jpeg": None, "frame": None}
_started = False


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
    from camera import Camera
    from lane_detector import LaneDetector
    camera = Camera()
    if camera.cap is None:
        return
    import cv2
    detector = LaneDetector()
    while True:
        try:
            frame = camera.read()
            if frame is None:
                time.sleep(0.1)
                continue
            lane = detector.detect(frame)
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

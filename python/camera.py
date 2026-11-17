"""Grabs frames from the camera on the UNO Media Carrier's MIPI-CSI connector.

A CSI camera does not show up as a plain /dev/video webcam; libcamera turns
the raw sensor data into images. OpenCV reads it through a GStreamer
libcamerasrc pipeline when OpenCV was built with GStreamer. The pip
opencv-python wheels are not, so otherwise gst-launch-1.0 runs the same
pipeline and pipes raw BGR frames in. CAMERA_SOURCE = "usb" goes back to a
USB webcam on /dev/video<CAMERA_INDEX>.
"""

import shutil
import subprocess
import threading

import config


def _pipeline_caps():
    # videoscale covers sensors that can't output FRAME_WIDTH x FRAME_HEIGHT directly.
    size = f"width={config.FRAME_WIDTH},height={config.FRAME_HEIGHT}"
    return (f"libcamerasrc ! video/x-raw,{size} ! videoconvert ! videoscale "
            f"! video/x-raw,{size},format=BGR")


class Camera:
    def __init__(self):
        self.ok = False
        self._cv2 = None
        self._cap = None
        self._proc = None
        self._lock = threading.Lock()
        self._new_frame = threading.Condition(self._lock)
        self._frame = None
        try:
            import cv2
        except ImportError:
            print("OpenCV is not installed, so the camera is off.")
            return
        self._cv2 = cv2
        if config.CAMERA_SOURCE == "usb":
            self._open_usb()
        elif self._opencv_has_gstreamer():
            self._open_opencv_gstreamer()
        else:
            self._open_gst_launch()

    def _opencv_has_gstreamer(self):
        for line in self._cv2.getBuildInformation().splitlines():
            if "GStreamer" in line:
                return "YES" in line
        return False

    def _open_usb(self):
        cv2 = self._cv2
        cap = cv2.VideoCapture(config.CAMERA_INDEX, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            print(f"No camera on /dev/video{config.CAMERA_INDEX}. "
                  "Plug a USB webcam into the Uno Q.")
            cap.release()
            return
        self._cap = cap
        self.ok = True

    def _open_opencv_gstreamer(self):
        cv2 = self._cv2
        pipeline = _pipeline_caps() + " ! appsink drop=true max-buffers=1"
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            print("libcamera found no CSI camera. Check the ribbon cable on the "
                  "Media Carrier and that `cam -l` on the Uno Q lists the camera.")
            cap.release()
            return
        self._cap = cap
        self.ok = True

    def _open_gst_launch(self):
        gst = shutil.which("gst-launch-1.0")
        if gst is None:
            print("gst-launch-1.0 is not installed and OpenCV has no GStreamer, "
                  "so the CSI camera is off.")
            return
        # -q keeps GStreamer's status text out of the frame data on stdout.
        args = [gst, "-q"] + _pipeline_caps().split() + ["!", "fdsink", "fd=1"]
        try:
            self._proc = subprocess.Popen(args, stdout=subprocess.PIPE,
                                          stderr=subprocess.DEVNULL,
                                          bufsize=0)
        except OSError as exc:
            print(f"Could not start the CSI camera: {exc}")
            return
        self.ok = True
        threading.Thread(target=self._pipe_reader, name="camera", daemon=True).start()

    def _pipe_reader(self):
        """Keep only the newest frame, so a slow lane detector never falls behind."""
        import numpy as np
        size = config.FRAME_WIDTH * config.FRAME_HEIGHT * 3
        stream = self._proc.stdout
        while True:
            buf = bytearray(size)
            view = memoryview(buf)
            got = 0
            while got < size:
                n = stream.readinto(view[got:])
                if not n:
                    print("CSI camera stopped. Check the ribbon cable on the Media "
                          "Carrier and that `cam -l` on the Uno Q lists the camera.")
                    with self._lock:
                        self.ok = False
                        self._new_frame.notify_all()
                    return
                got += n
            frame = np.frombuffer(buf, dtype=np.uint8).reshape(
                config.FRAME_HEIGHT, config.FRAME_WIDTH, 3)
            with self._lock:
                self._frame = frame
                self._new_frame.notify_all()

    def _orient(self, frame):
        cv2 = self._cv2
        if config.CAMERA_FLIP == "180":
            return cv2.rotate(frame, cv2.ROTATE_180)
        if config.CAMERA_FLIP == "horizontal":
            return cv2.flip(frame, 1)
        if config.CAMERA_FLIP == "vertical":
            return cv2.flip(frame, 0)
        return frame

    def read(self):
        """Return the next frame, or None if the camera is unavailable."""
        if not self.ok:
            return None
        if self._cap is not None:
            ok, frame = self._cap.read()
            return self._orient(frame) if ok else None
        with self._lock:
            if self._frame is None:
                self._new_frame.wait(timeout=1.0)
            frame, self._frame = self._frame, None
        return self._orient(frame) if frame is not None else None

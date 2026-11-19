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
    # Scale before converting so videoconvert only touches the small frame.
    sensor = f"width={config.CAMERA_SENSOR_WIDTH},height={config.CAMERA_SENSOR_HEIGHT}"
    size = f"width={config.FRAME_WIDTH},height={config.FRAME_HEIGHT}"
    return (f"libcamerasrc ! video/x-raw,{sensor} ! videoscale "
            f"! video/x-raw,{size} ! videoconvert ! video/x-raw,{size},format=BGR")


class Camera:
    def __init__(self):
        self.ok = False
        self._cv2 = None
        self._cap = None
        self._proc = None
        self._lock = threading.Lock()
        self._new_frame = threading.Condition(self._lock)
        self._frame = None
        self._wb_gains = None
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

    def _white_balance(self, frame):
        """Gray-world white balance: scale each channel so the frame averages to gray.

        libcamera has no tuning file for this camera, so its colors come out
        with a purple cast that moves the tape colors out of their HSV ranges.
        """
        if not config.CAMERA_WHITE_BALANCE:
            return frame
        import numpy as np
        means = frame[::4, ::4].reshape(-1, 3).mean(axis=0)
        gains = np.clip(means.mean() / np.maximum(means, 1.0), 0.5, 2.5)
        # Smooth over frames so the colors don't flicker as the view changes.
        if self._wb_gains is None:
            self._wb_gains = gains
        else:
            self._wb_gains = 0.9 * self._wb_gains + 0.1 * gains
        return self._cv2.transform(frame, np.diag(self._wb_gains).astype(np.float32))

    def _prepare(self, frame):
        return self._white_balance(self._orient(frame))

    def read(self):
        """Return the next frame, or None if the camera is unavailable."""
        if not self.ok:
            return None
        if self._cap is not None:
            ok, frame = self._cap.read()
            return self._prepare(frame) if ok else None
        with self._lock:
            if self._frame is None:
                self._new_frame.wait(timeout=1.0)
            frame, self._frame = self._frame, None
        return self._prepare(frame) if frame is not None else None

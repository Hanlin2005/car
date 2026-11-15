"""Grabs frames from a USB camera plugged into the Uno Q."""

import config


class Camera:
    def __init__(self):
        self.cap = None
        try:
            import cv2
        except ImportError:
            print("OpenCV is not installed, so the camera is off.")
            return
        try:
            self.cap = cv2.VideoCapture(config.CAMERA_INDEX, cv2.CAP_V4L2)
        except Exception as exc:
            print(f"Could not open the camera: {exc}")
            self.cap = None
            return
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            print(f"No camera on /dev/video{config.CAMERA_INDEX}. "
                  "Plug a USB webcam into the Uno Q.")
            self.cap.release()
            self.cap = None

    def read(self):
        """Return the latest frame, or None if the camera is unavailable."""
        if self.cap is None:
            return None
        ok, frame = self.cap.read()
        return frame if ok else None

"""Camera frame -> where the car is in its lane. Uses the camera only.

The left lane line is yellow tape and the right lane line is red tape. Each
color is picked out in HSV, a straight line is fitted to each one in the
bottom part of the image, and the lane center is the midpoint between them.
"""

import math
from dataclasses import dataclass

import config


@dataclass
class LaneInfo:
    offset: float    # meters from lane center (+ = car is right of center)
    heading: float   # radians between car and lane direction (+ = pointing right)
    left_lane_exists: bool    # the yellow (left) line was seen this frame
    right_lane_exists: bool   # the red (right) line was seen this frame


@dataclass
class _Line:
    """x = slope * y + intercept, in full-frame pixel coordinates."""
    slope: float
    intercept: float
    y_top: int
    y_bottom: int

    def x_at(self, y):
        return self.slope * y + self.intercept


class LaneDetector:
    def __init__(self):
        self._cv2 = None
        self._np = None
        try:
            import cv2
            import numpy as np
            self._cv2, self._np = cv2, np
        except ImportError:
            print("OpenCV/numpy missing, so lane detection is off. "
                  "Install python3-opencv on the Uno Q.")
        # Lane width in pixels at the bottom row, learned when both lines are
        # visible. Used to place the missing line when only one is seen.
        self._lane_px = None
        # Horizon row (where the lane lines meet), learned the same way.
        self._horizon = None
        # The last frame's results, for drawing on the web page.
        self.debug = None

    def detect(self, frame):
        """Return LaneInfo, or None if no lane is visible."""
        self.debug = None
        if frame is None or self._cv2 is None:
            return None
        cv2, np = self._cv2, self._np

        h, w = frame.shape[:2]
        roi_top = int(h * config.LANE_ROI_TOP)
        roi = frame[roi_top:, :]

        # Blur before converting: red's hue wraps from 179 to 0, so blurring
        # in HSV would average red pixels into a nonsense hue.
        hsv = cv2.cvtColor(cv2.GaussianBlur(roi, (5, 5), 0), cv2.COLOR_BGR2HSV)
        left_mask = self._mask(hsv, config.LEFT_LINE_HSV_RANGES)     # yellow
        right_mask = self._mask(hsv, config.RIGHT_LINE_HSV_RANGES)   # red

        left = self._fit(left_mask, roi_top)
        right = self._fit(right_mask, roi_top)

        y_bot = h - 1
        # Yellow has to be on the left of red. If it isn't, something else
        # red or yellow is in view, so trust neither line.
        if left and right and left.x_at(y_bot) >= right.x_at(y_bot):
            left = right = None

        self.debug = {"roi_top": roi_top, "left": left, "right": right,
                      "left_mask": left_mask, "right_mask": right_mask, "info": None}
        if left is None and right is None:
            return None

        # Lane width in pixels at the bottom row.
        if left and right:
            width = right.x_at(y_bot) - left.x_at(y_bot)
            if width > config.LANE_MIN_WIDTH_FRAC * w:
                self._lane_px = width if self._lane_px is None else \
                    0.8 * self._lane_px + 0.2 * width
        lane_px = self._lane_px or config.LANE_DEFAULT_WIDTH_FRAC * w

        # Vanishing point: where the two lane lines meet in the distance.
        # Perspective makes being off-center TILT the lane in the image, but
        # pointing at an angle SHIFTS the vanishing point sideways. So heading
        # comes from the vanishing point, and offset from the bottom row.
        if left and right and abs(left.slope - right.slope) > 0.05:
            y_vp = (right.intercept - left.intercept) / (left.slope - right.slope)
            if -h < y_vp < roi_top:
                # Remember the horizon row for frames with only one line.
                self._horizon = y_vp if self._horizon is None else \
                    0.9 * self._horizon + 0.1 * y_vp
        y_vp = self._horizon if self._horizon is not None else config.LANE_HORIZON_FRAC * h
        if left and right:
            x_vp = (left.x_at(y_vp) + right.x_at(y_vp)) / 2
            cx_bot = (left.x_at(y_bot) + right.x_at(y_bot)) / 2
        else:
            # One line: it still runs through the vanishing point, and the
            # lane center is half a lane width from it at the bottom.
            line = left or right
            x_vp = line.x_at(y_vp)
            cx_bot = line.x_at(y_bot) + (lane_px / 2 if left else -lane_px / 2)

        # Lane center left of the image center -> car is right of center.
        offset = (w / 2 - cx_bot) * config.LANE_WIDTH_M / lane_px
        # Vanishing point left of the image center -> car points right.
        heading = math.atan2(w / 2 - x_vp, self._focal_px(w))
        cx_look = cx_bot + (x_vp - cx_bot) * (y_bot - roi_top) / max(y_bot - y_vp, 1)

        info = LaneInfo(offset=offset, heading=heading,
                        left_lane_exists=left is not None,
                        right_lane_exists=right is not None)
        self.debug.update({"center_bot": cx_bot, "center_look": cx_look,
                           "y_look": roi_top, "x_vp": x_vp, "info": info})
        return info

    @staticmethod
    def _focal_px(width):
        return (width / 2) / math.tan(math.radians(config.CAMERA_HFOV_DEG) / 2)

    def _mask(self, hsv, ranges):
        cv2, np = self._cv2, self._np
        mask = None
        for lo, hi in ranges:
            part = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask

    def _fit(self, mask, roi_top):
        """Fit x = slope*y + b through the mask, one point per row band."""
        np = self._np
        if self._cv2.countNonZero(mask) < config.LANE_MIN_PIXELS:
            return None
        rows = mask.shape[0]
        bands = config.LANE_BANDS
        ys, xs = [], []
        for i in range(bands):
            y0 = rows * i // bands
            y1 = rows * (i + 1) // bands
            cols = np.nonzero(mask[y0:y1])[1]
            if len(cols) < config.LANE_MIN_BAND_PIXELS:
                continue
            ys.append(roi_top + (y0 + y1) / 2)
            xs.append(float(np.median(cols)))
        if len(ys) < 2:
            return None
        slope, intercept = np.polyfit(ys, xs, 1)
        return _Line(float(slope), float(intercept), int(min(ys)), int(max(ys)))

    def draw(self, frame):
        """Draw the last detection on a copy of the frame (for the video).

        Yellow/red tint = pixels that matched each color. Thick yellow/red
        lines = the fitted lane lines. Green = lane center. White tick =
        image center (where the car is pointing).
        """
        if frame is None or self._cv2 is None:
            return frame
        cv2, np = self._cv2, self._np
        out = frame.copy()
        h, w = out.shape[:2]
        d = self.debug
        if d is None:
            self._label(out, "NO LANE", (0, 0, 255))
            return out
        y_bot = h - 1
        top = d["roi_top"]

        # Tint the matched pixels so color tuning is easy to see.
        roi = out[top:]
        for mask, color in ((d["left_mask"], (0, 255, 255)), (d["right_mask"], (0, 0, 255))):
            roi[mask > 0] = (0.4 * roi[mask > 0] + 0.6 * np.array(color)).astype(np.uint8)

        cv2.line(out, (0, top), (w, top), (200, 200, 200), 1)
        for line, color in ((d["left"], (0, 255, 255)), (d["right"], (0, 0, 255))):
            if line:
                cv2.line(out, (int(line.x_at(top)), top),
                         (int(line.x_at(y_bot)), y_bot), color, 3)
        if "center_bot" in d:
            cv2.line(out, (int(d["center_look"]), d["y_look"]),
                     (int(d["center_bot"]), y_bot), (0, 255, 0), 2)
        cv2.line(out, (w // 2, y_bot - 20), (w // 2, y_bot), (255, 255, 255), 2)

        info = d["info"]
        if info is None:
            self._label(out, "NO LANE", (0, 0, 255))
        else:
            side = "R" if info.offset > 0 else "L"
            turn = "R" if info.heading > 0 else "L"
            l = "Y" if info.left_lane_exists else "-"
            r = "R" if info.right_lane_exists else "-"
            text = (f"off {abs(info.offset) * 100:.0f}cm {side}  "
                    f"head {abs(math.degrees(info.heading)):.0f} {turn}  [{l}|{r}]")
            self._label(out, text, (0, 255, 0))
        return out

    def _label(self, img, text, color):
        cv2 = self._cv2
        cv2.rectangle(img, (0, 0), (img.shape[1], 18), (0, 0, 0), -1)
        cv2.putText(img, text, (4, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1,
                    cv2.LINE_AA)

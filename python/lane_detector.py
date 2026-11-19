"""Camera frame -> where the car is in its lane. Uses the camera only.

The left lane line is red tape and the right lane line is yellow tape. Each
color is picked out in HSV, a straight line is fitted to each one in the
bottom part of the image, and the lane center is the midpoint between them.

A second line of the same color is a neighboring lane, not part of this one.
The fit keeps the pair followed on the previous frame. The other lines are
reported as a lane on the left or the right.
"""

import math
from dataclasses import dataclass

import config


@dataclass
class LaneInfo:
    offset: float    # meters from lane center (+ = car is right of center)
    heading: float   # radians between car and lane direction (+ = pointing right)
    left_lane_exists: bool    # the red (left) line was seen this frame
    right_lane_exists: bool   # the yellow (right) line was seen this frame
    neighbor_left: bool       # a full lane is visible on the left
    neighbor_right: bool      # a full lane is visible on the right


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
        # The pair followed last frame, so a second line of the same color
        # stays a neighbor instead of pulling the fit sideways.
        self._prev_left = None
        self._prev_right = None
        # The last frame's results, for drawing on the web page.
        self.debug = None

    @property
    def horizon_row(self):
        """Learned image row where the lane lines meet, or None before both are seen."""
        return self._horizon

    @property
    def lane_width_px(self):
        """Learned lane width on the bottom image row, or None before both are seen."""
        return self._lane_px

    def reset_track(self):
        """Forget which lines were followed, and lock onto the lane ahead."""
        self._prev_left = None
        self._prev_right = None

    def detect(self, frame, hue_offset=0):
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
        if hue_offset:
            hsv[:, :, 0] = (hsv[:, :, 0].astype(np.int16) + hue_offset) % 180
        left_mask = self._mask(hsv, config.LEFT_LINE_HSV_RANGES)     # red
        right_mask = self._mask(hsv, config.RIGHT_LINE_HSV_RANGES)   # yellow

        lefts = self._fit_lines(left_mask, roi_top, w)
        rights = self._fit_lines(right_mask, roi_top, w)

        y_bot = h - 1
        left, right = self._choose_pair(lefts, rights, y_bot, w)
        if left is None and right is None:
            # One color only: keep the line nearest the one followed last frame.
            left = self._nearest(lefts, self._prev_left, y_bot, w)
            right = self._nearest(rights, self._prev_right, y_bot, w)
        # Red has to be on the left of yellow. If it isn't, something else
        # red or yellow is in view, so trust neither line.
        if left and right and left.x_at(y_bot) >= right.x_at(y_bot):
            left = right = None
        left_extra = [line for line in lefts if line is not left]
        right_extra = [line for line in rights if line is not right]

        self.debug = {"roi_top": roi_top, "left": left, "right": right,
                      "left_extra": left_extra, "right_extra": right_extra,
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
        # Measure a neighbor against this pair's own width, not a full-frame guess.
        if left is not None and right is not None:
            pair_px = right.x_at(y_bot) - left.x_at(y_bot)
        else:
            pair_px = lane_px
        neighbor_left, neighbor_right = self._neighbors(
            left, right, left_extra, right_extra, y_bot, pair_px)

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

        if left is not None:
            self._prev_left = left
        if right is not None:
            self._prev_right = right
        info = LaneInfo(offset=offset, heading=heading,
                        left_lane_exists=left is not None,
                        right_lane_exists=right is not None,
                        neighbor_left=neighbor_left,
                        neighbor_right=neighbor_right)
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

    def _fit_lines(self, mask, roi_top, width):
        """Fit every separate line of this color.

        Pixels in a row band split into clusters when a gap is wide enough to
        be two tapes. Clusters are then joined from the bottom of the image
        upward into lines. The caller keeps the line nearest the one followed
        last frame and treats the others as a neighboring lane.
        """
        np = self._np
        if self._cv2.countNonZero(mask) < config.LANE_MIN_PIXELS:
            return []
        rows = mask.shape[0]
        bands = config.LANE_BANDS
        known = self._lane_px if self._lane_px is not None else 0.45 * width
        # Split two tapes, but keep one tape's pixels in a single cluster.
        # Link a tape to itself in the next row band, not across to the next lane.
        split_gap = max(12.0, 0.22 * known)
        link_gap = max(28.0, 0.55 * known)
        band_points = []
        for i in range(bands):
            y0 = rows * i // bands
            y1 = rows * (i + 1) // bands
            cols = np.nonzero(mask[y0:y1])[1]
            y = roi_top + (y0 + y1) / 2.0
            band_points.append(self._cluster_medians(cols, y, split_gap))

        # Bottom band first, so each tape is anchored where the car is.
        tracks = []
        for points in reversed(band_points):
            fresh = []
            for y, x in points:
                best = None
                best_d = link_gap
                for track in tracks:
                    dist = abs(x - track[-1][1])
                    if dist < best_d:
                        best, best_d = track, dist
                if best is None:
                    fresh.append([(y, x)])
                else:
                    best.append((y, x))
            tracks.extend(fresh)

        y_mid = roi_top + rows / 2.0
        lines = []
        for track in tracks:
            if len(track) < 2 or not any(y >= y_mid for y, _x in track):
                continue
            ys = [p[0] for p in track]
            xs = [p[1] for p in track]
            slope, intercept = np.polyfit(ys, xs, 1)
            lines.append(_Line(float(slope), float(intercept), int(min(ys)), int(max(ys))))
        return lines

    def _cluster_medians(self, cols, y, gap):
        """One (y, median x) per cluster of columns separated by more than gap."""
        np = self._np
        if len(cols) < config.LANE_MIN_BAND_PIXELS:
            return []
        cols = np.sort(cols)
        groups = []
        start = 0
        for i in range(1, len(cols)):
            if cols[i] - cols[i - 1] > gap:
                groups.append(cols[start:i])
                start = i
        groups.append(cols[start:])
        out = []
        for group in groups:
            if len(group) >= config.LANE_MIN_BAND_PIXELS:
                out.append((y, float(np.median(group))))
        return out

    def _choose_pair(self, lefts, rights, y_bot, width):
        """The red/yellow pair of the lane being followed.

        A pair has to be about one lane wide, with no other tape between
        the red and the yellow. Among those, stay on the pair nearest the
        one followed last frame.
        With no history, take the pair centered closest to the image center.
        """
        if not lefts or not rights:
            return None, None
        if self._lane_px is None:
            lo = config.LANE_MIN_WIDTH_FRAC * width
            hi = 0.95 * width
        else:
            lo = 0.5 * self._lane_px
            hi = 1.6 * self._lane_px
        pairs = []
        for left_line in lefts:
            for right_line in rights:
                gap = right_line.x_at(y_bot) - left_line.x_at(y_bot)
                if lo <= gap <= hi and not self._pair_blocked(
                        left_line, right_line, lefts, rights, y_bot):
                    pairs.append((left_line, right_line, gap))
        if not pairs:
            return None, None
        if self._prev_left is not None and self._prev_right is not None:
            def rank(pair):
                left_line, right_line, _gap = pair
                return (abs(left_line.x_at(y_bot) - self._prev_left.x_at(y_bot))
                        + abs(right_line.x_at(y_bot) - self._prev_right.x_at(y_bot)))
        else:
            def rank(pair):
                left_line, right_line, _gap = pair
                mid = (left_line.x_at(y_bot) + right_line.x_at(y_bot)) / 2.0
                return abs(mid - width / 2.0)
        left_line, right_line, _gap = min(pairs, key=rank)
        return left_line, right_line

    @staticmethod
    def _pair_blocked(left_line, right_line, lefts, rights, y_bot):
        """True when some other tape sits between this red and this yellow."""
        left = left_line.x_at(y_bot)
        right = right_line.x_at(y_bot)
        for line in lefts:
            if line is not left_line and left < line.x_at(y_bot) < right:
                return True
        for line in rights:
            if line is not right_line and left < line.x_at(y_bot) < right:
                return True
        return False

    def _nearest(self, lines, previous, y_bot, width):
        if not lines:
            return None
        if previous is not None:
            return min(lines, key=lambda line: abs(line.x_at(y_bot) - previous.x_at(y_bot)))
        return min(lines, key=lambda line: abs(line.x_at(y_bot) - width / 2.0))

    def _neighbors(self, left, right, left_extra, right_extra, y_bot, lane_px):
        """A neighbor lane is the other color just outside this lane, plus
        its far line about one lane width further out."""
        neighbor_left = False
        neighbor_right = False
        if left is not None:
            lx = left.x_at(y_bot)
            near_other = any(lx - 0.55 * lane_px < line.x_at(y_bot) < lx - 0.02 * lane_px
                           for line in right_extra)
            far_same = any(lx - 1.5 * lane_px < line.x_at(y_bot) < lx - 0.65 * lane_px
                             for line in left_extra)
            neighbor_left = near_other and far_same
        if right is not None:
            rx = right.x_at(y_bot)
            near_other = any(rx + 0.02 * lane_px < line.x_at(y_bot) < rx + 0.55 * lane_px
                              for line in left_extra)
            far_same = any(rx + 0.65 * lane_px < line.x_at(y_bot) < rx + 1.5 * lane_px
                          for line in right_extra)
            neighbor_right = near_other and far_same
        return neighbor_left, neighbor_right

    def draw(self, frame):
        """Draw the last detection on a copy of the frame (for the video).

        Red/yellow tint = pixels that matched each color. Thick red/yellow
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
        for mask, color in ((d["left_mask"], (0, 0, 255)), (d["right_mask"], (0, 255, 255))):
            roi[mask > 0] = (0.4 * roi[mask > 0] + 0.6 * np.array(color)).astype(np.uint8)

        cv2.line(out, (0, top), (w, top), (200, 200, 200), 1)
        for line in d.get("left_extra", []):
            cv2.line(out, (int(line.x_at(top)), top),
                     (int(line.x_at(y_bot)), y_bot), (80, 80, 180), 1)
        for line in d.get("right_extra", []):
            cv2.line(out, (int(line.x_at(top)), top),
                     (int(line.x_at(y_bot)), y_bot), (0, 180, 180), 1)
        for line, color in ((d["left"], (0, 0, 255)), (d["right"], (0, 255, 255))):
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
            l = "R" if info.left_lane_exists else "-"
            r = "Y" if info.right_lane_exists else "-"
            nb = (" <L" if info.neighbor_left else "") + (" R>" if info.neighbor_right else "")
            text = (f"off {abs(info.offset) * 100:.0f}cm {side}  "
                    f"head {abs(math.degrees(info.heading)):.0f} {turn}  [{l}|{r}]{nb}")
            self._label(out, text, (0, 255, 0))
        return out

    def _label(self, img, text, color):
        cv2 = self._cv2
        cv2.rectangle(img, (0, 0), (img.shape[1], 18), (0, 0, 0), -1)
        cv2.putText(img, text, (4, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1,
                    cv2.LINE_AA)

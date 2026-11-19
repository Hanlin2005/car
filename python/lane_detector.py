"""Camera frame -> where the car is in its lane. Uses the camera only.

Three lanes on the course:

  RED |-- lane 0 --| YELLOW |-- lane 1 --| YELLOW |-- lane 2 --| RED

Lane 0 is red on the left and yellow on the right. Lane 1 is yellow on both
sides. Lane 2 is yellow on the left and red on the right. So the colors of
the pair being followed say which lane the car is in. Each color is picked
out in HSV, straight lines are fitted in the bottom of the image, and the
lane center is the midpoint of the pair being followed.

The fit keeps the pair from the previous frame. When that pair drifts far
off-center (lane change), it re-locks onto the pair nearest the image center.
"""

import math
from dataclasses import dataclass

import birdseye
import config


@dataclass
class LaneInfo:
    offset: float    # meters from lane center (+ = car is right of center)
    heading: float   # radians between car and lane direction (+ = pointing right)
    left_lane_exists: bool    # left boundary of this lane was seen this frame
    right_lane_exists: bool   # right boundary of this lane was seen this frame
    neighbor_left: bool       # a full lane is visible on the left
    neighbor_right: bool      # a full lane is visible on the right
    lane_index: int           # 0 red|yellow, 1 yellow|yellow, 2 yellow|red
    lane_confirmed: bool      # at least two lines agree on the lane (or both
                              # boundaries seen), not carried over
    lane_pos: float = None    # car's sideways position in lane widths from
                              # the left red line (0..3), from the floor fit


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
        # Which lane the followed pair belongs to, by its colors:
        # 0 = red|yellow, 1 = yellow|yellow, 2 = yellow|red.
        self._lane = config.START_LANE
        # Sideways position in lane widths from the left red line, from the
        # floor fit. Used to keep the lane steady between frames.
        self._pos = config.START_LANE + 0.5
        # Frames in a row where lines were seen but the floor fit failed.
        # Many in a row means the learned width is wrong: forget it.
        self._floor_misses = 0
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
        """Forget which lines were followed, and lock onto the lane ahead.

        Also forgets the learned lane width and horizon: a run that spun or
        crossed the lines at an angle may have taught it nonsense.
        """
        self._prev_left = None
        self._prev_right = None
        self._lane = config.START_LANE
        self._pos = config.START_LANE + 0.5
        self._lane_px = None
        self._horizon = None
        self._floor_misses = 0

    @property
    def lane_index(self):
        """Lane the detector believes it is following (0, 1 or 2)."""
        return self._lane

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
        red_mask = self._mask(hsv, config.LEFT_LINE_HSV_RANGES)      # red
        yellow_mask = self._mask(hsv, config.RIGHT_LINE_HSV_RANGES)  # yellow

        reds = self._fit_lines(red_mask, roi_top, w)
        yellows = self._fit_lines(yellow_mask, roi_top, w)

        y_bot = h - 1
        left, right, lane_kind = self._choose_pair(reds, yellows, y_bot, w)
        confirmed = left is not None and right is not None
        if left is None and right is None:
            left, right, lane_kind = self._fallback_lines(
                reds, yellows, y_bot, w)
        if left and right and left.x_at(y_bot) >= right.x_at(y_bot):
            left = right = None
            confirmed = False

        # On the floor, the lines' sideways distances from the car say which
        # lane it is in whatever way the car is pointing. The image-row pair
        # above is only a fallback for when the floor mapping is unusable.
        floor = self._floor_lines(reds, yellows, w, h, roi_top)
        located = self._locate(floor) if floor else None
        pos = None
        if located is not None:
            left, right, lane_kind, pos, confirmed, slots = located
            straight = all(abs(phi) < 0.2 for _l, _c, _u, phi in floor)
            self._floor_misses = 0
        else:
            # No floor fit. Judge "pointing along the lines" from the image:
            # a straight-on pair is left/right symmetric about the center.
            straight = (left is not None and right is not None and
                        abs(left.slope + right.slope) < 0.3 * abs(left.slope - right.slope))
            if len(reds) + len(yellows) >= 2 and self._lane_px is not None:
                self._floor_misses += 1
                if self._floor_misses > 30:
                    # Two or more lines in view that the floor model cannot
                    # place, for a second: the learned width is wrong.
                    # Start learning again.
                    self._lane_px = None
                    self._horizon = None
                    self._floor_misses = 0

        self.debug = {"roi_top": roi_top, "left": left, "right": right,
                      "lane": lane_kind, "confirmed": confirmed, "pos": pos,
                      "left_extra": [line for line in reds if line is not left],
                      "right_extra": [line for line in yellows
                                      if line is not left and line is not right],
                      "left_mask": red_mask, "right_mask": yellow_mask,
                      "info": None}
        if left is None and right is None:
            return None

        # Lane width in pixels at the bottom row. Only meaningful when the
        # car points along the lines; at an angle the row cuts them obliquely.
        if left and right and straight:
            width = right.x_at(y_bot) - left.x_at(y_bot)
            if width > config.LANE_MIN_WIDTH_FRAC * w:
                self._lane_px = width if self._lane_px is None else \
                    0.8 * self._lane_px + 0.2 * width
        lane_px = self._lane_px or config.LANE_DEFAULT_WIDTH_FRAC * w
        if left is not None and right is not None:
            pair_px = right.x_at(y_bot) - left.x_at(y_bot)
        else:
            pair_px = lane_px
        if located is not None:
            # A neighbor lane is in view when its far boundary was placed.
            neighbor_left = (lane_kind - 1) in slots
            neighbor_right = (lane_kind + 2) in slots
        else:
            neighbor_left, neighbor_right = self._neighbors(
                left, right, reds, yellows, y_bot, pair_px, lane_kind)

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
        if confirmed or located is not None:
            # The floor fit decides the lane (its hysteresis stops flip-flop
            # on a line). Updating only on two-line confirmation left the
            # one-line fallback answering with the OLD lane after a change,
            # so the offset alternated between two lanes' values.
            self._lane = lane_kind
        if pos is not None:
            self._pos = pos
        info = LaneInfo(offset=offset, heading=heading,
                        left_lane_exists=left is not None,
                        right_lane_exists=right is not None,
                        neighbor_left=neighbor_left,
                        neighbor_right=neighbor_right,
                        lane_index=lane_kind,
                        lane_confirmed=confirmed,
                        lane_pos=pos)
        self.debug.update({"center_bot": cx_bot, "center_look": cx_look,
                           "y_look": roi_top, "x_vp": x_vp, "info": info})
        return info

    # ---- Floor-based lane localization --------------------------------

    def _floor_lines(self, reds, yellows, w, h, roi_top):
        """Each fitted line as (line, color, u, phi) on the floor.

        u is the line's sideways distance from the car in lane widths
        (+ = right), measured perpendicular to the line, so it does not
        change with the car's heading. phi is the line's direction relative
        to the car (+ = it runs off to the right). None if the camera pose
        is unusable (horizon too low) or no line maps to the floor.
        """
        horizon = self._horizon if self._horizon is not None else config.LANE_HORIZON_FRAC * h
        lane_px = self._lane_px or config.LANE_DEFAULT_WIDTH_FRAC * w
        pose = birdseye.camera_pose(w, h, horizon, lane_px)
        if pose is None:
            return None
        y_bot = h - 1
        # Second sample well below the horizon, where the mapping is stable.
        y_far = max(roi_top + 0.2 * (y_bot - roi_top), horizon + 0.12 * (y_bot - horizon))
        out = []
        for lines, color in ((reds, "red"), (yellows, "yellow")):
            for line in lines:
                near = self._to_floor(line.x_at(y_bot), y_bot, w, h, pose)
                far = self._to_floor(line.x_at(y_far), y_far, w, h, pose)
                if near is None or far is None:
                    continue
                dx, dz = far[0] - near[0], far[1] - near[1]
                norm = math.hypot(dx, dz)
                if norm < 1e-6:
                    continue
                dx, dz = dx / norm, dz / norm
                if dz < 0:
                    dx, dz = -dx, -dz
                # Signed perpendicular distance from the reference point
                # (the camera, or LANE_REF_BEHIND_CAMERA_M behind it).
                d = near[0] * dz - near[1] * dx
                d -= config.LANE_REF_BEHIND_CAMERA_M * dx
                out.append((line, color, d / config.LANE_WIDTH_M, math.atan2(dx, dz)))
        return out or None

    @staticmethod
    def _to_floor(u, v, w, h, pose):
        """Image pixel -> floor point (meters right, meters ahead of the camera)."""
        focal, tilt, cam_h = pose
        a = (u - w / 2) / focal
        b = (v - h / 2) / focal
        s, c = math.sin(tilt), math.cos(tilt)
        down = b * c + s          # ray's downward component per unit forward
        if down <= 1e-3:
            return None           # at or above the horizon
        k = cam_h / down
        return a * k, (c - b * s) * k

    _SLOTS = {"red": (0, 3), "yellow": (1, 2)}   # tape index from the left red

    def _locate(self, floor):
        """Fit the floor lines to RED|YEL|YEL|RED.

        Tape k sits at k lane widths from the left red. A line of color c at
        sideways distance u puts the car at p = k - u for each slot k of that
        color. The p with the most agreeing lines wins; ties go to the p
        closest to last frame's. Returns (left, right, lane, p, confirmed,
        slots) or None.
        """
        tol = config.LANE_LOCATE_TOL
        count = config.LANE_COUNT
        candidates = []
        for line, color, u, _phi in floor:
            for k in self._SLOTS[color]:
                p = k - u
                if -0.5 <= p <= count + 0.5:
                    candidates.append(p)
        if not candidates:
            return None

        best = None
        for p0 in candidates:
            slots = {}
            for line, color, u, phi in floor:
                k = min(self._SLOTS[color], key=lambda kk: abs(kk - u - p0))
                err = abs(k - u - p0)
                if err <= tol and (k not in slots or slots[k][1] > err):
                    slots[k] = (line, err, u)
            if not slots:
                continue
            p = sum(k - u for k, (_l, _e, u) in slots.items()) / len(slots)
            # Most lines agreeing wins; then a position on the course beats
            # one beside it (a lone red could be either edge); then the
            # position nearest last frame's; then the tightest fit.
            on_course = -0.15 <= p <= count + 0.15
            score = (len(slots), on_course, -abs(p - self._pos),
                     -sum(e for _l, e, _u in slots.values()))
            if best is None or score > best[0]:
                best = (score, p, slots)
        if best is None:
            return None
        _score, p, slots = best
        lane = int(min(count - 1, max(0, math.floor(p))))
        # Hysteresis: stay in the last lane until the car is clearly past
        # its line, so the answer does not flip-flop while crossing.
        hyst = config.LANE_SWITCH_HYST
        prev = self._lane
        if lane != prev and (prev - hyst) <= p <= (prev + 1 + hyst):
            lane = prev
        placed = {k: v[0] for k, v in slots.items()}
        left = placed.get(lane)
        right = placed.get(lane + 1)
        if left is None and right is None:
            return None       # placed only a far line: no boundary to follow
        confirmed = len(placed) >= 2
        return left, right, lane, p, confirmed, placed

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

    def _width_bounds(self, width):
        if self._lane_px is None:
            return config.LANE_MIN_WIDTH_FRAC * width, 0.95 * width
        return 0.5 * self._lane_px, 1.6 * self._lane_px

    def _choose_pair(self, reds, yellows, y_bot, width):
        """Pick the lane pair being followed.

        Lane 0: red left + yellow right. Lane 1: yellow + yellow.
        Lane 2: yellow left + red right. A pair has to be about one lane
        wide with no other tape between.
        """
        lo, hi = self._width_bounds(width)
        all_lines = list(reds) + list(yellows)
        pairs = []  # (left, right, lane_index)

        for red in reds:
            for yel in yellows:
                gap = yel.x_at(y_bot) - red.x_at(y_bot)
                if gap >= 0:
                    kind = 0           # red on the left
                else:
                    gap = -gap
                    kind = 2           # red on the right
                left_line, right_line = (red, yel) if kind == 0 else (yel, red)
                if lo <= gap <= hi and not self._pair_blocked(
                        left_line, right_line, all_lines, y_bot):
                    pairs.append((left_line, right_line, kind))

        for i, a in enumerate(yellows):
            for b in yellows[i + 1:]:
                y_left, y_right = ((a, b) if a.x_at(y_bot) <= b.x_at(y_bot)
                                   else (b, a))
                gap = y_right.x_at(y_bot) - y_left.x_at(y_bot)
                if lo <= gap <= hi and not self._pair_blocked(
                        y_left, y_right, all_lines, y_bot):
                    pairs.append((y_left, y_right, 1))

        if not pairs:
            return None, None, self._lane

        def center_dist(pair):
            left_line, right_line, _ = pair
            mid = (left_line.x_at(y_bot) + right_line.x_at(y_bot)) / 2.0
            return abs(mid - width / 2.0)

        lane_px = self._lane_px if self._lane_px is not None else 0.45 * width
        if self._prev_left is not None and self._prev_right is not None:
            prev_mid = (self._prev_left.x_at(y_bot) + self._prev_right.x_at(y_bot)) / 2.0
            # During a lane change the old pair drifts off-center. Once the
            # car is past that lane's edge tape (more than half a lane from
            # its center), re-lock onto whichever valid pair is nearest the
            # image center, so a change through the middle lane sees it.
            if abs(prev_mid - width / 2.0) > 0.6 * lane_px:
                left_line, right_line, lane_kind = min(pairs, key=center_dist)
            else:
                def rank(pair):
                    left_line, right_line, _ = pair
                    return (abs(left_line.x_at(y_bot) - self._prev_left.x_at(y_bot))
                            + abs(right_line.x_at(y_bot) - self._prev_right.x_at(y_bot)))
                left_line, right_line, lane_kind = min(pairs, key=rank)
        else:
            # No history: take the lane the car is actually in (the pair
            # around the image center). START_LANE only breaks a tie when
            # the car sits between two lanes at startup.
            centered = [p for p in pairs if center_dist(p) < 0.5 * lane_px]
            preferred = ([p for p in centered if p[2] == config.START_LANE]
                         or centered or pairs)
            left_line, right_line, lane_kind = min(preferred, key=center_dist)
        return left_line, right_line, lane_kind

    @staticmethod
    def _pair_blocked(left_line, right_line, lines, y_bot):
        """True when some other tape sits between these two boundaries."""
        left = left_line.x_at(y_bot)
        right = right_line.x_at(y_bot)
        for line in lines:
            if line is left_line or line is right_line:
                continue
            if left < line.x_at(y_bot) < right:
                return True
        return False

    def _fallback_lines(self, reds, yellows, y_bot, width):
        """One-sided detection when no full-width pair is visible.

        Uses the lane followed last, since one line alone does not say
        which lane it belongs to.
        """
        lane = self._lane
        if lane == 0:
            left = self._nearest(reds, self._prev_left, y_bot, width)
            right = self._nearest(yellows, self._prev_right, y_bot, width)
        elif lane == 2:
            left = self._nearest(yellows, self._prev_left, y_bot, width)
            right = self._nearest(reds, self._prev_right, y_bot, width)
        else:
            # Middle lane: both boundaries are yellow.
            left = self._nearest(yellows, self._prev_left, y_bot, width)
            others = [line for line in yellows if line is not left]
            right = self._nearest(others, self._prev_right, y_bot, width)
        return left, right, lane

    def _nearest(self, lines, previous, y_bot, width):
        if not lines:
            return None
        if previous is not None:
            return min(lines, key=lambda line: abs(line.x_at(y_bot) - previous.x_at(y_bot)))
        return min(lines, key=lambda line: abs(line.x_at(y_bot) - width / 2.0))

    def _neighbors(self, left, right, reds, yellows, y_bot, lane_px, lane):
        """Neighbor lanes for RED|YELLOW|YELLOW|RED.

        A neighbor exists when that lane's far boundary is seen about one
        lane width beyond the pair. Lane 0: a yellow to the right. Lane 1:
        a red on either side. Lane 2: a yellow to the left.
        """
        if left is None or right is None:
            return False, False
        lx = left.x_at(y_bot)
        rx = right.x_at(y_bot)

        def beyond_right(lines):
            return any(rx + 0.65 * lane_px < line.x_at(y_bot) < rx + 1.5 * lane_px
                       for line in lines if line is not right)

        def beyond_left(lines):
            return any(lx - 1.5 * lane_px < line.x_at(y_bot) < lx - 0.65 * lane_px
                       for line in lines if line is not left)

        if lane == 0:
            return False, beyond_right(yellows)
        if lane == 2:
            return beyond_left(yellows), False
        return beyond_left(reds), beyond_right(reds)

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
        lane = d.get("lane", 0)
        red, yellow = (0, 0, 255), (0, 255, 255)
        left_color = red if lane == 0 else yellow
        right_color = red if lane == 2 else yellow
        for line, color in ((d["left"], left_color), (d["right"], right_color)):
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
            l_col = "R" if lane == 0 else "Y"
            r_col = "R" if lane == 2 else "Y"
            l = l_col if info.left_lane_exists else "-"
            r = r_col if info.right_lane_exists else "-"
            kind = f"L{lane}" + ("" if info.lane_confirmed else "?")
            if info.lane_pos is not None:
                kind += f" p{info.lane_pos:.1f}"
            nb = (" <L" if info.neighbor_left else "") + (" R>" if info.neighbor_right else "")
            text = (f"{kind} off {abs(info.offset) * 100:.0f}cm {side}  "
                    f"head {abs(math.degrees(info.heading)):.0f} {turn}  [{l}|{r}]{nb}")
            self._label(out, text, (0, 255, 0))
        return out

    def _label(self, img, text, color):
        cv2 = self._cv2
        cv2.rectangle(img, (0, 0), (img.shape[1], 18), (0, 0, 0), -1)
        cv2.putText(img, text, (4, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1,
                    cv2.LINE_AA)

"""2D pose of the car in the lane frame, from wheel encoders + camera fixes.

Lane frame (matches lane_detector / controller):
  x      meters right of the lane center
  s      meters traveled along the lane
  theta  radians; 0 = pointed along the lane, + = nose pointed right of the lane

Encoders propagate the pose between camera frames. A good lane reading nudges
x and theta back toward what the camera sees so drift does not run away.
"""

import math

import config


class LanePose:
    def __init__(self):
        self.reset()

    def reset(self):
        self.x = 0.0
        self.s = 0.0
        self.theta = 0.0
        self.locked = False
        self._prev_left = None
        self._prev_right = None

    def capture_baseline(self, telemetry):
        """Remember encoder distances without moving the pose (before the first lock)."""
        try:
            self._prev_left = float(telemetry["left_dist"])
            self._prev_right = float(telemetry["right_dist"])
        except (KeyError, TypeError, ValueError):
            pass

    def predict(self, telemetry):
        """Advance the pose using encoder distance since the last call."""
        try:
            left = float(telemetry["left_dist"])
            right = float(telemetry["right_dist"])
        except (KeyError, TypeError, ValueError):
            return
        if self._prev_left is None:
            self._prev_left, self._prev_right = left, right
            return
        dl = left - self._prev_left
        dr = right - self._prev_right
        self._prev_left, self._prev_right = left, right
        # Telemetry reset or glitch.
        if abs(dl) > 0.5 or abs(dr) > 0.5:
            return
        ds = 0.5 * (dl + dr)
        # Vision heading +: nose right. Right wheel farther => turn left => theta down.
        dtheta = (dl - dr) / max(config.WHEEL_BASE_M, 1e-3)
        mid = self.theta + 0.5 * dtheta
        self.x += ds * math.sin(mid)
        self.s += ds * math.cos(mid)
        self.theta += dtheta

    def lock(self, offset, heading):
        """Snap pose to a camera reading (first fix or lane-change relock)."""
        self.x = float(offset)
        self.theta = float(heading)
        self.locked = True

    def correct(self, offset, heading, blend):
        """Pull pose toward the camera. blend 1 = trust camera fully, 0 = ignore it."""
        blend = max(0.0, min(1.0, float(blend)))
        self.x = (1.0 - blend) * self.x + blend * float(offset)
        self.theta = (1.0 - blend) * self.theta + blend * float(heading)
        self.locked = True

"""Turns 'go this fast, stay on the lane' into left/right wheel speeds.

LaneFollower tracks a straight lane-center path in 2D:

  1. Wheel encoders update a lane-frame pose (x = right of center, theta =
     heading). See odometry.py.
  2. A good camera reading nudges that pose toward the measured offset and
     heading (not a full rewrite every frame).
  3. Pure pursuit aims PURE_PURSUIT_LOOKAHEAD_M ahead on the path x = target
     (usually the lane center). Steering uses the fused pose, so between
     camera frames the car still follows the line instead of twitching on
     every noisy image.
  4. If the camera is gone longer than LANE_LOST_TIMEOUT_S, stop. Brief gaps
     keep following on odometry alone.

VISION_CONTROL off drives straight. The page sets that and VISION_GAIN.
"""

import math
import time

import config
from odometry import LanePose


def _clamp(x, limit):
    return max(-limit, min(limit, x))


def _pure_pursuit_omega(offset, heading, target_offset, speed):
    """Turn rate (rad/s) to arc toward the path x = target_offset.

    Vehicle frame: x right, z forward. omega > 0 turns left (right wheel faster).
    """
    err = offset - target_offset
    dead = getattr(config, "PURE_PURSUIT_DEADZONE_M", 0.0)
    if abs(err) < dead:
        err = 0.0
    ld = config.PURE_PURSUIT_LOOKAHEAD_M
    cos_h = math.cos(heading)
    sin_h = math.sin(heading)
    # Path point (target is already removed via err) at (0, ld) in path frame
    # relative to the car at (err, 0).
    x = -err * cos_h - ld * sin_h
    z = -err * sin_h + ld * cos_h
    dist2 = x * x + z * z
    if dist2 < 1e-6:
        return 0.0
    kappa = 2.0 * x / dist2
    return -config.VISION_GAIN * speed * kappa


class LaneFollower:
    def __init__(self):
        self.reset()

    def reset(self):
        self.pose = LanePose()
        self._omega = 0.0
        self._last_seen = None
        self._last_time = None
        self._rejects = 0
        self.rejected = 0
        self.lane_lost = False
        self.omega = 0.0
        # Last camera values (for the log / overlay text).
        self._offset = None
        self._heading = None
        # Filtered rate of change of the fused heading, for damping.
        self._prev_theta = None
        self._theta_rate = 0.0

    def update(self, lane, speed, target_offset, telemetry=None, now=None, relock=False):
        """Return (left, right) wheel speeds in m/s.

        telemetry is car_link.telemetry (encoder distances). relock clears the
        pose so the next good camera frame re-locks the lane path.
        """
        now = time.monotonic() if now is None else now
        dt = 0.05 if self._last_time is None else min(max(now - self._last_time, 0.001), 0.5)
        self._last_time = now

        if speed == 0.0:
            self.reset()
            self._last_time = now
            return 0.0, 0.0

        if relock:
            self.pose.reset()
            self._rejects = 0
            self._offset = None
            self._heading = None

        if telemetry is not None:
            if self.pose.locked:
                self.pose.predict(telemetry)
            else:
                # Seed encoder baselines so the first lock does not get a huge Δ.
                self.pose.capture_baseline(telemetry)

        if not config.VISION_CONTROL:
            self._omega = self.omega = 0.0
            self.lane_lost = False
            return speed, speed

        if lane is not None:
            heading = _clamp(lane.heading, config.LANE_MAX_HEADING)
            accept = True
            if self._offset is not None and \
                    abs(lane.offset - self._offset) > config.LANE_JUMP_M:
                self._rejects += 1
                if self._rejects < config.LANE_JUMP_CONFIRM:
                    accept = False
                    self.rejected += 1
                else:
                    self._offset = None
            if accept:
                self._rejects = 0
                self._offset = lane.offset
                self._heading = heading
                both = lane.left_lane_exists and lane.right_lane_exists
                blend = config.ODOM_CAMERA_BLEND if both else 0.5 * config.ODOM_CAMERA_BLEND
                if not self.pose.locked:
                    self.pose.lock(lane.offset, heading)
                else:
                    self.pose.correct(lane.offset, heading, blend)
            self._last_seen = now
            self.lane_lost = False

        cam_age = None if self._last_seen is None else now - self._last_seen
        if not self.pose.locked or cam_age is None or \
                cam_age > config.ODOM_BLIND_TIMEOUT_S:
            self.lane_lost = True
            self._omega = self.omega = 0.0
            return 0.0, 0.0

        # Follow x = target_offset using the fused pose (encoders bridge camera gaps).
        self.lane_lost = cam_age > config.LANE_LOST_TIMEOUT_S
        target_omega = _pure_pursuit_omega(
            self.pose.x, self.pose.theta, target_offset, speed)
        # Damping: oppose the heading's rate of change. theta + is nose right;
        # omega + turns left (theta down). A swing to the right (rate > 0)
        # gets a left turn added, which also softens the controller's own turns.
        if self._prev_theta is not None:
            raw_rate = (self.pose.theta - self._prev_theta) / dt
            a = config.HEADING_RATE_FILTER
            self._theta_rate = (1.0 - a) * self._theta_rate + a * raw_rate
            target_omega += config.HEADING_DAMPING * self._theta_rate
        self._prev_theta = self.pose.theta
        target_omega = _clamp(target_omega, config.MAX_OMEGA)
        step = config.MAX_OMEGA_RATE * dt
        self._omega += _clamp(target_omega - self._omega, step)
        self.omega = self._omega

        turn = abs(self._omega) / max(config.MAX_OMEGA, 1e-3)
        v = speed * (1.0 - config.TURN_SLOWDOWN * turn)

        half_base = config.WHEEL_BASE_M / 2
        return v - self._omega * half_base, v + self._omega * half_base


_default = LaneFollower()


def compute_wheel_speeds(lane, speed, target_offset):
    """Return (left, right) wheel speeds in m/s. Kept for older callers."""
    return _default.update(lane, speed, target_offset)

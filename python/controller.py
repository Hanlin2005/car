"""Turns 'go this fast, stay at this offset' into left/right wheel speeds.

LaneFollower keeps the car driving straight down the middle of the lane:

  1. Throw away readings that can't be real: a heading beyond
     LANE_MAX_HEADING, or an offset that jumped more than LANE_JUMP_M since
     the last frame (the car can't move sideways that fast). If the new value
     keeps showing up for LANE_JUMP_CONFIRM frames, believe it.
     Then smooth what's left (one noisy frame shouldn't jerk the wheels).
  2. Turn rate = VISION_GAIN * (KP_OFFSET * how far off center + KP_HEADING *
     how far the car points away from the lane). The heading term is the
     damping: it starts straightening the car out before it reaches the
     center, so it doesn't overshoot and weave. VISION_CONTROL off skips
     this and drives straight. The page sets both while a run is going.
  3. Limit how hard and how suddenly it can turn, and slow down while turning.
  4. If the lane disappears for a moment, keep driving straight (the wheel
     speed controller holds both wheels equal). If it stays gone longer than
     LANE_LOST_TIMEOUT_S, stop.

The per-wheel speed control (encoders + PID) is in hardware.py, so "equal
wheel speeds" really does mean straight even though the two motors differ.
"""

import time

import config


def _clamp(x, limit):
    return max(-limit, min(limit, x))


class LaneFollower:
    def __init__(self):
        self.reset()

    def reset(self):
        self._offset = None
        self._heading = None
        self._omega = 0.0
        self._last_seen = None
        self._last_time = None
        self._rejects = 0
        self.rejected = 0         # bad readings thrown away (for printing)
        self.lane_lost = False
        self.omega = 0.0          # last turn rate (rad/s), for printing

    def update(self, lane, speed, target_offset, now=None):
        """Return (left, right) wheel speeds in m/s."""
        now = time.monotonic() if now is None else now
        dt = 0.05 if self._last_time is None else min(max(now - self._last_time, 0.001), 0.5)
        self._last_time = now

        if speed == 0.0:
            self.reset()
            self._last_time = now
            return 0.0, 0.0

        if not config.VISION_CONTROL:
            self._omega = self.omega = 0.0
            self.lane_lost = False
            return speed, speed

        if lane is not None and abs(lane.heading) > config.LANE_MAX_HEADING:
            lane = None           # no real lane-following angle is that big
            self.rejected += 1
        if lane is not None and self._offset is not None and \
                abs(lane.offset - self._offset) > config.LANE_JUMP_M:
            self._rejects += 1
            if self._rejects < config.LANE_JUMP_CONFIRM:
                lane = None       # sudden jump: skip it unless it persists
                self.rejected += 1
            else:
                self._offset = None   # it persisted: start over from it
        if lane is not None:
            self._rejects = 0
            a = config.LANE_FILTER
            if self._offset is None:
                self._offset, self._heading = lane.offset, lane.heading
            else:
                self._offset = a * lane.offset + (1 - a) * self._offset
                self._heading = a * lane.heading + (1 - a) * self._heading
            self._last_seen = now
            self.lane_lost = False
            # omega > 0 turns left (right wheel faster). Car right of center
            # or pointing right -> positive error/heading -> turn left.
            error = self._offset - target_offset
            target_omega = config.VISION_GAIN * (
                config.KP_OFFSET * error + config.KP_HEADING * self._heading)
        else:
            if self._last_seen is None or now - self._last_seen > config.LANE_LOST_TIMEOUT_S:
                self.lane_lost = True
                self._omega = self.omega = 0.0
                return 0.0, 0.0
            target_omega = 0.0    # lane briefly gone: go straight

        target_omega = _clamp(target_omega, config.MAX_OMEGA)
        # Don't snap the steering: change the turn rate gradually.
        step = config.MAX_OMEGA_RATE * dt
        self._omega += _clamp(target_omega - self._omega, step)
        self.omega = self._omega

        # Slow down in proportion to how hard it's turning.
        turn = abs(self._omega) / config.MAX_OMEGA
        v = speed * (1.0 - config.TURN_SLOWDOWN * turn)

        # Differential drive: v - w*b/2 for left, v + w*b/2 for right
        half_base = config.WHEEL_BASE_M / 2
        return v - self._omega * half_base, v + self._omega * half_base


_default = LaneFollower()


def compute_wheel_speeds(lane, speed, target_offset):
    """Return (left, right) wheel speeds in m/s. Kept for older callers."""
    return _default.update(lane, speed, target_offset)

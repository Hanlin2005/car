"""Decides what the car should do: combines lane info, lidar and wheel telemetry.

Lane changes come from the Left and Right buttons, not from an obstacle.
CHANGE_LANE has two phases, both at the slow speed. Shift aims one lane
width off the lane the car is leaving. Relock starts when the measured
center jumps onto the new pair: the target goes back to 0 and the follower
is told to drop its smoothed offset. Once the new lane stays centered, the
car follows it and keeps the new lane index.

The lane index also follows the camera: when both boundaries of the lane
are seen, their colors say which lane it is (see lane_detector), and that
overrides the count kept here.
"""

import time

import config

IDLE = "IDLE"
FOLLOW_LANE = "FOLLOW_LANE"
SLOW = "SLOW"
CHANGE_LANE = "CHANGE_LANE"
STOP = "STOP"


class Planner:
    def __init__(self):
        self.state = IDLE
        self.change_direction = None   # "left" or "right" during CHANGE_LANE
        self.phase = None              # "shift" or "settle" during CHANGE_LANE
        self.lane_index = config.START_LANE
        self.left_available = False
        self.right_available = False
        self.relock = False            # follower should drop its smoothed offset
        self._request = None
        self._phase_t0 = 0.0
        self._settle_since = None
        self._last_offset = None
        self._hold = False             # a lane change that did not finish: stay stopped

    def start(self):
        self.state = FOLLOW_LANE
        self.lane_index = config.START_LANE
        self.change_direction = None
        self.phase = None
        self._hold = False
        self.relock = False
        self._last_offset = None
        self._settle_since = None

    def request(self, direction):
        """Ask to change lanes. Ignored unless that side is available."""
        if direction in ("left", "right"):
            self._request = direction

    def update(self, lane, obstacles, telemetry):
        """Return (state, target_speed, target_offset) for the controller."""
        now = time.monotonic()
        self.relock = False
        if (lane is not None and lane.lane_confirmed
                and self.state != CHANGE_LANE
                and 0 <= lane.lane_index < config.LANE_COUNT):
            self.lane_index = lane.lane_index
        self._set_available(lane, obstacles)

        if self._hold:
            self.state = STOP
            return self.state, 0.0, 0.0

        if self.state == IDLE:
            return self.state, 0.0, 0.0

        # Safety first. A missing lane is handled by LaneFollower: it drives
        # straight through short gaps and stops after LANE_LOST_TIMEOUT_S.
        if telemetry.get("estop") or obstacles.front < config.STOP_DISTANCE:
            self.state = STOP
            self.phase = None
            self.change_direction = None
            self._last_offset = None
            return self.state, 0.0, 0.0

        if self.state == STOP:
            # The obstacle check above already returned if it is still blocked.
            self.state = FOLLOW_LANE

        if self.state in (FOLLOW_LANE, SLOW):
            self._consider_request(now)
            if self.state == CHANGE_LANE:
                return self._changing(lane, obstacles, now)

        if self.state == FOLLOW_LANE:
            if obstacles.front < config.SLOW_DISTANCE:
                self.state = SLOW
            return self.state, config.CRUISE_SPEED, 0.0

        if self.state == SLOW:
            if obstacles.front >= config.SLOW_DISTANCE:
                self.state = FOLLOW_LANE
            return self.state, config.SLOW_SPEED, 0.0

        if self.state == CHANGE_LANE:
            return self._changing(lane, obstacles, now)

        return self.state, 0.0, 0.0

    def _set_available(self, lane, obstacles):
        self.left_available = False
        self.right_available = False
        if lane is None or self._hold or self.state == CHANGE_LANE:
            return
        # The course is known (LANE_COUNT lanes), so once the camera has
        # confirmed which lane this is, the neighbors exist whether or not
        # their far line is in view. From the middle lane the reds are
        # outside the camera's field of view, so requiring them never offered
        # a change. Seeing the far line still counts when the lane is not
        # confirmed (one line only).
        known = getattr(lane, "lane_confirmed", False)
        if (self.lane_index - 1 >= 0 and (known or lane.neighbor_left)
                and obstacles.left > config.SIDE_CLEAR_DISTANCE):
            self.left_available = True
        if (self.lane_index + 1 < config.LANE_COUNT and (known or lane.neighbor_right)
                and obstacles.right > config.SIDE_CLEAR_DISTANCE):
            self.right_available = True

    def _consider_request(self, now):
        direction = self._request
        if direction is None:
            return
        self._request = None
        if direction == "left" and not self.left_available:
            return
        if direction == "right" and not self.right_available:
            return
        self.state = CHANGE_LANE
        self.change_direction = direction
        self.phase = "shift"
        self._phase_t0 = now
        self._settle_since = None
        self._last_offset = None
        self.left_available = False
        self.right_available = False

    @staticmethod
    def _change_speed(obstacles):
        """Speed during a lane change: near cruise, unless something is close ahead."""
        if obstacles.front < config.SLOW_DISTANCE:
            return config.SLOW_SPEED
        return max(config.SLOW_SPEED, config.LANE_CHANGE_SPEED_FRAC * config.CRUISE_SPEED)

    def _changing(self, lane, obstacles, now):
        direction = self.change_direction
        sign = -1.0 if direction == "left" else 1.0
        if now - self._phase_t0 > config.LANE_CHANGE_TIMEOUT_S:
            self._fail()
            return self.state, 0.0, 0.0

        if self.phase == "shift":
            if lane is not None and self._last_offset is not None:
                jumped = abs(lane.offset - self._last_offset) > config.LANE_WIDTH_M * 0.5
                if jumped:
                    self.phase = "settle"
                    self._phase_t0 = now
                    self._settle_since = None
                    self.relock = True
                    self._last_offset = lane.offset
                    return self.state, self._change_speed(obstacles), 0.0
            side = obstacles.left if direction == "left" else obstacles.right
            if side <= config.SIDE_CLEAR_DISTANCE:
                return self._cancel(lane, obstacles)
            if lane is not None:
                self._last_offset = lane.offset
            ramp = min(1.0, (now - self._phase_t0) / max(config.LANE_CHANGE_RAMP_S, 1e-3))
            return self.state, self._change_speed(obstacles), sign * config.LANE_WIDTH_M * ramp

        if (lane is not None
                and abs(lane.offset) < config.LANE_CHANGE_DONE_M
                and abs(lane.heading) < config.LANE_CHANGE_DONE_HEADING):
            if self._settle_since is None:
                self._settle_since = now
            elif now - self._settle_since >= config.LANE_CHANGE_SETTLE_S:
                self.lane_index += int(sign)
                self.change_direction = None
                self.phase = None
                self._last_offset = None
                return self._resume(lane, obstacles)
        else:
            self._settle_since = None
        return self.state, self._change_speed(obstacles), 0.0

    def _cancel(self, lane, obstacles):
        """The destination closed up. Stay in this lane."""
        self.change_direction = None
        self.phase = None
        self._last_offset = None
        return self._resume(lane, obstacles)

    def _resume(self, lane, obstacles):
        if obstacles.front < config.SLOW_DISTANCE:
            self.state = SLOW
            speed = config.SLOW_SPEED
        else:
            self.state = FOLLOW_LANE
            speed = config.CRUISE_SPEED
        self._set_available(lane, obstacles)
        return self.state, speed, 0.0

    def _fail(self):
        self._hold = True
        self.state = STOP
        self.phase = None
        self.change_direction = None
        self._last_offset = None

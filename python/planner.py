"""Decides what the car should do: combines lane info, lidar and wheel telemetry."""

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

    def start(self):
        self.state = FOLLOW_LANE

    def update(self, lane, obstacles, telemetry):
        """Return (state, target_speed, target_offset) for the controller."""
        if self.state == IDLE:
            return self.state, 0.0, 0.0

        # Safety first. A missing lane is handled by LaneFollower: it drives
        # straight through short gaps and stops after LANE_LOST_TIMEOUT_S.
        if telemetry.get("estop") or obstacles.front < config.STOP_DISTANCE:
            self.state = STOP
            return self.state, 0.0, 0.0

        if self.state == STOP:
            # TODO: decide when it's safe to go again (e.g. path clear for 1 s)
            self.state = FOLLOW_LANE

        if self.state == FOLLOW_LANE:
            if obstacles.front < config.SLOW_DISTANCE:
                # Something ahead: try to pass if allowed, otherwise slow down
                can_change = config.LANE_CHANGE_ENABLED and lane is not None
                if can_change and lane.left_lane_exists and obstacles.left > config.SIDE_CLEAR_DISTANCE:
                    self.state, self.change_direction = CHANGE_LANE, "left"
                elif can_change and lane.right_lane_exists and obstacles.right > config.SIDE_CLEAR_DISTANCE:
                    self.state, self.change_direction = CHANGE_LANE, "right"
                else:
                    self.state = SLOW
            return self.state, config.CRUISE_SPEED, 0.0

        if self.state == SLOW:
            if obstacles.front >= config.SLOW_DISTANCE:
                self.state = FOLLOW_LANE
            return self.state, config.SLOW_SPEED, 0.0

        if self.state == CHANGE_LANE:
            # TODO: steer toward the target lane (e.g. target_offset = +/- lane width),
            #       then switch back to FOLLOW_LANE once centered in the new lane
            return self.state, config.SLOW_SPEED, 0.0

        return self.state, 0.0, 0.0

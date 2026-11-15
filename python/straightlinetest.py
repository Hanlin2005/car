"""Straight-line test: drives forward a set distance and uses the encoders
to keep both wheels travelling the same distance, so the car goes straight.

Run it from the web page: pick "Straight-line test", press Run.
"""

import time

import car_link
import config

TEST_DISTANCE = 1.00    # m to drive before stopping
KP_STRAIGHT = 4.0       # how hard to correct drift (rad/s per rad of heading error)
START_DELAY_S = 3.0     # time to put the car down before it moves
MAX_TIME_S = 10.0       # safety: stop after this long no matter what

_start_time = time.time()
_start_dist = None
_go_time = None
_done = False


def reset():
    """Start a fresh run (called each time the mode is started)."""
    global _start_time, _start_dist, _go_time, _done
    _start_time = time.time()
    _start_dist = None
    _go_time = None
    _done = False


def is_done():
    return _done


def loop():
    global _start_dist, _go_time, _done
    t = car_link.telemetry

    # Wait before moving
    if time.time() - _start_time < START_DELAY_S:
        time.sleep(0.1)
        return

    if _done:
        car_link.set_wheel_speeds(0.0, 0.0)
        time.sleep(0.5)
        return

    # Remember where the wheels were when the test started
    if _start_dist is None:
        _start_dist = (t["left_dist"], t["right_dist"])
        _go_time = time.time()
        print("Straight-line test: GO")

    left_dist = t["left_dist"] - _start_dist[0]
    right_dist = t["right_dist"] - _start_dist[1]
    travelled = (left_dist + right_dist) / 2

    # If one wheel has gone further, the car has turned: heading > 0 means turned left
    heading = (right_dist - left_dist) / config.WHEEL_BASE_M

    elapsed = time.time() - _go_time

    # Encoders not counting? Stop instead of driving blind
    if elapsed > 1.5 and left_dist == 0 and right_dist == 0:
        car_link.set_wheel_speeds(0.0, 0.0)
        _done = True
        print("Straight-line test STOPPED: no encoder ticks. Check A/B, 3.3V and GND "
              "(pin numbers are in sketch/pins.h).")
        return

    # A wheel counting backwards while driving forward would make the speed
    # control run away: stop as soon as it shows, and say which encoder to flip
    for side, dist in (("left", left_dist), ("right", right_dist)):
        if dist < -0.005:
            car_link.set_wheel_speeds(0.0, 0.0)
            _done = True
            flag = "INVERT_LEFT_ENCODER" if side == "left" else "INVERT_RIGHT_ENCODER"
            print(f"Straight-line test STOPPED: {side} encoder counts backwards - "
                  f"set {flag} to true in sketch/pins.h")
            return

    timed_out = elapsed > MAX_TIME_S
    if travelled >= TEST_DISTANCE or t["estop"] or timed_out:
        car_link.set_wheel_speeds(0.0, 0.0)
        _done = True
        reason = "obstacle" if t["estop"] else "timeout" if timed_out else "distance reached"
        print(f"Straight-line test DONE ({reason}): "
              f"L={left_dist:.3f} m  R={right_dist:.3f} m  "
              f"diff={right_dist - left_dist:+.3f} m  heading={heading:+.3f} rad")
        return

    # Steer back toward the starting heading
    omega = -KP_STRAIGHT * heading
    half_base = config.WHEEL_BASE_M / 2
    left = config.CRUISE_SPEED - omega * half_base
    right = config.CRUISE_SPEED + omega * half_base
    car_link.set_wheel_speeds(left, right)

    print(f"L={left_dist:.3f} R={right_dist:.3f} heading={heading:+.3f} "
          f"cmd L={left:.2f} R={right:.2f}")
    time.sleep(1 / config.LOOP_HZ)

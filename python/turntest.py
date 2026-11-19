"""Turn calibration: measures the effective wheel base and the turn response.

The camera's heading relative to the lane lines is the reference; the
encoders only count wheel rotation and cannot tell slip from turning.

Part 1, spin in place: TURN_TEST_SPIN_TURNS whole turns by encoder count,
then read the camera's residual heading once the lane is back in view.
Effective wheel base = WHEEL_BASE_M x encoder angle / (whole turns + residual).
Whole turns make the breakaway slip (a few cm per wheel) negligible.
Part 2, timed gentle arcs: compares the turn rate the camera sees with the
one commanded. If the car turns much less than commanded while rolling, the
wheels slip or the chassis resists yaw (a stiff caster), and no wheel-base
number will fix that.

Run it from the web page: pick "Turn test", press Run. Car on the floor in a
lane, pointed along the lines, both lines in view, caster free to swivel.
Results print to the log and go to runs/turn-<date-time>.csv.
"""

import math
import os
import statistics
import time

import car_link
import config
import vision

SETTLE_S = 1.0          # stand still this long before/after each sample
SAMPLE_S = 1.0          # average the camera heading over this long
MAX_SAMPLE_S = 8.0      # give up on a sample after this long
MIN_FRAMES = 5

_stop = None
detail = ""
_trace = None      # per-20 ms log of commanded vs measured wheel speed, see run()
_trace_label = ""
on_detail = lambda text: None     # modes.py hooks this to show progress on the page


def _detail(text):
    global detail
    detail = text
    on_detail(text)


def _say(text):
    _detail(text)
    print(text)


def _enc():
    t = car_link.telemetry
    return float(t["left_dist"]), float(t["right_dist"])


def _front_blocked():
    front = car_link.telemetry.get("tof_front", -1.0)
    return 0 <= front < config.STOP_DISTANCE


def _camera_heading(seconds, label):
    """Average camera heading (rad, + = nose right) over `seconds`.

    Prefers frames with both lane lines. Returns (heading, frames, both) or
    None when the lane was not seen enough.
    """
    both, one = [], []
    last_id = None
    end = time.monotonic() + seconds
    while time.monotonic() < end and not _stop.is_set():
        frame_id, frame_time, lane, _ = vision.latest()
        if frame_id != last_id and lane is not None and time.monotonic() - frame_time < 0.3:
            last_id = frame_id
            h = lane.heading          # not clamped: residuals up to ~40 deg are real
            if abs(h) < 0.8:
                (both if lane.left_lane_exists and lane.right_lane_exists else one).append(h)
        time.sleep(0.01)
    if len(both) >= MIN_FRAMES:
        return statistics.median(both), len(both), True
    frames = both + one
    if len(frames) >= MIN_FRAMES:
        return statistics.median(frames), len(frames), False
    _say(f"{label}: lane not seen (only {len(frames)} frames). Re-aim the car.")
    return None


def _drive(left, right, label, enc_target=None, seconds=None):
    """Drive wheels at (left, right) m/s until the encoders show enc_target rad
    of yaw, or for `seconds`. Returns (encoder yaw, seconds, mean measured
    left/right speeds) or None.
    """
    l0, r0 = _enc()
    t0 = time.monotonic()
    speeds_l, speeds_r = [], []
    if seconds is not None:
        limit = seconds + 1.0
    else:
        # Whole turns at this wheel speed take 2*pi*turns*B/(2*v); allow double.
        expect = abs(enc_target) * config.WHEEL_BASE_M / (abs(left) + abs(right))
        limit = 2.0 * expect + 5.0
    while not _stop.is_set():
        if _front_blocked():
            car_link.set_wheel_speeds(0.0, 0.0)
            _say(f"{label}: obstacle ahead, stopped")
            return None
        car_link.set_wheel_speeds(left, right)
        l, r = _enc()
        angle = ((l - l0) - (r - r0)) / config.WHEEL_BASE_M   # + = nose right
        t = car_link.telemetry
        speeds_l.append(float(t["left_speed"]))
        speeds_r.append(float(t["right_speed"]))
        elapsed = time.monotonic() - t0
        if _trace is not None:
            _trace.write(f"{_trace_label},{elapsed:.3f},{left:.3f},{right:.3f},"
                         f"{t['left_speed']:.3f},{t['right_speed']:.3f},{angle:.4f}\n")
        _detail(f"{label}: enc {math.degrees(angle):+.0f} deg, {elapsed:.1f} s | "
                f"L {left:+.2f} R {right:+.2f} m/s")
        reached = (enc_target is not None and
                   ((enc_target > 0 and angle >= enc_target) or
                    (enc_target < 0 and angle <= enc_target)))
        if reached or (seconds is not None and elapsed >= seconds):
            car_link.set_wheel_speeds(0.0, 0.0)
            n = max(1, len(speeds_l))
            return angle, elapsed, sum(speeds_l) / n, sum(speeds_r) / n
        if elapsed > limit:
            car_link.set_wheel_speeds(0.0, 0.0)
            _say(f"{label}: timed out before reaching the angle")
            return None
        time.sleep(0.02)
    car_link.set_wheel_speeds(0.0, 0.0)
    return None


def _sample(kind, sign, csv):
    """One spin (whole turns) or one timed arc. sign +1 = right, -1 = left."""
    global _trace_label
    label = f"{kind} {'right' if sign > 0 else 'left'}"
    _trace_label = label
    if kind == "spin":
        s = config.TURN_TEST_SPIN_SPEED
        # Right wheel back, left forward -> nose right (encoder yaw +).
        left, right = sign * s, -sign * s
        enc_target = sign * 2 * math.pi * config.TURN_TEST_SPIN_TURNS
        seconds = None
    else:
        v, w = config.TURN_TEST_ARC_SPEED, config.TURN_TEST_ARC_OMEGA
        # omega > 0 turns left (right wheel faster), as in controller.py.
        omega = -sign * w
        half = config.WHEEL_BASE_M / 2
        left, right = v - omega * half, v + omega * half
        enc_target = None
        seconds = config.TURN_TEST_ARC_S

    car_link.set_wheel_speeds(0.0, 0.0)
    time.sleep(SETTLE_S)
    before = _camera_heading(SAMPLE_S, label + " before")
    if before is None or _stop.is_set():
        return None
    driven = _drive(left, right, label, enc_target=enc_target, seconds=seconds)
    if driven is None or _stop.is_set():
        return None
    enc_angle, elapsed, meas_l, meas_r = driven
    time.sleep(SETTLE_S * 2 if kind == "spin" else SETTLE_S)
    if kind == "spin":
        # The pair followed before the spin is stale; lock onto what is ahead now.
        vision.reset_lane_track()
        time.sleep(0.3)
    after = _camera_heading(SAMPLE_S, label + " after")
    if after is None:
        return None
    cam_delta = after[0] - before[0]
    note = "" if before[2] and after[2] else "  (one-line heading, less reliable)"

    if kind == "spin":
        # True rotation = the whole turns plus what the camera sees left over.
        true_angle = sign * 2 * math.pi * config.TURN_TEST_SPIN_TURNS + cam_delta
        base = config.WHEEL_BASE_M * enc_angle / true_angle
        if abs(cam_delta) > 0.6:
            note += ("  LARGE RESIDUAL: near the camera's limit, treat as approximate; "
                     "set WHEEL_BASE_M nearer this value and rerun")
        _say(f"{label}: {config.TURN_TEST_SPIN_TURNS} turns by encoder in {elapsed:.1f} s, "
             f"camera residual {math.degrees(cam_delta):+.1f} deg -> "
             f"wheel base {base * 100:.1f} cm; wheels measured "
             f"L {meas_l:+.2f} R {meas_r:+.2f} m/s{note}")
        rate = true_angle / elapsed
    else:
        true_angle = cam_delta
        rate = cam_delta / elapsed
        base = (config.WHEEL_BASE_M * enc_angle / cam_delta
                if abs(cam_delta) > 0.02 else float("nan"))
        want = -sign * config.TURN_TEST_ARC_OMEGA
        _say(f"{label}: {elapsed:.1f} s, encoders {math.degrees(enc_angle):+.1f} deg, "
             f"camera {math.degrees(cam_delta):+.1f} deg -> actual turn rate "
             f"{rate:+.2f} rad/s vs commanded {-want:+.2f} (camera sign); "
             f"wheels commanded L {left:.2f} R {right:.2f}, measured "
             f"L {meas_l:.2f} R {meas_r:.2f} m/s{note}")
    csv.write(f"{kind},{sign},{enc_angle:.4f},{true_angle:.4f},{elapsed:.3f},"
              f"{left:.3f},{right:.3f},{meas_l:.3f},{meas_r:.3f},"
              f"{int(before[2] and after[2])}\n")
    csv.flush()
    return kind, base, rate, left, right, meas_l, meas_r


def run(stop_event, on_progress=None):
    global _stop, on_detail
    _stop = stop_event
    if on_progress is not None:
        on_detail = on_progress
    vision.reset_lane_track()
    go_at = time.monotonic() + config.DRIVE_START_DELAY_S
    _say(f"Turn test starts in {config.DRIVE_START_DELAY_S:.0f} s. Car in a lane, "
         f"pointed along the lines, caster free. Spins {config.TURN_TEST_SPIN_TURNS} "
         f"full turns each way (count them!), then {config.TURN_TEST_ARC_REPEATS}x gentle arcs.")
    while time.monotonic() < go_at and not stop_event.is_set():
        _detail(f"starting in {max(0.0, go_at - time.monotonic()):.1f} s")
        time.sleep(0.1)
    if stop_event.is_set():
        return

    base = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "runs")
    os.makedirs(base, exist_ok=True)
    stamp = time.strftime("turn-%Y%m%d-%H%M%S")
    path = os.path.join(base, stamp + ".csv")
    global _trace
    _trace = open(os.path.join(base, stamp + "-trace.csv"), "w")
    _trace.write("sample,t,cmd_left,cmd_right,meas_left,meas_right,enc_yaw_rad\n")
    results = []
    with open(path, "w") as csv:
        csv.write("kind,sign,enc_rad,true_rad,seconds,cmd_left,cmd_right,"
                  "meas_left,meas_right,both_lines\n")
        misses = 0
        for kind, repeats in (("spin", config.TURN_TEST_SPIN_REPEATS),
                              ("arc", config.TURN_TEST_ARC_REPEATS)):
            for _ in range(repeats):
                for sign in (+1, -1):
                    if stop_event.is_set() or misses >= 2:
                        break
                    r = _sample(kind, sign, csv)
                    if r is not None:
                        results.append(r)
                        misses = 0
                    else:
                        misses += 1
        if misses >= 2:
            print("Two samples in a row failed: stopping. Re-aim the car at the lines and rerun.")
    car_link.set_wheel_speeds(0.0, 0.0)
    _trace.close()
    _trace = None

    spins = [r[1] for r in results if r[0] == "spin" and not math.isnan(r[1])]
    arcs = [r for r in results if r[0] == "arc" and not math.isnan(r[1])]
    print("=== Turn test results ===")
    print(f"Current WHEEL_BASE_M = {config.WHEEL_BASE_M:.3f}")
    if spins:
        med = statistics.median(spins)
        spread = (max(spins) - min(spins)) * 100
        print(f"Spin in place: effective wheel base {med * 100:.1f} cm "
              f"(n={len(spins)}, spread {spread:.1f} cm). "
              f"Set WHEEL_BASE_M = {med:.3f} in python/config.py.")
    else:
        print("Spin in place: no usable samples.")
    if arcs:
        want = config.TURN_TEST_ARC_OMEGA
        got = statistics.median(abs(r[2]) for r in arcs)
        bases = [r[1] for r in arcs if not math.isnan(r[1])]
        inner_cmd = min(abs(arcs[0][3]), abs(arcs[0][4]))
        inner_meas = statistics.median(min(abs(r[5]), abs(r[6])) for r in arcs)
        print(f"Arcs: commanded turn rate {want:.2f} rad/s, actual (camera) {got:.2f} rad/s "
              f"({got / want * 100:.0f}%). Inner wheel commanded {inner_cmd:.2f} m/s, "
              f"measured {inner_meas:.2f} m/s."
              + (f" Wheel base implied by arcs {statistics.median(bases) * 100:.1f} cm."
                 if bases else ""))
        if got < 0.7 * want:
            print("The car turns much less than commanded while rolling: the wheels "
                  "slip or the chassis resists yaw (caster). Check the caster swivels "
                  "freely; if it does, the speed loops need more authority.")
    else:
        print("Arcs: no usable samples.")
    summary = (f"done | base {statistics.median(spins) * 100:.1f} cm" if spins else
               "done | no usable spin samples")
    _say(summary + f" | {path}")

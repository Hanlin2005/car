"""Duty-cycle comparison: runs both motors at the same PWM duty cycles,
measures each wheel's speed with the encoders, and prints the differences.

PROP THE CAR UP so the wheels spin in the air - this runs for ~30 s.
Run it from the web page: pick "Duty-cycle test", press Run.
"""

import statistics
import time

import car_link
import config

DUTY_CYCLES = [20, 30, 40, 50, 60, 70, 80, 90, 100]   # percent
SETTLE_S = 1.0          # let the motor reach speed before measuring
MEASURE_S = 2.0         # how long to measure at each duty cycle
START_DELAY_S = 3.0

_done = False
_stop_event = None


def reset(stop_event=None):
    """Start a fresh run. stop_event ends it early."""
    global _done, _stop_event
    _done = False
    _stop_event = stop_event


def is_done():
    return _done


def _stopped():
    return _stop_event is not None and _stop_event.is_set()


TIMING_KEYS = ["left_pd", "left_ab", "right_pd", "right_ab"]


def _hold(left_pct, right_pct, seconds):
    """Keep sending the duty cycle (the microcontroller stops the motors if commands go quiet) and
    collect samples. Returns (left_speeds, right_speeds, timing) where
    timing maps each TIMING_KEYS entry to its non-zero samples in ms."""
    left, right = [], []
    timing = {k: [] for k in TIMING_KEYS}
    end = time.time() + seconds
    while time.time() < end and not _stopped():
        car_link.set_duty(left_pct, right_pct)
        t = car_link.telemetry
        left.append(abs(t["left_speed"]))
        right.append(abs(t["right_speed"]))
        for k in TIMING_KEYS:
            if t[k] > 0:
                timing[k].append(t[k])
        time.sleep(0.05)
    return left, right, timing


def _avg(samples):
    return statistics.mean(samples) if samples else 0.0


def _interp_duty(speed, duties, speeds):
    """Duty cycle at which a motor reaches `speed`, from its measured curve."""
    points = sorted(zip(speeds, duties))
    for (s0, d0), (s1, d1) in zip(points, points[1:]):
        if s0 <= speed <= s1 and s1 > s0:
            return d0 + (speed - s0) * (d1 - d0) / (s1 - s0)
    return None


def run_sweep():
    print(f"Duty-cycle test starting in {START_DELAY_S:.0f} s - wheels must be off the ground!")
    end = time.time() + START_DELAY_S
    while time.time() < end and not _stopped():
        time.sleep(0.05)

    results = []
    for duty in DUTY_CYCLES:
        if _stopped():
            car_link.stop()
            print("Duty-cycle test stopped.")
            return
        _hold(duty, duty, SETTLE_S)
        t = car_link.telemetry
        start_l, start_r, start_time = t["left_dist"], t["right_dist"], time.time()
        left, right, timing = _hold(duty, duty, MEASURE_S)
        elapsed = time.time() - start_time
        if _stopped() or elapsed <= 0:
            continue
        # Average speed from total distance (more accurate than averaging samples)
        results.append({
            "duty": duty,
            "left": abs(t["left_dist"] - start_l) / elapsed,
            "right": abs(t["right_dist"] - start_r) / elapsed,
            "left_sd": statistics.pstdev(left),
            "right_sd": statistics.pstdev(right),
            **{k: _avg(timing[k]) for k in TIMING_KEYS},
        })
        print(f"  {duty:3d}% measured")

    car_link.set_duty(0, 0)
    car_link.stop()

    # --- Table: same duty cycle on both motors ---
    print()
    print("Duty |  Left m/s (±sd)   | Right m/s (±sd)   |  R-L m/s | R vs L  | Steadiness L / R")
    print("-----+-------------------+-------------------+----------+---------+-----------------")
    for r in results:
        diff = r["right"] - r["left"]
        pct = 100 * diff / r["left"] if r["left"] > 0 else float("nan")
        cv_l = 100 * r["left_sd"] / r["left"] if r["left"] > 0 else float("nan")
        cv_r = 100 * r["right_sd"] / r["right"] if r["right"] > 0 else float("nan")
        print(f"{r['duty']:3d}% | {r['left']:.3f} (±{r['left_sd']:.3f}) | "
              f"{r['right']:.3f} (±{r['right_sd']:.3f}) | {diff:+.3f}   | {pct:+6.1f}% | "
              f"{cv_l:5.1f}% / {cv_r:5.1f}%")
    print("(Steadiness = speed jitter as % of speed; lower is more consistent)")

    # --- Table: encoder timing, same quantities as the scope readings ---
    print()
    print("Encoder timing (ms) - pd = period of A, 2-1 = A edge -> B edge")
    print("Duty |  L pd  |  R pd  | R-L pd |  L 2-1 |  R 2-1 | R-L 2-1")
    print("-----+--------+--------+--------+--------+--------+--------")
    for r in results:
        print(f"{r['duty']:3d}% | {r['left_pd']:6.2f} | {r['right_pd']:6.2f} | "
              f"{r['right_pd'] - r['left_pd']:+6.2f} | {r['left_ab']:6.2f} | "
              f"{r['right_ab']:6.2f} | {r['right_ab'] - r['left_ab']:+6.2f}")
    print("(0.00 = no edges seen on that pin)")

    # --- Matching: what right duty gives the same speed as left? ---
    duties = [r["duty"] for r in results]
    right_speeds = [r["right"] for r in results]
    print()
    print("Left duty -> right duty for the same speed")
    print("---------------------------------------------")
    for r in results:
        match = _interp_duty(r["left"], duties, right_speeds)
        if match is None:
            print(f"   {r['duty']:3d}% -> out of measured range")
        else:
            print(f"   {r['duty']:3d}% -> {match:5.1f}%   (offset {match - r['duty']:+.1f}%)")


def loop():
    global _done
    if not _done:
        run_sweep()
        _done = True

    car_link.set_duty(0, 0)
    time.sleep(0.5)

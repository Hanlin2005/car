"""Link to the Uno Q microcontroller (sketch/sketch.ino) over the Bridge.

The sketch drives the DRV8833, reads the encoders, and decodes the lidar.
This module keeps the same commands and telemetry the rest of the program
already uses. Bridge calls happen from pump(), which main.py runs on the
App thread, so the web page and the mode threads can request speeds safely.
"""

import threading
import time

import config

telemetry = {
    "left_speed": 0.0,
    "right_speed": 0.0,
    "left_dist": 0.0,
    "right_dist": 0.0,
    "tof_front": 0.0,        # front lidar distance, meters
    "estop": False,
    "left_pd": 0.0,
    "left_ab": 0.0,
    "right_pd": 0.0,
    "right_ab": 0.0,
}

sensors = {
    "tof_m": -1.0,           # VL53L1X distance, meters; -1 if there is no reading
    "lux": -1.0,             # BH1750; -1 if there is no reading
    "lights_on": False,
}

lidar_stats = {
    "front": float("inf"),
    "back": float("inf"),
    "left": float("inf"),
    "right": float("inf"),
    "good": 0,
    "bad": 0,
    "bytes": 0,
    "updates": 0,
    "error": "",
}

_lock = threading.Lock()
_pending = {
    "kind": "stop",          # "stop" | "speed" | "duty"
    "left": 0.0,
    "right": 0.0,
    "mode": "idle",
    "straight_hold": True,   # encoder-difference straightening, web page toggle
    "all_lights": False,
    "rear1": False,
    "rear2": False,
    "head": False,
}
_started = False
_Bridge = None


def _on_telemetry(left_speed, right_speed, left_dist, right_dist, front, estop):
    telemetry["left_speed"] = float(left_speed)
    telemetry["right_speed"] = float(right_speed)
    telemetry["left_dist"] = float(left_dist)
    telemetry["right_dist"] = float(right_dist)
    telemetry["tof_front"] = float(front)
    telemetry["estop"] = bool(estop)


def _on_timing(left_pd, left_ab, right_pd, right_ab):
    telemetry["left_pd"] = float(left_pd)
    telemetry["left_ab"] = float(left_ab)
    telemetry["right_pd"] = float(right_pd)
    telemetry["right_ab"] = float(right_ab)


def _on_sensors(tof_m, lux, lights_on):
    sensors["tof_m"] = float(tof_m)
    sensors["lux"] = float(lux)
    sensors["lights_on"] = bool(lights_on)


def _on_lidar(front, back, left, right, good, bad, nbytes):
    def meters(value):
        value = float(value)
        return value if value >= 0 else float("inf")

    lidar_stats["front"] = meters(front)
    lidar_stats["back"] = meters(back)
    lidar_stats["left"] = meters(left)
    lidar_stats["right"] = meters(right)
    lidar_stats["good"] = int(good)
    lidar_stats["bad"] = int(bad)
    lidar_stats["bytes"] = int(nbytes)
    lidar_stats["updates"] += 1
    lidar_stats["error"] = ""


def start():
    """Register the messages the sketch sends up. Call before App.run()."""
    global _started, _Bridge
    if _started:
        return
    try:
        from arduino.app_utils import Bridge
    except ImportError:
        raise SystemExit(
            "This folder runs on the Arduino Uno Q, inside Arduino App Lab."
        )
    _Bridge = Bridge
    Bridge.provide("telemetry", _on_telemetry)
    Bridge.provide("encoder_timing", _on_timing)
    Bridge.provide("lidar", _on_lidar)
    Bridge.provide("sensors", _on_sensors)
    with _lock:
        _pending["mode"] = config.MODE if config.MODE in (
            "web", "drive", "straight_test", "duty_test", "motor_test", "lidar_test",
            "light_test", "sensor_test", "motor1_test", "motor2_test",
        ) else "idle"
    _started = True
    print("Uno Q: waiting for the microcontroller (pin numbers are in sketch/pins.h).")


def pump():
    """Forward the latest command to the sketch. Call this from the App loop."""
    if _Bridge is None:
        return
    with _lock:
        kind = _pending["kind"]
        left = _pending["left"]
        right = _pending["right"]
        mode = _pending["mode"]
        straight_hold = _pending["straight_hold"]
        all_lights = _pending["all_lights"]
        rear1 = _pending["rear1"]
        rear2 = _pending["rear2"]
        head = _pending["head"]
    _Bridge.notify("set_mode", mode)
    _Bridge.notify("set_straight_hold", bool(straight_hold))
    _Bridge.notify("set_all_lights", all_lights)
    _Bridge.notify("set_lamps", bool(rear1), bool(rear2), bool(head))
    if kind == "speed":
        _Bridge.notify("set_wheel_speeds", float(left), float(right))
    elif kind == "duty":
        _Bridge.notify("set_duty", float(left), float(right))
    else:
        _Bridge.notify("stop")


def set_mode(name):
    with _lock:
        name = str(name)
        _pending["mode"] = name
        if name != "light_test":
            _pending["rear1"] = False
            _pending["rear2"] = False
            _pending["head"] = False


def set_lamps(rear1, rear2, head):
    """Drive the three lamp pins. The sketch applies this only in light test."""
    with _lock:
        _pending["rear1"] = bool(rear1)
        _pending["rear2"] = bool(rear2)
        _pending["head"] = bool(head)


def set_all_lights(enabled):
    with _lock:
        _pending["all_lights"] = enabled


def all_lights_enabled():
    with _lock:
        return _pending["all_lights"]


def lamp_state():
    with _lock:
        return {
            "rear1": bool(_pending["rear1"]),
            "rear2": bool(_pending["rear2"]),
            "head": bool(_pending["head"]),
        }


def set_straight_hold(enabled):
    """Turn encoder-difference straightening on or off. Manual driving only."""
    with _lock:
        _pending["straight_hold"] = bool(enabled)


def straight_hold_enabled():
    with _lock:
        return bool(_pending["straight_hold"])


def set_wheel_speeds(left, right):
    with _lock:
        _pending["kind"] = "speed"
        _pending["left"] = float(left)
        _pending["right"] = float(right)


def set_duty(left_pct, right_pct):
    """Raw PWM duty cycle in percent (-100..100), no speed control. For testing."""
    with _lock:
        _pending["kind"] = "duty"
        _pending["left"] = float(left_pct)
        _pending["right"] = float(right_pct)


def stop():
    with _lock:
        _pending["kind"] = "stop"
        _pending["left"] = 0.0
        _pending["right"] = 0.0


def shutdown():
    stop()
    set_mode("idle")
    pump()


def _wait(seconds, stop_event):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if stop_event is not None and stop_event.is_set():
            return False
        time.sleep(0.05)
    return True


def _hold_duty(left_pct, right_pct, seconds, stop_event):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if stop_event is not None and stop_event.is_set():
            return False
        set_duty(left_pct, right_pct)
        time.sleep(0.05)
    return True


def motor_test(stop_event=None):
    """Spin each motor both ways, full power then 60%. Wheels must be off the ground."""
    print("Motor test starts in 3 s. Prop the car up so the wheels are in the air.")
    if not _wait(3.0, stop_event):
        stop()
        return
    steps = (
        ("LEFT  forward  (full)", 100, 0),
        ("LEFT  backward (full)", -100, 0),
        ("RIGHT forward  (full)", 0, 100),
        ("RIGHT backward (full)", 0, -100),
        ("LEFT  forward  (60%)", 60, 0),
        ("LEFT  backward (60%)", -60, 0),
        ("RIGHT forward  (60%)", 0, 60),
        ("RIGHT backward (60%)", 0, -60),
    )
    while stop_event is None or not stop_event.is_set():
        for label, left, right in steps:
            if stop_event is not None and stop_event.is_set():
                break
            start_l = telemetry["left_dist"]
            start_r = telemetry["right_dist"]
            print(f"TEST: {label}")
            if not _hold_duty(left, right, 2.0, stop_event):
                break
            stop()
            moved_l = telemetry["left_dist"] - start_l
            moved_r = telemetry["right_dist"] - start_r
            tick = config.METERS_PER_TICK or 1
            print(f"  encoder ticks  left={moved_l / tick:.0f}  right={moved_r / tick:.0f}")
            if not _wait(1.0, stop_event):
                break
        else:
            print("TEST: done, repeating in 3 s")
            if _wait(3.0, stop_event):
                continue
        break
    stop()

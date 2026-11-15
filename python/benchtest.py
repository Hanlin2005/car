"""Bench checks started from the web page.

Light test prints the lux reading and which lamp switches are on. The page
drives the pins. Sensor test prints the distance sensors and does not move
the wheels. Motor1 and Motor2 each run one wheel forward and back.
"""

import time

import car_link
import hardware
from lidar import Lidar

_SPEED = 0.20
_PHASES = (("forward", 1.5), ("pause", 0.5), ("back", 1.5), ("pause", 0.5))
_lidar = Lidar()


def _lux():
    lux = hardware.sensors["lux"]
    return "no reading" if lux < 0 else f"{lux:.0f} lux"


def _tof():
    distance = hardware.sensors["tof_m"]
    return "no reading" if distance < 0 else f"{distance:.2f} m"


def _meters(value):
    return "  ---  " if value == float("inf") else f"{value:5.2f} m"


def light_loop(stop_event):
    print("Light test: use the switches. The wheels stay still.")
    while not stop_event.is_set():
        lamps = hardware.lamp_state()
        print(f"{_lux()} | rear 1 {'on' if lamps['rear1'] else 'off'} | "
              f"rear 2 {'on' if lamps['rear2'] else 'off'} | "
              f"headlights {'on' if lamps['head'] else 'off'}")
        stop_event.wait(0.5)


def sensor_loop(stop_event):
    print("Sensor test: the wheels stay still.")
    while not stop_event.is_set():
        obstacles = _lidar.read()
        lights = "on" if hardware.sensors["lights_on"] else "off"
        print(f"ToF {_tof()} | {_lux()} | lights {lights} | "
              f"front {_meters(obstacles.front)} | back {_meters(obstacles.back)} | "
              f"left {_meters(obstacles.left)} | right {_meters(obstacles.right)}")
        stop_event.wait(0.5)


def motor_loop(stop_event, which):
    """Run one wheel forward, pause, back, pause, until stop_event is set.

    which is "motor1" (right command) or "motor2" (left command).
    """
    right = which == "motor1"
    label = "Motor1" if right else "Motor2"
    side = "right" if right else "left"
    print(f"{label}: commanding the {side} wheel only at {_SPEED:.2f} m/s. "
          "The other wheel stays stopped.")
    step = 0
    phase_until = 0.0
    phase_name = "pause"
    while not stop_event.is_set():
        now = time.monotonic()
        if now >= phase_until:
            phase_name, duration = _PHASES[step % len(_PHASES)]
            step += 1
            phase_until = now + duration
            speed = _SPEED if phase_name == "forward" else -_SPEED if phase_name == "back" else 0.0
            if right:
                car_link.set_wheel_speeds(0.0, speed)
            else:
                car_link.set_wheel_speeds(speed, 0.0)
        t = car_link.telemetry
        print(f"{label} {phase_name} | "
              f"left {t['left_dist']:.3f} m {t['left_speed']:+.2f} m/s | "
              f"right {t['right_dist']:.3f} m {t['right_speed']:+.2f} m/s")
        stop_event.wait(0.5)
    car_link.stop()

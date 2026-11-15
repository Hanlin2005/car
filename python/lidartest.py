"""Lidar test: prints the distance to the front, back, left and right
continuously. Run it from the web page: pick "Lidar test", press Run.
"""

import time

import config
from lidar import Lidar

lidar = Lidar()
_last_warning = 0.0


def _fmt(meters):
    return "  ---  " if meters == float("inf") else f"{meters:5.2f} m"


def _warn(message):
    global _last_warning
    if time.time() - _last_warning > 3:
        print(message)
        _last_warning = time.time()


def loop():
    s = lidar.stats
    if s["error"] and s["bytes"] == 0:
        _warn(s["error"])
    elif s["bytes"] == 0:
        _warn("No lidar bytes on D0. Check lidar TX -> D0, lidar 5V -> the Uno Q 5V pin, "
              "and a shared ground.")
    elif s["good"] == 0:
        _warn(f"Seeing {s['bytes']} bytes but no valid packets "
              f"({s['bad']} bad checksums). Check the ground, or the baud rate "
              f"(expected {config.LIDAR_BAUD}).")
    else:
        o = lidar.read()
        print(f"front {_fmt(o.front)} | back {_fmt(o.back)} | "
              f"left {_fmt(o.left)} | right {_fmt(o.right)}")

    time.sleep(0.2)

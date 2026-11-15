"""Wheel commands and encoder readings.

hardware.py talks to the Uno Q microcontroller. This is the small API the
rest of the program uses: set a speed, set a raw duty cycle, or read telemetry.
"""

import hardware

# Latest wheel data. Speeds are m/s, distances are meters since startup.
telemetry = hardware.telemetry


def set_wheel_speeds(left, right):
    hardware.set_wheel_speeds(left, right)


def set_straight_hold(enabled):
    """Turn encoder-difference straightening on or off while driving straight."""
    hardware.set_straight_hold(enabled)


def straight_hold_enabled():
    return hardware.straight_hold_enabled()


def set_duty(left_pct, right_pct):
    """Raw PWM duty cycle in percent (-100..100), no speed control. For testing."""
    hardware.set_duty(left_pct, right_pct)


def stop():
    hardware.stop()

"""Runs one mode at a time, started and stopped from the web page.

select(name) picks a mode (and stops whatever was running).
run() starts the selected mode in a background thread.
stop() ends it and stops the wheels.

Everything printed is also kept in a log the web page shows.
"""

import collections
import sys
import threading
import time
import traceback

import car_link
import config
import hardware

# name: (button label, what it does, moves the car by itself)
MODES = collections.OrderedDict([
    ("web", ("Manual driving",
             "Drive with the buttons or WASD / arrow keys. Releasing stops the car.",
             False)),
    ("drive", ("Lane following",
               "Follows the yellow (left) and red (right) lines by itself. "
               "Put the car in the middle of the lane. It starts moving 3 s after Run.",
               True)),
    ("straight_test", ("Straight-line test",
                       "Drives 1 m straight using the wheel encoders. Car on the floor. "
                       "Starts 3 s after Run.",
                       True)),
    ("motor_test", ("Motor test",
                    "PROP THE CAR UP. Spins each wheel both ways and prints encoder ticks. "
                    "Repeats until Stop.",
                    True)),
    ("duty_test", ("Duty-cycle test",
                   "PROP THE CAR UP. Compares left/right wheel speed at each duty cycle "
                   "(about 30 s), then prints tables.",
                   True)),
    ("lidar_test", ("Lidar test",
                    "Prints front / back / left / right distances. The wheels don't move.",
                    False)),
    ("light_test", ("Light test",
                    "Switches for each lamp. The wheels don't move. "
                    "Automatic lights resume after Stop.",
                    False)),
    ("sensor_test", ("Sensor test",
                     "Prints the time-of-flight distance, the light level, and the lidar "
                     "distances. The wheels don't move.",
                     False)),
    ("motor1_test", ("Motor1",
                     "PROP THE CAR UP. Runs Motor1 (right command) forward and back. "
                     "Prints both encoder readings.",
                     True)),
    ("motor2_test", ("Motor2",
                     "PROP THE CAR UP. Runs Motor2 (left command) forward and back. "
                     "Prints both encoder readings.",
                     True)),
])

_lock = threading.RLock()
_selected = config.MODE if config.MODE in MODES else "web"
_running = None           # name of the mode running now, or None
_thread = None
_stop_event = None
_result = ""              # how the last run ended
_heartbeat = time.monotonic()
detail = ""               # one live status line from the running mode


# ---------------------------------------------------------------- log

class _Tee:
    """Send prints to the terminal and keep the last lines for the page."""

    def __init__(self, real):
        self.real = real
        self.lines = collections.deque(maxlen=400)
        self.next_id = 0
        self._partial = ""
        self._lock = threading.Lock()

    def write(self, text):
        self.real.write(text)
        with self._lock:
            parts = (self._partial + text).split("\n")
            self._partial = parts.pop()
            for line in parts:
                self.lines.append((self.next_id, line))
                self.next_id += 1
        return len(text)

    def flush(self):
        self.real.flush()

    def since(self, first_id):
        with self._lock:
            return [(i, line) for i, line in self.lines if i >= first_id], self.next_id

    def __getattr__(self, name):
        return getattr(self.real, name)


log = _Tee(sys.stdout)


def install_log():
    sys.stdout = log


# ---------------------------------------------------------------- control

def heartbeat():
    """The web page calls this while it is open."""
    global _heartbeat
    _heartbeat = time.monotonic()


def status():
    with _lock:
        return {
            "selected": _selected,
            "running": _running,
            "result": _result,
            "detail": detail,
            "modes": [{"name": n, "label": m[0], "help": m[1], "moves": m[2]}
                      for n, m in MODES.items()],
        }


def manual_active():
    return _running == "web"


def select(name):
    global _selected, _result
    if name not in MODES:
        raise ValueError(f"unknown mode {name!r}")
    if _running is not None and _running != name:
        stop()              # not under _lock: the mode thread needs it to finish
    with _lock:
        _selected = name
        _result = ""


def run():
    global _running, _thread, _stop_event, _result, detail
    with _lock:
        if _running is not None:
            return
        name = _selected
        car_link.stop()
        config.MODE = name          # the sketch reads this (e-stop, web timeout)
        hardware.set_mode(name)
        heartbeat()
        _stop_event = threading.Event()
        _running, _result, detail = name, "", ""
        _thread = threading.Thread(target=_run_mode, args=(name, _stop_event),
                                   name=f"mode-{name}", daemon=True)
        _thread.start()
    print(f"=== {MODES[name][0]}: started ===")


def stop():
    global _running, _result
    with _lock:
        if _running is None:
            car_link.stop()
            return
        name, thread, event = _running, _thread, _stop_event
        event.set()
    thread.join(timeout=5.0)
    car_link.stop()
    with _lock:
        if _running == name:
            _running = None
            _result = "stopped"
            config.MODE = "idle"
            hardware.set_mode("idle")
    print(f"=== {MODES[name][0]}: stopped ===")


def _run_mode(name, stop_event):
    global _running, _result
    result = "finished"
    try:
        _RUNNERS[name](stop_event)
        if stop_event.is_set():
            result = "stopped"
    except Exception:
        traceback.print_exc()
        result = "error (see log)"
    finally:
        car_link.stop()
        with _lock:
            if _running == name and _stop_event is stop_event:
                _running = None
                _result = result
                config.MODE = "idle"
                hardware.set_mode("idle")
        if result != "stopped":
            print(f"=== {MODES[name][0]}: {result} ===")


def _watchdog():
    """Stop a self-driving mode if the web page goes away."""
    while True:
        time.sleep(0.25)
        name = _running
        if name and MODES[name][2]:
            if time.monotonic() - _heartbeat > config.WEB_HEARTBEAT_S:
                print(f"Web page lost for {config.WEB_HEARTBEAT_S:.0f} s: stopping.")
                stop()


def start_watchdog():
    threading.Thread(target=_watchdog, name="mode-watchdog", daemon=True).start()


# ---------------------------------------------------------------- the modes

def _run_web(stop_event):
    print("Manual driving: hold a direction on the page.")
    stop_event.wait()


def _run_drive(stop_event):
    global detail
    import vision
    from controller import LaneFollower
    from lidar import Lidar
    from planner import IDLE, Planner

    planner = Planner()
    follower = LaneFollower()
    lidar = Lidar()
    go_at = time.monotonic() + config.DRIVE_START_DELAY_S
    print(f"Lane following starts in {config.DRIVE_START_DELAY_S:.0f} s.")
    last_id = None
    last_print = 0.0
    rec = _Recorder() if config.RECORD_DRIVE else None

    while not stop_event.is_set():
        # Wait for a new camera frame (the camera sets the loop rate).
        wait_until = time.monotonic() + 0.3
        while not stop_event.is_set():
            frame_id, frame_time, lane, _ = vision.latest()
            if frame_id != last_id or time.monotonic() > wait_until:
                break
            time.sleep(0.005)
        if stop_event.is_set():
            break
        now = time.monotonic()
        if frame_id == last_id or now - frame_time > 0.3:
            lane = None            # no fresh frame: treat as no lane
        last_id = frame_id

        if planner.state == IDLE and now >= go_at:
            planner.start()
            print("Lane following: GO")
        obstacles = lidar.read()
        state, speed, target = planner.update(lane, obstacles, car_link.telemetry)
        left, right = follower.update(lane, speed, target)
        car_link.set_wheel_speeds(left, right)
        if rec is not None and planner.state != IDLE:
            rec.add(frame_id, lane, follower, planner.state, left, right)

        if planner.state == IDLE:
            state = f"starting in {max(0.0, go_at - now):.1f} s"
        elif follower.lane_lost and speed > 0:
            state = "LANE_LOST"
        seen = "no lane" if lane is None else \
            f"off {lane.offset * 100:+.1f} cm, head {lane.heading:+.2f}"
        front = "---" if obstacles.front == float("inf") else f"{obstacles.front:.2f} m"
        detail = (f"{state} | {seen} | turn {follower.omega:+.2f} | "
                  f"rejected {follower.rejected} | "
                  f"L {left:.2f} R {right:.2f} m/s | front {front}")
        if now - last_print > 0.5:
            last_print = now
            print(detail)


class _Recorder:
    """Saves each lane-following frame (raw JPEG) and a CSV of what the car
    measured and commanded, in runs/drive-<date-time>/ next to python/."""

    def __init__(self):
        import os
        import cv2
        self.cv2 = cv2
        base = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "runs")
        self.dir = os.path.join(base, time.strftime("drive-%Y%m%d-%H%M%S"))
        os.makedirs(self.dir, exist_ok=True)
        self.csv = open(os.path.join(self.dir, "log.csv"), "w")
        self.csv.write("t,frame,file,seen,offset_m,heading_rad,left_line,right_line,"
                       "state,turn,rejected,cmd_left,cmd_right,meas_left,meas_right\n")
        self.t0 = time.monotonic()
        self.count = 0
        self.last_saved = None
        print(f"Recording this run to {self.dir}")

    def add(self, frame_id, lane, follower, state, left, right):
        import vision
        name = ""
        if frame_id != self.last_saved and self.count < config.RECORD_MAX_FRAMES:
            fid, frame = vision.latest_frame()
            if frame is not None and fid == frame_id:
                name = f"f{self.count:05d}.jpg"
                self.cv2.imwrite(f"{self.dir}/{name}", frame,
                                 [int(self.cv2.IMWRITE_JPEG_QUALITY), 85])
                self.count += 1
                self.last_saved = frame_id
        t = car_link.telemetry
        if lane is None:
            lane_cols = "0,,,,"
        else:
            lane_cols = (f"1,{lane.offset:.4f},{lane.heading:.4f},"
                         f"{int(lane.left_lane_exists)},{int(lane.right_lane_exists)}")
        self.csv.write(f"{time.monotonic() - self.t0:.3f},{frame_id},{name},{lane_cols},"
                       f"{state},{follower.omega:.3f},{follower.rejected},{left:.3f},{right:.3f},"
                       f"{t['left_speed']:.3f},{t['right_speed']:.3f}\n")
        self.csv.flush()


def _run_straight(stop_event):
    import straightlinetest
    straightlinetest.reset()
    while not stop_event.is_set() and not straightlinetest.is_done():
        straightlinetest.loop()


def _run_motor(stop_event):
    hardware.motor_test(stop_event)


def _run_duty(stop_event):
    import dutycycletest
    dutycycletest.reset(stop_event)
    dutycycletest.loop()


def _run_lidar(stop_event):
    import lidartest
    while not stop_event.is_set():
        lidartest.loop()


def _run_light(stop_event):
    import benchtest
    benchtest.light_loop(stop_event)


def _run_sensors(stop_event):
    import benchtest
    benchtest.sensor_loop(stop_event)


def _run_motor1(stop_event):
    import benchtest
    benchtest.motor_loop(stop_event, "motor1")


def _run_motor2(stop_event):
    import benchtest
    benchtest.motor_loop(stop_event, "motor2")


_RUNNERS = {
    "web": _run_web,
    "drive": _run_drive,
    "straight_test": _run_straight,
    "motor_test": _run_motor,
    "duty_test": _run_duty,
    "lidar_test": _run_lidar,
    "light_test": _run_light,
    "sensor_test": _run_sensors,
    "motor1_test": _run_motor1,
    "motor2_test": _run_motor2,
}

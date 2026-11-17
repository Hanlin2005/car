"""The car's web page. Open http://10.194.91.71:8080 on your computer.

Pick a mode, press Run, press Stop. Video is a stream of JPEGs from the
Media Carrier camera with the lane detection drawn on it. In manual driving, holding a
direction sends wheel speeds; letting go, or losing the page, stops the car.
Self-driving modes stop if the page goes away (modes.py watchdog).
"""

import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import car_link
import config
import hardware
import modes
import vision

_command = {"left": 0.0, "right": 0.0, "blocked": False}

_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Car</title>
<style>
  body { margin: 0; background: #14160f; color: #ece7da;
         font: 16px/1.4 "Segoe UI", sans-serif; }
  main { max-width: 680px; margin: 0 auto; padding: 16px; }
  h1 { font-size: 20px; font-weight: 600; margin: 0 0 12px; }
  h2 { font-size: 15px; font-weight: 600; margin: 18px 0 8px; color: #c8c2b2; }
  .view { position: relative; background: #000; }
  .view img { display: block; width: 100%; aspect-ratio: 4 / 3;
              object-fit: contain; background: #000; }
  .novideo { position: absolute; inset: 0; display: flex; align-items: center;
             justify-content: center; margin: 0; padding: 16px; text-align: center;
             color: #c8c2b2; }
  .hidden { display: none !important; }
  .modes { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; }
  .modes button { height: 52px; font-size: 14px; }
  .modes button.selected { border: 2px solid #e0b03a; }
  .modes button.running { background: #3d5a1e; }
  #help { color: #c8c2b2; margin: 8px 0; min-height: 2.8em; }
  #help.warn { color: #e0b03a; }
  .runbar { display: flex; gap: 8px; }
  .runbar button { flex: 1; height: 56px; font-size: 18px; font-weight: 600; }
  #run { background: #2f5a1e; border-color: #5f9a3c; }
  #halt { background: #6a1f17; border-color: #b04a3a; }
  #state { margin: 10px 0; color: #c8c2b2; }
  #detail, #sensors { font: 13px/1.4 ui-monospace, Menlo, Consolas, monospace; color: #9fd77a;
            min-height: 1.4em; margin: 0 0 8px; }
  #link.bad, #state.bad { color: #e07a4a; }
  label.speed { display: flex; gap: 12px; align-items: center; }
  label.speed input { flex: 1; }
  label.check { display: flex; gap: 10px; align-items: center; margin: 12px 0 0; }
  label.check input { width: 20px; height: 20px; accent-color: #e0b03a; }
  label.check span.state { margin-left: auto; color: #c8c2b2; }
  p.note { color: #9a947f; margin: 4px 0 0; font-size: 14px; }
  .pad { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 8px;
         max-width: 300px; margin: 16px auto; }
  button { height: 64px; font: inherit; font-size: 16px; color: inherit;
           background: #26291e; border: 1px solid #5e654c; border-radius: 8px;
           cursor: pointer; }
  button:disabled { opacity: 0.4; cursor: default; }
  .pad button.held, .pad button:active { background: #e0b03a; color: #1c1708; }
  p.hint { color: #9a947f; margin: 0; text-align: center; }
  #log { background: #0b0c08; border: 1px solid #33372a; border-radius: 6px;
         height: 220px; overflow-y: auto; margin: 0; padding: 8px;
         font: 12px/1.35 ui-monospace, Menlo, Consolas, monospace; white-space: pre-wrap; }
</style>
</head>
<body>
<main>
  <h1>Car</h1>
  <div class="view">
    <img id="video" src="/stream.mjpg" alt="Camera">
    <p id="novideo" class="novideo">No camera frame yet. Check the camera ribbon cable on the Media Carrier.</p>
  </div>

  <h2>Mode</h2>
  <div class="modes" id="modes"></div>
  <p id="help"></p>
  <div class="runbar">
    <button type="button" id="run">Run</button>
    <button type="button" id="halt">Stop</button>
  </div>
  <p id="state">Connecting…</p>
  <p id="detail"></p>
  <p id="sensors"></p>

  <label class="speed">Speed
    <input id="speed" type="range" min="0.05" max="0.57" step="0.01" value="0.30">
    <span id="speed-label">0.30 m/s</span>
  </label>
  <p class="note">Manual driving, lane following, and the straight-line test use this speed. A close obstacle uses half of it.</p>

  <h2>Vision lane following</h2>
  <label class="check">
    <input id="vision" type="checkbox" checked>
    Vision control
    <span id="vision-label" class="state">On</span>
  </label>
  <label class="speed">Strength
    <input id="vision-gain" type="range" min="0" max="3" step="0.1" value="1">
    <span id="vision-gain-label">1.0×</span>
  </label>
  <p class="note">1.0 is the normal steering strength. Off drives straight and ignores the lane lines. A close obstacle still stops the car.</p>

  <div id="lanes" class="hidden">
    <h2>Change lane</h2>
    <div class="runbar">
      <button type="button" id="lane-left" disabled>Left: not available</button>
      <button type="button" id="lane-right" disabled>Right: not available</button>
    </div>
    <p class="note">Available means that lane is in view, that side is clear, and the car is not already at that edge. The car stays in the new lane.</p>
  </div>

  <div id="lamps" class="hidden">
    <h2>Lamps</h2>
    <label class="check">
      <input id="rear1" type="checkbox">
      Rear light 1
    </label>
    <label class="check">
      <input id="rear2" type="checkbox">
      Rear light 2
    </label>
    <label class="check">
      <input id="head" type="checkbox">
      Headlights
    </label>
  </div>

  <div id="manual" class="hidden">
    <h2>Manual driving</h2>
    <p id="link"></p>
    <label class="speed">Correction
      <input id="correction" type="range" min="-100" max="100" step="1" value="0">
      <span id="correction-label">Off</span>
    </label>
    <label class="check">
      <input id="straight-hold" type="checkbox" checked>
      Lane following via encoder difference
      <span id="straight-hold-label" class="state">On</span>
    </label>
    <p class="note">On while driving straight: the wheel that has travelled further is slowed. Turning leaves this off.</p>
    <div class="pad">
      <span></span>
      <button type="button" data-dir="forward">Forward</button>
      <span></span>
      <button type="button" data-dir="left">Left</button>
      <button type="button" id="stop">Stop</button>
      <button type="button" data-dir="right">Right</button>
      <span></span>
      <button type="button" data-dir="back">Back</button>
      <span></span>
    </div>
    <p class="hint">Hold a button, or hold WASD / the arrow keys. Releasing stops the car.</p>
  </div>

  <h2>Output</h2>
  <pre id="log"></pre>
  <p class="hint">Space bar or Esc = Stop. Closing this page stops a self-driving mode within a few seconds.</p>
</main>
<script>
const held = new Set();
let manual = false;
let modes = [];
let selected = null;
let running = null;
let logNext = 0;
const speed = document.getElementById("speed");
const speedLabel = document.getElementById("speed-label");
const correction = document.getElementById("correction");
const correctionLabel = document.getElementById("correction-label");
const straightHold = document.getElementById("straight-hold");
const straightHoldLabel = document.getElementById("straight-hold-label");
const vision = document.getElementById("vision");
const visionLabel = document.getElementById("vision-label");
const visionGain = document.getElementById("vision-gain");
const visionGainLabel = document.getElementById("vision-gain-label");
const rear1 = document.getElementById("rear1");
const rear2 = document.getElementById("rear2");
const head = document.getElementById("head");
const link = document.getElementById("link");
const novideo = document.getElementById("novideo");
const modesEl = document.getElementById("modes");
const help = document.getElementById("help");
const stateEl = document.getElementById("state");
const detailEl = document.getElementById("detail");
const sensorsEl = document.getElementById("sensors");
const logEl = document.getElementById("log");
const runBtn = document.getElementById("run");
const haltBtn = document.getElementById("halt");
const laneLeft = document.getElementById("lane-left");
const laneRight = document.getElementById("lane-right");
const keys = {ArrowUp:"forward", w:"forward", W:"forward",
              ArrowDown:"back", s:"back", S:"back",
              ArrowLeft:"left", a:"left", A:"left",
              ArrowRight:"right", d:"right", D:"right"};

async function post(path, body) {
  const res = await fetch(path, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(body || {})
  });
  return res.json();
}

function labelOf(name) {
  const m = modes.find((x) => x.name === name);
  return m ? m.label : name;
}

function drawModes() {
  if (modesEl.children.length !== modes.length) {
    modesEl.innerHTML = "";
    modes.forEach((m) => {
      const b = document.createElement("button");
      b.type = "button";
      b.textContent = m.label;
      b.dataset.mode = m.name;
      b.addEventListener("click", async () => {
        held.clear(); mark();
        apply(await post("/mode", {mode: m.name}));
      });
      modesEl.appendChild(b);
    });
  }
  modesEl.querySelectorAll("button").forEach((b) => {
    b.classList.toggle("selected", b.dataset.mode === selected);
    b.classList.toggle("running", b.dataset.mode === running);
  });
  const m = modes.find((x) => x.name === selected);
  help.textContent = m ? m.help : "";
  help.classList.toggle("warn", !!m && m.help.startsWith("PROP"));
}

function apply(data) {
  if (!data || !data.modes) return;
  modes = data.modes;
  selected = data.selected;
  running = data.running;
  manual = running === "web";
  drawModes();
  document.getElementById("manual").classList.toggle("hidden", !manual);
  document.getElementById("lamps").classList.toggle("hidden", running !== "light_test");
  document.getElementById("lanes").classList.toggle("hidden", running !== "drive");
  showLanes(data);
  runBtn.disabled = running !== null;
  haltBtn.disabled = running === null;
  stateEl.classList.toggle("bad", data.result && data.result.startsWith("error"));
  if (running) {
    stateEl.textContent = "Running: " + labelOf(running);
  } else {
    stateEl.textContent = "Selected: " + labelOf(selected) +
      (data.result ? " (last run " + data.result + ")" : "") + ". Press Run.";
  }
  detailEl.textContent = running ? (data.detail || "") : "";
}

function mark() {
  document.querySelectorAll("button[data-dir]").forEach((button) => {
    button.classList.toggle("held", held.has(button.dataset.dir));
  });
}

function speeds() {
  const s = Number(speed.value);
  const fwd = (held.has("forward") ? 1 : 0) - (held.has("back") ? 1 : 0);
  const turn = (held.has("right") ? 1 : 0) - (held.has("left") ? 1 : 0);
  let left = (fwd + turn) * s;
  let right = (fwd - turn) * s;
  const peak = Math.max(Math.abs(left), Math.abs(right));
  if (peak > s) {
    left *= s / peak;
    right *= s / peak;
  }
  // Straight driving only. Slider left slows the left wheel, so the car pulls left.
  if (turn === 0) {
    const c = Number(correction.value) / 100;
    left *= 1 - Math.max(0, -c);
    right *= 1 - Math.max(0, c);
  }
  return {left: left, right: right};
}

function show(data, ok) {
  link.classList.toggle("bad", !ok || data.blocked);
  if (!ok) {
    link.textContent = "No connection to the Uno Q";
    return;
  }
  const l = data.left.toFixed(2);
  const r = data.right.toFixed(2);
  link.textContent = data.blocked
    ? "Front is blocked. Reverse still works. Wheels " + l + " / " + r + " m/s"
    : "Wheels " + l + " / " + r + " m/s";
}

async function send() {
  if (!manual) return;
  try {
    const res = await fetch("/drive", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify(speeds())
    });
    show(await res.json(), res.ok);
  } catch (err) {
    show(null, false);
  }
}

function showSpeed() {
  speedLabel.textContent = Number(speed.value).toFixed(2) + " m/s";
}
const savedSpeed = Number(localStorage.getItem("speed"));
if (savedSpeed >= 0.05 && savedSpeed <= 0.57) speed.value = String(savedSpeed);
showSpeed();
function postSpeed() {
  localStorage.setItem("speed", speed.value);
  showSpeed();
  post("/speed", {speed: Number(speed.value)}).catch(() => {});
}
speed.addEventListener("input", postSpeed);

function showSensors(data) {
  const tof = typeof data.tof_m === "number" && data.tof_m >= 0
    ? data.tof_m.toFixed(2) + " m" : "no reading";
  const level = typeof data.lux === "number" && data.lux >= 0
    ? Math.round(data.lux) + " lux" : "no reading";
  sensorsEl.textContent = "ToF " + tof + " | " + level + " | lights " + (data.lights_on ? "on" : "off");
}

function showCorrection() {
  const c = Number(correction.value);
  correctionLabel.textContent = c === 0 ? "Off" : (c < 0 ? "Left " : "Right ") + Math.abs(c) + "%";
}
correction.value = localStorage.getItem("correction") || "0";
showCorrection();
correction.addEventListener("input", () => {
  localStorage.setItem("correction", correction.value);
  showCorrection();
});

function showStraightHold() {
  straightHoldLabel.textContent = straightHold.checked ? "On" : "Off";
}
const savedHold = localStorage.getItem("straightHold");
straightHold.checked = savedHold === null ? true : savedHold === "1";
showStraightHold();
straightHold.addEventListener("change", () => {
  localStorage.setItem("straightHold", straightHold.checked ? "1" : "0");
  showStraightHold();
  post("/straight-hold", {enabled: straightHold.checked}).catch(() => {});
});

function showVision() {
  visionLabel.textContent = vision.checked ? "On" : "Off";
  visionGain.disabled = !vision.checked;
  visionGainLabel.textContent = Number(visionGain.value).toFixed(1) + "×";
}
const savedVision = localStorage.getItem("vision");
vision.checked = savedVision === null ? true : savedVision === "1";
const savedGain = localStorage.getItem("visionGain");
if (savedGain !== null && savedGain !== "") visionGain.value = savedGain;
showVision();
function postVision() {
  localStorage.setItem("vision", vision.checked ? "1" : "0");
  localStorage.setItem("visionGain", visionGain.value);
  showVision();
  post("/vision", {enabled: vision.checked, gain: Number(visionGain.value)}).catch(() => {});
}
vision.addEventListener("change", postVision);
visionGain.addEventListener("input", postVision);

function showLanes(data) {
  const left = running === "drive" && !!data.lane_left;
  const right = running === "drive" && !!data.lane_right;
  laneLeft.disabled = !left;
  laneRight.disabled = !right;
  laneLeft.textContent = left ? "Left: available" : "Left: not available";
  laneRight.textContent = right ? "Right: available" : "Right: not available";
}
function changeLane(direction) {
  const button = direction === "left" ? laneLeft : laneRight;
  if (button.disabled) return;
  post("/lane-change", {direction: direction}).catch(() => {});
}
laneLeft.addEventListener("click", () => changeLane("left"));
laneRight.addEventListener("click", () => changeLane("right"));

function postLamps() {
  post("/lamps", {rear1: rear1.checked, rear2: rear2.checked, head: head.checked}).catch(() => {});
}
rear1.addEventListener("change", postLamps);
rear2.addEventListener("change", postLamps);
head.addEventListener("change", postLamps);

document.querySelectorAll("button[data-dir]").forEach((button) => {
  button.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    try { button.setPointerCapture(event.pointerId); } catch (err) {}
    held.add(button.dataset.dir);
    mark();
    send();
  });
  const release = () => {
    held.delete(button.dataset.dir);
    mark();
    send();
  };
  button.addEventListener("pointerup", release);
  button.addEventListener("pointercancel", release);
});

document.getElementById("stop").addEventListener("pointerdown", (event) => {
  event.preventDefault();
  held.clear();
  mark();
  send();
});

runBtn.addEventListener("click", async () => apply(await post("/run")));
async function haltAll() {
  held.clear(); mark(); send();
  apply(await post("/stop"));
}
haltBtn.addEventListener("click", haltAll);

window.addEventListener("keydown", (event) => {
  if (event.target === speed || event.target === correction || event.target === straightHold ||
      event.target === vision || event.target === visionGain ||
      event.target === rear1 || event.target === rear2 || event.target === head) return;
  if (event.key === " " || event.key === "Escape") {
    event.preventDefault();
    if (manual) {
      held.clear(); mark(); send();
    } else if (running) {
      haltAll();
    }
    return;
  }
  const dir = keys[event.key];
  if (!dir || event.repeat || !manual) return;
  event.preventDefault();
  held.add(dir);
  mark();
  send();
});
window.addEventListener("keyup", (event) => {
  const dir = keys[event.key];
  if (!dir) return;
  held.delete(dir);
  mark();
  send();
});
window.addEventListener("blur", () => {
  held.clear();
  mark();
  send();
});

setInterval(send, 200);

async function refresh() {
  try {
    const res = await fetch("/status?log=" + logNext);
    const data = await res.json();
    novideo.classList.toggle("hidden", data.camera);
    apply(data);
    showSensors(data);
    if (typeof data.straight_hold === "boolean" && data.straight_hold !== straightHold.checked) {
      post("/straight-hold", {enabled: straightHold.checked}).catch(() => {});
    }
    if (typeof data.vision === "boolean" && typeof data.vision_gain === "number" &&
        (data.vision !== vision.checked || Math.abs(data.vision_gain - Number(visionGain.value)) > 0.001)) {
      postVision();
    }
    if (typeof data.speed === "number" && Math.abs(data.speed - Number(speed.value)) > 0.001) {
      postSpeed();
    }
    if (running === "light_test" && data.lamps &&
        (data.lamps.rear1 !== rear1.checked || data.lamps.rear2 !== rear2.checked ||
         data.lamps.head !== head.checked)) {
      postLamps();
    }
    if (data.log.length) {
      const atBottom = logEl.scrollTop + logEl.clientHeight >= logEl.scrollHeight - 4;
      logEl.textContent += data.log.join("\n") + "\n";
      if (logEl.textContent.length > 60000) {
        logEl.textContent = logEl.textContent.slice(-40000);
      }
      if (atBottom) logEl.scrollTop = logEl.scrollHeight;
    }
    logNext = data.log_next;
  } catch (err) {
    stateEl.textContent = "No connection to the Uno Q";
    stateEl.classList.add("bad");
    novideo.classList.remove("hidden");
  }
}
setInterval(refresh, 400);
refresh();
</script>
</body>
</html>
"""


def _clamp_speed(value):
    try:
        speed = float(value)
    except (TypeError, ValueError):
        return None
    if speed != speed or speed in (float("inf"), float("-inf")):
        return None
    limit = config.WEB_MAX_SPEED
    return max(-limit, min(limit, speed))


def _front_m():
    """Nearest front lidar distance in meters, or None if there is no reading."""
    front = hardware.lidar_stats["front"]
    if isinstance(front, float) and (front != front or front == float("inf") or front == float("-inf")):
        return None
    return front


def _tof_m():
    """VL53L1X distance in meters, or None if the sensor has no reading."""
    try:
        distance = float(hardware.sensors.get("tof_m"))
    except (TypeError, ValueError):
        return None
    if distance < 0 or distance != distance or distance in (float("inf"), float("-inf")):
        return None
    return distance


def _front_blocked():
    front = _front_m()
    tof = _tof_m()
    lidar_blocked = front is not None and front < config.ESTOP_DISTANCE_M
    tof_blocked = tof is not None and tof < config.ESTOP_DISTANCE_M
    return lidar_blocked or tof_blocked


def _apply(left, right):
    """Command both wheels. Positive speed is forward. A close obstacle
    drops any forward component so the car can still back away."""
    blocked = _front_blocked()
    if blocked:
        left = min(left, 0.0)
        right = min(right, 0.0)
    _command["left"] = left
    _command["right"] = right
    _command["blocked"] = blocked
    if left == 0.0 and right == 0.0:
        car_link.stop()
    else:
        car_link.set_wheel_speeds(left, right)
    return {"left": left, "right": right, "blocked": blocked}


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        path = self.path.split("?", 1)[0]
        if path in ("/drive", "/status", "/stream.mjpg", "/mode", "/run", "/stop",
                    "/straight-hold", "/vision", "/speed", "/lamps", "/lane-change"):
            return
        super().log_message(fmt, *args)

    def _send(self, code, body, content_type, extra=None):
        data = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        if extra:
            for key, value in extra:
                self.send_header(key, value)
        self.end_headers()
        self.wfile.write(data)

    def _json(self, code, obj):
        self._send(code, json.dumps(obj), "application/json")

    def do_GET(self):
        url = urlparse(self.path)
        try:
            if url.path == "/":
                self._send(200, _PAGE, "text/html; charset=utf-8")
            elif url.path == "/status":
                modes.heartbeat()
                try:
                    since = int(parse_qs(url.query).get("log", ["0"])[0])
                except ValueError:
                    since = 0
                self._json(200, self._status(since))
            elif url.path == "/stream.mjpg":
                self._stream()
            else:
                self._send(404, "Not found\n", "text/plain; charset=utf-8")
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _body(self):
        length = int(self.headers.get("Content-Length", "0"))
        if length < 0 or length > 200:
            raise ValueError("body too large")
        payload = json.loads(self.rfile.read(length) or b"{}")
        if not isinstance(payload, dict):
            raise ValueError("expected an object")
        return payload

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        try:
            if path == "/drive":
                self._drive()
            elif path == "/mode":
                try:
                    modes.select(self._body().get("mode"))
                except (ValueError, json.JSONDecodeError, UnicodeDecodeError) as exc:
                    self._json(400, {"error": str(exc)})
                    return
                _apply(0.0, 0.0)
                self._json(200, self._status(None))
            elif path == "/run":
                modes.run()
                self._json(200, self._status(None))
            elif path == "/stop":
                modes.stop()
                _apply(0.0, 0.0)
                self._json(200, self._status(None))
            elif path == "/straight-hold":
                self._straight_hold()
            elif path == "/vision":
                self._vision()
            elif path == "/speed":
                self._speed()
            elif path == "/lamps":
                self._lamps()
            elif path == "/lane-change":
                self._lane_change()
            else:
                self._send(404, "Not found\n", "text/plain; charset=utf-8")
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _drive(self):
        if not modes.manual_active():
            # Another mode owns the wheels. Don't touch them.
            self._json(409, {"error": "manual driving is not running",
                             "left": 0.0, "right": 0.0, "blocked": False})
            return
        try:
            payload = self._body()
            left = _clamp_speed(payload.get("left"))
            right = _clamp_speed(payload.get("right"))
        except (ValueError, json.JSONDecodeError, UnicodeDecodeError):
            left = right = None
        if left is None or right is None:
            _apply(0.0, 0.0)
            self._json(400, _command)
            return
        self._json(200, _apply(left, right))

    def _straight_hold(self):
        try:
            enabled = self._body().get("enabled")
            if not isinstance(enabled, bool):
                raise ValueError("enabled must be true or false")
        except (ValueError, json.JSONDecodeError, UnicodeDecodeError) as exc:
            self._json(400, {"error": str(exc)})
            return
        car_link.set_straight_hold(enabled)
        self._json(200, {"straight_hold": car_link.straight_hold_enabled()})

    def _vision(self):
        try:
            payload = self._body()
            enabled = payload.get("enabled")
            gain = float(payload.get("gain"))
            if not isinstance(enabled, bool) or gain != gain or gain in (float("inf"), float("-inf")):
                raise ValueError("enabled must be true or false, and gain a number")
        except (TypeError, ValueError, json.JSONDecodeError, UnicodeDecodeError) as exc:
            self._json(400, {"error": str(exc)})
            return
        config.VISION_CONTROL = enabled
        config.VISION_GAIN = max(0.0, min(3.0, gain))
        self._json(200, {"vision": config.VISION_CONTROL, "vision_gain": config.VISION_GAIN})

    def _speed(self):
        try:
            speed = float(self._body().get("speed"))
            if speed != speed or speed in (float("inf"), float("-inf")):
                raise ValueError("speed must be a number")
        except (TypeError, ValueError, json.JSONDecodeError, UnicodeDecodeError) as exc:
            self._json(400, {"error": str(exc)})
            return
        speed = max(0.05, min(config.WEB_MAX_SPEED, speed))
        config.CRUISE_SPEED = speed
        config.SLOW_SPEED = speed / 2.0
        self._json(200, {"speed": config.CRUISE_SPEED})

    def _lamps(self):
        if modes.status()["running"] != "light_test":
            self._json(409, {"error": "light test is not running"})
            return
        try:
            payload = self._body()
            rear1 = payload.get("rear1")
            rear2 = payload.get("rear2")
            head = payload.get("head")
            if not all(isinstance(flag, bool) for flag in (rear1, rear2, head)):
                raise ValueError("rear1, rear2, and head must be true or false")
        except (TypeError, ValueError, json.JSONDecodeError, UnicodeDecodeError) as exc:
            self._json(400, {"error": str(exc)})
            return
        hardware.set_lamps(rear1, rear2, head)
        self._json(200, hardware.lamp_state())

    def _lane_change(self):
        try:
            direction = self._body().get("direction")
            if direction not in ("left", "right"):
                raise ValueError("direction must be left or right")
        except (TypeError, ValueError, json.JSONDecodeError, UnicodeDecodeError) as exc:
            self._json(400, {"error": str(exc)})
            return
        if not modes.request_lane_change(direction):
            self._json(409, {"error": "that lane is not available"})
            return
        self._json(200, self._status(None))

    def _status(self, since):
        out = modes.status()
        out.update({
            "camera": vision.camera_ok(),
            "left": _command["left"],
            "right": _command["right"],
            "blocked": _command["blocked"] or _front_blocked(),
            "estop": bool(hardware.telemetry.get("estop")),
            "front": _front_m(),
            "tof_m": hardware.sensors["tof_m"],
            "lux": hardware.sensors["lux"],
            "lights_on": bool(hardware.sensors["lights_on"]),
            "straight_hold": car_link.straight_hold_enabled(),
            "vision": bool(config.VISION_CONTROL),
            "vision_gain": float(config.VISION_GAIN),
            "speed": float(config.CRUISE_SPEED),
            "lamps": hardware.lamp_state(),
        })
        lane_left, lane_right = modes.lane_availability()
        out["lane_left"] = lane_left
        out["lane_right"] = lane_right
        if since is not None:
            lines, next_id = modes.log.since(since)
            out["log"] = [line for _, line in lines]
            out["log_next"] = next_id
        return out

    def _stream(self):
        self.send_response(200)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        last = None
        while True:
            _, _, _, jpeg = vision.latest()
            if jpeg is not None and jpeg is not last:
                last = jpeg
                self.wfile.write(
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                    + jpeg + b"\r\n"
                )
                self.wfile.flush()
            else:
                time.sleep(0.03)


def _urls(port):
    found = [f"http://{config.CAR_IP}:{port}", f"http://car.local:{port}"]
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        ip = sock.getsockname()[0]
        sock.close()
        if ip and not ip.startswith("127.") and ip != config.CAR_IP:
            print(f"Note: the Uno Q is on {ip} now, not CAR_IP {config.CAR_IP}.", flush=True)
            found.append(f"http://{ip}:{port}")
    except OSError:
        pass
    return found


def serve(port=None):
    """Serve the page until Ctrl+C. The wheels are stopped on the way out."""
    port = config.WEB_PORT if port is None else port
    vision.start()
    modes.start_watchdog()
    server = ThreadingHTTPServer(("0.0.0.0", port), _Handler)
    server.daemon_threads = True
    print("Car control page:", flush=True)
    for url in _urls(port):
        print(f"  {url}", flush=True)
    print("Pick a mode, press Run. Ctrl+C here quits.", flush=True)
    try:
        server.serve_forever()
    finally:
        modes.stop()
        _apply(0.0, 0.0)
        server.server_close()

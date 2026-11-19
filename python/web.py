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
  .lanebar { display: flex; align-items: center; gap: 10px; margin: 10px 0 0; }
  .lanebar .title { color: #c8c2b2; font-size: 15px; font-weight: 600; }
  .lanes3 { display: flex; flex: 1; max-width: 320px; height: 34px;
            border-radius: 6px; overflow: hidden; background: #26291e; }
  .lanes3 .tape { width: 5px; flex: none; }
  .lanes3 .tape.red { background: #d23b2a; }
  .lanes3 .tape.yellow { background: #e0b03a; }
  .lanes3 .cell { flex: 1; display: flex; align-items: center; justify-content: center;
                  font-size: 13px; color: #9a947f; }
  .lanes3 .cell.here { background: #3d5a1e; color: #ece7da; font-weight: 600; }
  .lanes3 .cell.here.unsure { background: #4d4a1e; }
  #lane-text { color: #c8c2b2; font-size: 14px; min-width: 7em; }
  details { margin: 12px 0 0; }
  summary { cursor: pointer; color: #c8c2b2; font-weight: 600; font-size: 15px; }
  img.birdseye { display: block; margin: 8px auto 0; max-width: 100%; height: auto;
                 image-rendering: pixelated; background: #000; }
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
  <div class="lanebar">
    <span class="title">Lane</span>
    <div class="lanes3" id="lane-diagram" aria-label="Which lane the camera sees">
      <span class="tape red"></span><span class="cell" data-lane="0">1</span>
      <span class="tape yellow"></span><span class="cell" data-lane="1">2</span>
      <span class="tape yellow"></span><span class="cell" data-lane="2">3</span>
      <span class="tape red"></span>
    </div>
    <span id="lane-text">No lane</span>
  </div>

  <details id="birdseye-panel">
    <summary>Top-down view</summary>
    <img id="birdseye" class="birdseye" alt="Top-down view of the floor ahead">
    <p class="note">The camera image mapped onto the floor. Grid squares are 10 cm; the green box is the car. Straight tape should show as two parallel lines one lane width apart. The scale comes from LANE_WIDTH_M, so it is only as right as that number.</p>
  </details>

  <label class="check">
    <input id="hue-enabled" type="checkbox" disabled>
    Hue calibration
  </label>
  <label class="speed">Hue offset
    <input id="hue-offset" type="range" min="-90" max="90" step="1" value="0" disabled>
    <span id="hue-value">0°</span>
  </label>
  <p id="hue-status" class="note" role="status">Adjust while parked until the preview highlights both tapes.</p>

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
  <label class="check">
    <input id="all-lights" type="checkbox" disabled>
    Force all lights on
  </label>
  <p id="all-lights-error" class="bad" role="status" hidden></p>

  <label class="speed">Speed
    <input id="speed" type="range" min="0.05" max="0.75" step="0.01" value="0.30">
    <span id="speed-label">0.30 m/s</span>
  </label>
  <p class="note">Manual driving, lane following, and the straight-line test use this speed. A close obstacle uses half of it.</p>
  <label class="check">
    <input id="speed-control" type="checkbox" checked>
    Wheel speed control (encoders)
    <span id="speed-control-label" class="state">On</span>
  </label>
  <p class="note">On: each wheel's encoder corrects its power to hold the commanded speed. Off: power comes from the speed alone, so the wheels can run at different speeds. Applies to every driving mode.</p>

  <h2>Vision lane following</h2>
  <label class="check">
    <input id="vision" type="checkbox" checked>
    Vision control
    <span id="vision-label" class="state">On</span>
  </label>
  <label class="speed">Strength
    <input id="vision-gain" type="range" min="0" max="1.5" step="0.05" value="1.0">
    <span id="vision-gain-label">1.0×</span>
  </label>
  <p class="note">1.0 is normal strength. Lower if it zigzags; raise only if it under-steers. Off drives straight and ignores the lane lines.</p>

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
    <p class="hint">Hold a button, or hold WASD / the arrow keys. Releasing stops the car.
      Left/Right alone spins in place (one wheel forward, one reverse). Hold Forward with Left/Right to pivot while rolling.</p>

    <h2>Breakaway / PWM test</h2>
    <p class="note">Raw motor power with no speed control. Pick a wheel, set the duty, and hold Spin.
      Raise the slider from a full stop until that wheel just starts to turn. Do this on the foam
      for the real breakaway number; in the air is fine for checking a motor. The code is the same either way.</p>
    <label class="speed">Wheel
      <select id="break-wheel">
        <option value="left">Left</option>
        <option value="right">Right</option>
        <option value="both">Both</option>
      </select>
    </label>
    <label class="speed">Duty
      <input id="break-duty" type="range" min="0" max="100" step="1" value="0">
      <span id="break-duty-label">0% (PWM 0)</span>
    </label>
    <button type="button" id="break-spin">Hold to spin</button>
    <p id="break-link" class="hint">Release to stop. PWM_OFFSET in the sketch is roughly this PWM value for the slower wheel.</p>
  </div>

  <h2>Output</h2>
  <pre id="log"></pre>
  <p class="hint">Space bar or Esc = Stop. Closing this page stops a self-driving mode within a few seconds.</p>
</main>
<script>
const held = new Set();
let manual = false;
let breakSpinning = false;
let modes = [];
let selected = null;
let running = null;
let logNext = 0;
const speed = document.getElementById("speed");
const speedLabel = document.getElementById("speed-label");
const correction = document.getElementById("correction");
const correctionLabel = document.getElementById("correction-label");
const speedControl = document.getElementById("speed-control");
const speedControlLabel = document.getElementById("speed-control-label");
const straightHold = document.getElementById("straight-hold");
const straightHoldLabel = document.getElementById("straight-hold-label");
const breakWheel = document.getElementById("break-wheel");
const breakDuty = document.getElementById("break-duty");
const breakDutyLabel = document.getElementById("break-duty-label");
const breakSpin = document.getElementById("break-spin");
const breakLink = document.getElementById("break-link");
const vision = document.getElementById("vision");
const visionLabel = document.getElementById("vision-label");
const visionGain = document.getElementById("vision-gain");
const visionGainLabel = document.getElementById("vision-gain-label");
const rear1 = document.getElementById("rear1");
const rear2 = document.getElementById("rear2");
const head = document.getElementById("head");
const allLights = document.getElementById("all-lights");
const allLightsError = document.getElementById("all-lights-error");
let savingAllLights = false;
const link = document.getElementById("link");
const novideo = document.getElementById("novideo");
const hueEnabled = document.getElementById("hue-enabled");
const hueOffset = document.getElementById("hue-offset");
const hueValue = document.getElementById("hue-value");
const hueStatus = document.getElementById("hue-status");
let savedHue = null;
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
const laneCells = Array.from(document.querySelectorAll("#lane-diagram .cell"));
const laneText = document.getElementById("lane-text");
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
  if (!manual && breakSpinning) {
    // Mode change already stops the motors; only clear the UI latch here.
    breakSpinning = false;
    breakSpin.classList.remove("held");
  }
  drawModes();
  document.getElementById("manual").classList.toggle("hidden", !manual);
  document.getElementById("lamps").classList.toggle("hidden", running !== "light_test");
  document.getElementById("lanes").classList.toggle("hidden", running !== "drive");
  showLanes(data);
  showCurrentLane(data.lane);
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

// Tank steer: Left/Right alone spins in place (one wheel forward, one reverse).
// Forward+Left/Right pivots around the inner wheel.
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

function breakDuties() {
  const duty = Number(breakDuty.value);
  const wheel = breakWheel.value;
  if (wheel === "left") return {left: duty, right: 0};
  if (wheel === "right") return {left: 0, right: duty};
  return {left: duty, right: duty};
}

function showBreak(data, ok) {
  if (!ok || !data) {
    breakLink.textContent = "No connection to the Uno Q";
    breakLink.classList.add("bad");
    return;
  }
  breakLink.classList.remove("bad");
  const pwmL = Math.round(Math.abs(data.left) * 255 / 100);
  const pwmR = Math.round(Math.abs(data.right) * 255 / 100);
  breakLink.textContent = breakSpinning
    ? ("Spinning raw duty L " + data.left + "% (PWM " + pwmL +
       ") / R " + data.right + "% (PWM " + pwmR + "). Release to stop.")
    : "Release to stop. PWM_OFFSET in the sketch is roughly this PWM value for the slower wheel.";
}

async function send() {
  if (!manual) return;
  // Breakaway test owns the motors while Spin is held.
  if (breakSpinning) {
    try {
      const res = await fetch("/duty", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(breakDuties())
      });
      showBreak(await res.json(), res.ok);
    } catch (err) {
      showBreak(null, false);
    }
    return;
  }
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
if (savedSpeed >= 0.05 && savedSpeed <= 0.75) speed.value = String(savedSpeed);
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

const birdseyePanel = document.getElementById("birdseye-panel");
const birdseyeImg = document.getElementById("birdseye");
function showBirdseye() {
  // Only stream while open: the car draws the view only while someone watches.
  if (birdseyePanel.open) birdseyeImg.src = "/birdseye.mjpg";
  else birdseyeImg.removeAttribute("src");
}
birdseyePanel.open = localStorage.getItem("birdseye") === "1";
showBirdseye();
birdseyePanel.addEventListener("toggle", () => {
  localStorage.setItem("birdseye", birdseyePanel.open ? "1" : "0");
  showBirdseye();
});

function showBreakDuty() {
  const duty = Number(breakDuty.value);
  const pwm = Math.round(duty * 255 / 100);
  breakDutyLabel.textContent = duty + "% (PWM " + pwm + ")";
}
breakDuty.value = localStorage.getItem("breakDuty") || "0";
showBreakDuty();
breakDuty.addEventListener("input", () => {
  localStorage.setItem("breakDuty", breakDuty.value);
  showBreakDuty();
  if (breakSpinning) send();
});
breakWheel.value = localStorage.getItem("breakWheel") || "left";
breakWheel.addEventListener("change", () => {
  localStorage.setItem("breakWheel", breakWheel.value);
  if (breakSpinning) send();
});
function setBreakSpinning(on) {
  breakSpinning = on;
  breakSpin.classList.toggle("held", on);
  if (on) {
    held.clear();
    mark();
  }
  send();
}
breakSpin.addEventListener("pointerdown", (event) => {
  event.preventDefault();
  if (!manual) return;
  breakSpin.setPointerCapture(event.pointerId);
  setBreakSpinning(true);
});
function endBreakSpin() {
  if (!breakSpinning) return;
  setBreakSpinning(false);
}
breakSpin.addEventListener("pointerup", endBreakSpin);
breakSpin.addEventListener("pointercancel", endBreakSpin);
breakSpin.addEventListener("lostpointercapture", endBreakSpin);

function showSpeedControl() {
  speedControlLabel.textContent = speedControl.checked ? "On" : "Off";
}
const savedSpeedControl = localStorage.getItem("speedControl");
speedControl.checked = savedSpeedControl === null ? true : savedSpeedControl === "1";
showSpeedControl();
speedControl.addEventListener("change", () => {
  localStorage.setItem("speedControl", speedControl.checked ? "1" : "0");
  showSpeedControl();
  post("/speed-control", {enabled: speedControl.checked}).catch(() => {});
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
let visionSynced = false;  // first /status wins so an old localStorage gain cannot overwrite the car
const savedVision = localStorage.getItem("vision");
vision.checked = savedVision === null ? true : savedVision === "1";
const savedGain = localStorage.getItem("visionGain");
if (savedGain !== null && savedGain !== "") {
  const g = Number(savedGain);
  // Old sessions saved gains up to 3×; those weave. Cap what we restore.
  visionGain.value = String(Math.max(0, Math.min(1.5, g > 1.5 ? 0.5 : g)));
}
showVision();
function postVision() {
  localStorage.setItem("vision", vision.checked ? "1" : "0");
  localStorage.setItem("visionGain", visionGain.value);
  showVision();
  post("/vision", {enabled: vision.checked, gain: Number(visionGain.value)}).catch(() => {});
}
vision.addEventListener("change", postVision);
visionGain.addEventListener("input", postVision);

function showHue(settings) {
  savedHue = settings;
  hueEnabled.checked = settings.enabled;
  hueOffset.value = settings.offset;
  hueEnabled.disabled = false;
  hueOffset.disabled = !settings.enabled;
  hueValue.textContent = (2 * Number(hueOffset.value)) + "°";
}

async function saveHue() {
  hueEnabled.disabled = true;
  hueOffset.disabled = true;
  try {
    const settings = await post("/hue-calibration", {
      enabled: hueEnabled.checked, offset: Number(hueOffset.value)
    });
    if (settings.error) throw new Error(settings.error);
    showHue(settings);
    hueStatus.textContent = "Saved on the car.";
  } catch (error) {
    showHue(savedHue);
    hueStatus.textContent = "Could not save hue calibration. " + error.message;
  }
}
hueEnabled.addEventListener("change", saveHue);
hueOffset.addEventListener("input", () => {
  hueValue.textContent = (2 * Number(hueOffset.value)) + "°";
});
hueOffset.addEventListener("change", saveHue);

function showCurrentLane(lane) {
  // lane: {index, confirmed, left, right} from the camera, or null.
  laneCells.forEach((cell) => {
    const here = !!lane && Number(cell.dataset.lane) === lane.index;
    cell.classList.toggle("here", here);
    cell.classList.toggle("unsure", here && !lane.confirmed);
  });
  if (!lane) {
    laneText.textContent = "No lane";
    return;
  }
  const names = ["Left", "Middle", "Right"];
  const seen = (lane.left || "-") + " | " + (lane.right || "-");
  let where = "";
  if (typeof lane.pos === "number") {
    // Position inside the lane: 0% at its left line, 100% at its right line.
    const frac = Math.min(1, Math.max(0, lane.pos - lane.index));
    where = " · " + Math.round(frac * 100) + "% across";
  }
  laneText.textContent = (names[lane.index] || ("Lane " + (lane.index + 1))) +
    (lane.confirmed ? "" : " (unsure)") + where + " · " + seen;
}
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

allLights.addEventListener("change", async () => {
  const enabled = allLights.checked;
  savingAllLights = true;
  allLights.disabled = true;
  allLightsError.hidden = true;
  try {
    const data = await post("/all-lights", {enabled});
    if (data.all_lights !== enabled) throw new Error("Light override failed");
  } catch (err) {
    allLights.checked = !enabled;
    allLightsError.textContent = "Could not update the lights.";
    allLightsError.hidden = false;
  } finally {
    savingAllLights = false;
    allLights.disabled = false;
  }
});

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
  if (breakSpinning) setBreakSpinning(false);
  else send();
});

runBtn.addEventListener("click", async () => apply(await post("/run")));
async function haltAll() {
  held.clear(); mark();
  if (breakSpinning) setBreakSpinning(false);
  else send();
  apply(await post("/stop"));
}
haltBtn.addEventListener("click", haltAll);

window.addEventListener("keydown", (event) => {
  if (event.target === speed || event.target === correction || event.target === straightHold ||
      event.target === speedControl ||
      event.target === vision || event.target === visionGain ||
      event.target === hueEnabled || event.target === hueOffset ||
      event.target === rear1 || event.target === rear2 || event.target === head ||
      event.target === breakWheel || event.target === breakDuty || event.target === breakSpin) return;
  if (breakSpinning) return;
  if (event.key === " " || event.key === "Escape") {
    event.preventDefault();
    if (manual) {
      held.clear(); mark();
      if (breakSpinning) setBreakSpinning(false);
      else send();
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
  if (breakSpinning) setBreakSpinning(false);
  else send();
});

setInterval(send, 200);

async function refresh() {
  try {
    const res = await fetch("/status?log=" + logNext);
    const data = await res.json();
    novideo.classList.toggle("hidden", data.camera);
    apply(data);
    showSensors(data);
    if (savedHue === null && data.hue_calibration) showHue(data.hue_calibration);
    if (!savingAllLights && typeof data.all_lights === "boolean") {
      allLights.checked = data.all_lights;
      allLights.disabled = false;
    }
    if (typeof data.speed_control === "boolean" && data.speed_control !== speedControl.checked) {
      post("/speed-control", {enabled: speedControl.checked}).catch(() => {});
    }
    if (typeof data.straight_hold === "boolean" && data.straight_hold !== straightHold.checked) {
      post("/straight-hold", {enabled: straightHold.checked}).catch(() => {});
    }
    // Adopt the car's vision settings once. Do NOT push the page's saved gain
    // onto the car every refresh — that was restoring 2–3× and causing weave.
    if (!visionSynced && typeof data.vision === "boolean" && typeof data.vision_gain === "number") {
      vision.checked = data.vision;
      visionGain.value = String(Math.max(0, Math.min(1.5, data.vision_gain)));
      localStorage.setItem("vision", vision.checked ? "1" : "0");
      localStorage.setItem("visionGain", visionGain.value);
      showVision();
      visionSynced = true;
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


def _clamp_duty(value):
    """Duty cycle percent for set_duty, -100..100."""
    try:
        duty = float(value)
    except (TypeError, ValueError):
        return None
    if duty != duty or duty in (float("inf"), float("-inf")):
        return None
    return max(-100.0, min(100.0, duty))


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
        if path in ("/drive", "/duty", "/status", "/stream.mjpg", "/birdseye.mjpg", "/mode", "/run",
                    "/stop", "/straight-hold", "/speed-control", "/vision", "/speed", "/lamps",
                    "/all-lights", "/lane-change"):
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
                self._stream(lambda: vision.latest()[3])
            elif url.path == "/birdseye.mjpg":
                self._stream(vision.latest_birdseye)
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
            elif path == "/duty":
                self._duty()
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
            elif path == "/speed-control":
                self._speed_control()
            elif path == "/vision":
                self._vision()
            elif path == "/hue-calibration":
                self._hue_calibration()
            elif path == "/speed":
                self._speed()
            elif path == "/lamps":
                self._lamps()
            elif path == "/all-lights":
                self._all_lights()
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

    def _duty(self):
        """Raw PWM duty in percent for the breakaway test. Manual mode only."""
        if not modes.manual_active():
            self._json(409, {"error": "manual driving is not running",
                             "left": 0, "right": 0})
            return
        try:
            payload = self._body()
            left = _clamp_duty(payload.get("left"))
            right = _clamp_duty(payload.get("right"))
        except (ValueError, json.JSONDecodeError, UnicodeDecodeError):
            left = right = None
        if left is None or right is None:
            car_link.stop()
            self._json(400, {"error": "duty must be a number from -100 to 100",
                             "left": 0, "right": 0})
            return
        if left == 0 and right == 0:
            car_link.stop()
        else:
            car_link.set_duty(left, right)
        self._json(200, {"left": left, "right": right})

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

    def _speed_control(self):
        try:
            enabled = self._body().get("enabled")
            if not isinstance(enabled, bool):
                raise ValueError("enabled must be true or false")
        except (ValueError, json.JSONDecodeError, UnicodeDecodeError) as exc:
            self._json(400, {"error": str(exc)})
            return
        car_link.set_speed_control(enabled)
        self._json(200, {"speed_control": car_link.speed_control_enabled()})

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
        config.VISION_GAIN = max(0.0, min(1.5, gain))
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

    def _all_lights(self):
        try:
            payload = self._body()
            if not isinstance(payload, dict) or not isinstance(payload.get("enabled"), bool):
                raise ValueError("enabled must be true or false")
        except (ValueError, UnicodeDecodeError) as exc:
            self._json(400, {"error": str(exc)})
            return
        enabled = payload["enabled"]
        hardware.set_all_lights(enabled)
        self._json(200, {"all_lights": enabled})

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

    def _hue_calibration(self):
        try:
            payload = self._body()
            settings = vision.set_hue_calibration(payload.get("enabled"), payload.get("offset"))
        except (ValueError, UnicodeDecodeError) as exc:
            self._json(400, {"error": str(exc)})
            return
        except OSError as exc:
            print(f"Could not save hue calibration: {exc}")
            self._json(500, {"error": "The settings file could not be written."})
            return
        self._json(200, settings)

    def _status(self, since):
        out = modes.status()
        out.update({
            "camera": vision.camera_ok(),
            "hue_calibration": vision.hue_calibration(),
            "left": _command["left"],
            "right": _command["right"],
            "blocked": _command["blocked"] or _front_blocked(),
            "estop": bool(hardware.telemetry.get("estop")),
            "front": _front_m(),
            "tof_m": hardware.sensors["tof_m"],
            "lux": hardware.sensors["lux"],
            "lights_on": bool(hardware.sensors["lights_on"]),
            "all_lights": hardware.all_lights_enabled(),
            "speed_control": car_link.speed_control_enabled(),
            "straight_hold": car_link.straight_hold_enabled(),
            "vision": bool(config.VISION_CONTROL),
            "vision_gain": float(config.VISION_GAIN),
            "speed": float(config.CRUISE_SPEED),
            "lamps": hardware.lamp_state(),
        })
        lane_left, lane_right = modes.lane_availability()
        out["lane_left"] = lane_left
        out["lane_right"] = lane_right
        out["lane"] = _current_lane()
        if since is not None:
            lines, next_id = modes.log.since(since)
            out["log"] = [line for _, line in lines]
            out["log_next"] = next_id
        return out

    def _stream(self, latest_jpeg):
        self.send_response(200)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        last = None
        while True:
            jpeg = latest_jpeg()
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


def _current_lane():
    """Which lane the camera sees right now, for the page. None if no lane."""
    _frame_id, frame_time, lane, _jpeg = vision.latest()
    if lane is None or time.monotonic() - frame_time > 1.0:
        return None
    colors = {0: ("red", "yellow"), 1: ("yellow", "yellow"), 2: ("yellow", "red")}
    left_col, right_col = colors.get(lane.lane_index, ("?", "?"))
    return {
        "index": lane.lane_index,
        "pos": None if lane.lane_pos is None else round(float(lane.lane_pos), 2),
        "confirmed": bool(lane.lane_confirmed),
        "left": left_col if lane.left_lane_exists else None,
        "right": right_col if lane.right_lane_exists else None,
    }


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

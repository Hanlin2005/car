// Uno Q microcontroller: DRV8833 motors, encoders, lidar, and the safety stop.
// The Linux side (python/main.py) sends target wheel speeds over the Bridge.

#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <BH1750.h>
#include "pins.h"

static_assert(LIDAR_RX == 0, "lidar TX is wired to Serial1 RX");

// ---------- Motor test ----------
// true = ignore Python and just spin the motors back and forth (see motorTest()).
// Set to false for normal driving.
const bool MOTOR_TEST = false;

// ---------- Settings ----------
const float STOP_DISTANCE_M = 0.15;        // VL53L1X and lidar emergency stop distance
const float LIGHT_ON_LUX = 20;             // lamps on when darker than this
const float LIGHT_OFF_LUX = 60;            // lamps off when brighter than this
const unsigned long LIGHT_PERIOD_MS = 200; // BH1750 is read about 5 times a second
const unsigned long CMD_TIMEOUT_MS = 300;  // stop if Python goes quiet
const unsigned long WEB_TIMEOUT_MS = 750;  // manual driving, the page sends slower
const unsigned long CONTROL_PERIOD_MS = 20;    // 50 Hz speed control
const unsigned long TELEMETRY_PERIOD_MS = 50;  // 20 Hz telemetry
const float METERS_PER_TICK = 0.0005;      // wheel circumference / ticks per rev
const float KP_STRAIGHT_HOLD = 4.0;        // m/s of trim per meter the wheels differ

// ---------- State ----------
float targetLeft = 0, targetRight = 0;     // m/s from Python
bool openLoop = false;                     // true = raw duty cycle, no speed control
int dutyLeft = 0, dutyRight = 0;           // -255..255, used when openLoop
float speedLeft = 0, speedRight = 0;       // m/s measured
float distLeft = 0, distRight = 0;         // m travelled since start
volatile long ticksLeft = 0, ticksRight = 0;
unsigned long lastCmdMs = 0;
bool estop = false;
float frontM = 10;
float tofM = -1;                           // VL53L1X meters, -1 if there is no reading
float lux = -1;                            // BH1750, -1 if there is no reading
bool lightsOn = false;
bool forceLightsOn = false;
bool automaticLightsOn = false;
bool tofOk = false;
bool lightOk = false;
VL53L1X tof;
BH1750 lightMeter;
String runMode = "idle";
bool straightHoldEnabled = true;   // web page can turn encoder straightening off
bool straightHoldOn = false;
float straightRef = 0;

// Speed controller state per wheel. Declared up here, before any function:
// the Arduino builder adds function prototypes above the first function,
// so types used in function parameters must be defined before it.
struct SpeedPI { float integral = 0; };
SpeedPI piLeft, piRight;

// ========== Encoders ==========
// B's level at A's rising edge tells which way the wheel turns.
// If a wheel reads negative while driving forward, flip its +/- in pins.h.
// Encoder timing, same as the scope readings (microseconds):
//   pd  = time between A rising edges (period)
//   2-1 = time from an A rising edge to the next B rising edge
struct EncTiming {
  volatile unsigned long lastA = 0;
  volatile unsigned long pdSum = 0, pdCount = 0;
  volatile unsigned long offSum = 0, offCount = 0;
};
EncTiming timingLeft, timingRight;
const unsigned long MAX_PERIOD_US = 100000;  // ignore gaps longer than this (wheel was stopped)

void recordA(EncTiming& t) {
  unsigned long now = micros();
  if (t.lastA && now - t.lastA < MAX_PERIOD_US) { t.pdSum += now - t.lastA; t.pdCount++; }
  t.lastA = now;
}

void recordB(EncTiming& t) {
  unsigned long now = micros();
  if (t.lastA && now - t.lastA < MAX_PERIOD_US) { t.offSum += now - t.lastA; t.offCount++; }
}

// Average pd and 2-1 (in ms) since the last call; 0 if no edges were seen
void readTiming(EncTiming& t, float& pdMs, float& offMs) {
  noInterrupts();
  pdMs = t.pdCount ? t.pdSum / 1000.0 / t.pdCount : 0;
  offMs = t.offCount ? t.offSum / 1000.0 / t.offCount : 0;
  t.pdSum = t.pdCount = t.offSum = t.offCount = 0;
  interrupts();
}

void onLeftTick() {
  int dir = digitalRead(LEFT_ENC_B) ? 1 : -1;
  if (INVERT_LEFT_ENCODER) dir = -dir;
  ticksLeft += dir;
  recordA(timingLeft);
}
void onRightTick() {
  int dir = digitalRead(RIGHT_ENC_B) ? 1 : -1;
  if (INVERT_RIGHT_ENCODER) dir = -dir;
  ticksRight += dir;
  recordA(timingRight);
}
void onLeftB()  { recordB(timingLeft); }
void onRightB() { recordB(timingRight); }

void setupEncoders() {
  pinMode(LEFT_ENC_A, INPUT_PULLUP);  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP); pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), onLeftTick, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), onRightTick, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B), onLeftB, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), onRightB, RISING);
}

void updateMeasuredSpeeds(float dt) {
  noInterrupts();
  long l = ticksLeft, r = ticksRight;
  ticksLeft = ticksRight = 0;
  interrupts();
  distLeft += l * METERS_PER_TICK;
  distRight += r * METERS_PER_TICK;
  speedLeft = l * METERS_PER_TICK / dt;
  speedRight = r * METERS_PER_TICK / dt;
}

// ========== Motors ==========
void setupMotors() {
  pinMode(LEFT_IN1, OUTPUT);  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT);
}

// DRV8833, one IN1/IN2 pair per wheel:
//   IN1  IN2   result
//   1    PWM'  forward, slow decay (brakes between pulses)
//   PWM' 1     backward, slow decay
//   PWM  0     forward, fast decay (coasts between pulses)
//   0    PWM   backward, fast decay
//   0    0     coast
//   1    1     brake
// PWM' is the inverted duty, 255 - power. Slow decay keeps speed close to
// linear in duty, which the feed-forward below relies on.
// The DRV8833 pulls a disconnected input low, so with one wire missing,
// "brake" drives that wheel at full speed. Idle and timeouts coast; only the
// emergency stop brakes.
const bool SLOW_DECAY = true;

void driveMotor(int in1, int in2, int power) {  // power: -255..255
  power = constrain(power, -255, 255);
  int duty = abs(power);
  if (power == 0) {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  } else if (SLOW_DECAY) {
    if (power > 0) {
      analogWrite(in1, 255);
      analogWrite(in2, 255 - duty);
    } else {
      analogWrite(in1, 255 - duty);
      analogWrite(in2, 255);
    }
  } else {
    if (power > 0) {
      analogWrite(in1, duty);
      analogWrite(in2, 0);
    } else {
      analogWrite(in1, 0);
      analogWrite(in2, duty);
    }
  }
}

void stopMotors() {  // coast
  analogWrite(LEFT_IN1, 0);  analogWrite(LEFT_IN2, 0);
  analogWrite(RIGHT_IN1, 0); analogWrite(RIGHT_IN2, 0);
}

void brakeMotors() {
  analogWrite(LEFT_IN1, 255);  analogWrite(LEFT_IN2, 255);
  analogWrite(RIGHT_IN1, 255); analogWrite(RIGHT_IN2, 255);
}

// ========== Speed control (feed-forward + PI, per wheel) ==========
// Feed-forward from the duty-cycle benchmark (speed is close to linear in duty):
//   left 100% -> 0.597 m/s, right 100% -> 0.571 m/s (right is ~4% slower)
// Measured on the MDD3A. Re-run python/dutycycletest.py on the DRV8833 and
// whenever the battery or METERS_PER_TICK changes.
const float KF_LEFT = 422.0;    // PWM per m/s
const float KF_RIGHT = 441.0;
const float PWM_OFFSET = 3.0;   // PWM needed just to start moving
const float KP_SPEED = 200.0;
const float KI_SPEED = 400.0;
const float I_LIMIT = 40.0;     // max PWM the integral can add

int speedToPower(SpeedPI& pi, float kf, float target, float measured, float dt) {
  if (target == 0) { pi.integral = 0; return 0; }
  float error = target - measured;
  pi.integral = constrain(pi.integral + KI_SPEED * error * dt, -I_LIMIT, I_LIMIT);
  float ff = kf * target + (target > 0 ? PWM_OFFSET : -PWM_OFFSET);
  return (int)(ff + KP_SPEED * error + pi.integral);
}

// ========== VL53L1X, BH1750, and lamps ==========
// Both sensors share the header SDA/SCL bus (Wire). A failed distance read
// is -1, which does not trip the stop. A failed light read leaves the lamps off.
const uint8_t BH1750_ADDRESS = 0x23;
const unsigned long TOF_STALE_MS = 200;    // no new VL53L1X sample this long = no reading
unsigned long lastTofMs = 0;
void writeLights(bool on) {
  lightsOn = on;
  digitalWrite(REAR_LIGHT_1, on ? HIGH : LOW);
  digitalWrite(REAR_LIGHT_2, on ? HIGH : LOW);
  digitalWrite(HEADLIGHTS, on ? HIGH : LOW);
}

void setupSensors() {
  pinMode(REAR_LIGHT_1, OUTPUT);
  pinMode(REAR_LIGHT_2, OUTPUT);
  pinMode(HEADLIGHTS, OUTPUT);
  writeLights(false);

  Wire.begin();
  tof.setBus(&Wire);
  tof.setTimeout(500);
  tofOk = tof.init();
  if (!tofOk) {
    Serial.println("VL53L1X not found on SDA/SCL");
  } else {
    // Short mode reaches about 1.3 m, plenty for the stop distance, and allows
    // the 20 ms minimum budget. The period must be at least the budget.
    tof.setDistanceMode(VL53L1X::Short);
    tof.setMeasurementTimingBudget(20000);
    tof.startContinuous(25);
  }
  lightOk = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, BH1750_ADDRESS, &Wire);
  if (!lightOk) Serial.println("BH1750 not found on SDA/SCL");
}

// Non-blocking: called every control tick (20 ms), keeps the last sample
// until the next one is ready.
float readTofFront() {
  if (!tofOk) {
    tofM = -1;
    return tofM;
  }
  if (tof.dataReady()) {
    uint16_t mm = tof.read(false);
    lastTofMs = millis();
    // Out of range, low signal, and wrap-around come back as a non-valid
    // status with a meaningless distance (often 0). None is an obstacle.
    if (tof.ranging_data.range_status == VL53L1X::RangeValid) tofM = mm / 1000.0f;
    else tofM = -1;
  } else if (millis() - lastTofMs > TOF_STALE_MS) {
    tofM = -1;
  }
  return tofM;
}

void updateLights() {
  float reading = lightOk ? lightMeter.readLightLevel() : -1;
  lux = reading >= 0 ? reading : -1;
  // Light test drives the pins itself. Keep reading lux for the log.
  if (runMode == "light_test") return;
  if (lux < 0 || lux > LIGHT_OFF_LUX) automaticLightsOn = false;
  else if (lux < LIGHT_ON_LUX) automaticLightsOn = true;
  bool on = forceLightsOn || automaticLightsOn;
  if (lightsOn != on) writeLights(on);
}

// ========== Bridge handlers (Python -> MCU) ==========
void set_wheel_speeds(float left, float right) {
  targetLeft = left;
  targetRight = right;
  openLoop = false;
  lastCmdMs = millis();
}

// Raw duty cycle in percent (-100..100), bypasses speed control. For testing.
void set_duty(float leftPct, float rightPct) {
  dutyLeft = (int)(leftPct * 255.0 / 100.0);
  dutyRight = (int)(rightPct * 255.0 / 100.0);
  openLoop = true;
  lastCmdMs = millis();
}

void stop_car() {
  targetLeft = targetRight = 0;
  openLoop = false;
  dutyLeft = dutyRight = 0;
  lastCmdMs = 0;
  straightHoldOn = false;
}

void set_mode(String mode) {
  if (runMode == "light_test" && mode != "light_test") {
    automaticLightsOn = false;
    writeLights(forceLightsOn);
  }
  runMode = mode;
}

void set_all_lights(bool enabled) {
  if (forceLightsOn == enabled) return;
  forceLightsOn = enabled;
  if (enabled || runMode != "light_test") writeLights(enabled || automaticLightsOn);
}

// Light test only. Each argument turns that lamp on or off.
void set_lamps(bool rear1, bool rear2, bool head) {
  if (runMode != "light_test" || forceLightsOn) return;
  digitalWrite(REAR_LIGHT_1, rear1 ? HIGH : LOW);
  digitalWrite(REAR_LIGHT_2, rear2 ? HIGH : LOW);
  digitalWrite(HEADLIGHTS, head ? HIGH : LOW);
  lightsOn = rear1 || rear2 || head;
}

// Web page toggle. Off leaves the commanded wheel speeds alone.
void set_straight_hold(bool enabled) {
  straightHoldEnabled = enabled;
  if (!enabled) straightHoldOn = false;
}

// While driving straight from the web page, slow the wheel that has gone further.
void holdStraight(float &left, float &right) {
  if (!straightHoldEnabled || runMode != "web" || left != right || left == 0) {
    straightHoldOn = false;
    return;
  }
  float drift = distLeft - distRight;
  if (!straightHoldOn) {
    straightRef = drift;
    straightHoldOn = true;
  }
  float limit = 0.5f * fabs(left);
  float trim = constrain(KP_STRAIGHT_HOLD * (drift - straightRef), -limit, limit);
  left -= trim;
  right += trim;
}

// ========== Lidar (STL19P / LD19) on Serial1: lidar TX -> D0 (RX) ==========
// 47-byte packets at 230400 baud, 12 distance points each. The sketch keeps the
// latest distance for every whole degree and sends the nearest object in each
// direction to Python.
const int LIDAR_ANGLE_OFFSET_DEG = 0;   // rotate if the lidar's 0 deg isn't the car's front
const int LIDAR_HALF_WIDTH_DEG = 15;    // each direction covers +/- this many degrees
const unsigned long LIDAR_STALE_MS = 500;

uint16_t lidarMm[360];
uint32_t lidarMs[360];
uint8_t lidarBuf[47];
int lidarLen = 0;
long lidarGood = 0, lidarBad = 0, lidarBytes = 0;

uint8_t lidarCrc(const uint8_t* d, int n) {
  uint8_t crc = 0;
  for (int i = 0; i < n; i++) {
    crc ^= d[i];
    for (int k = 0; k < 8; k++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x4D) : (uint8_t)(crc << 1);
  }
  return crc;
}

void lidarDecode() {
  float startDeg = (lidarBuf[4] | (lidarBuf[5] << 8)) / 100.0;
  float endDeg = (lidarBuf[42] | (lidarBuf[43] << 8)) / 100.0;
  if (endDeg < startDeg) endDeg += 360.0;
  float step = (endDeg - startDeg) / 11.0;
  uint32_t now = millis();
  for (int i = 0; i < 12; i++) {
    int angle = ((int)(startDeg + step * i + 0.5)) % 360;
    lidarMm[angle] = lidarBuf[6 + i * 3] | (lidarBuf[7 + i * 3] << 8);
    lidarMs[angle] = now;
  }
}

// Call often: the serial buffer is small and the lidar sends ~23 KB/s
void lidarPoll() {
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    lidarBytes++;
    if (lidarLen == 0) {
      if (b == 0x54) lidarBuf[lidarLen++] = b;
    } else if (lidarLen == 1) {
      if (b == 0x2C) lidarBuf[lidarLen++] = b;
      else lidarLen = 0;
    } else {
      lidarBuf[lidarLen++] = b;
      if (lidarLen == 47) {
        if (lidarCrc(lidarBuf, 46) == lidarBuf[46]) { lidarDecode(); lidarGood++; }
        else lidarBad++;
        lidarLen = 0;
      }
    }
  }
}

// Nearest valid distance in meters within +/- half-width of centerDeg, or -1
float lidarSector(int centerDeg) {
  uint32_t now = millis();
  uint16_t best = 0;
  for (int off = -LIDAR_HALF_WIDTH_DEG; off <= LIDAR_HALF_WIDTH_DEG; off++) {
    int a = ((centerDeg + off) % 360 + 360) % 360;
    if (lidarMm[a] > 0 && now - lidarMs[a] < LIDAR_STALE_MS) {
      if (best == 0 || lidarMm[a] < best) best = lidarMm[a];
    }
  }
  return best ? best / 1000.0 : -1.0;
}

// ========== Main ==========
void setup() {
  Serial.begin(115200);
  Serial1.begin(230400);   // lidar TX on LIDAR_RX (D0). D1 stays free.
  Bridge.begin();
  Bridge.provide("set_wheel_speeds", set_wheel_speeds);
  Bridge.provide("set_duty", set_duty);
  Bridge.provide("stop", stop_car);
  Bridge.provide("set_mode", set_mode);
  Bridge.provide("set_straight_hold", set_straight_hold);
  Bridge.provide("set_lamps", set_lamps);
  Bridge.provide("set_all_lights", set_all_lights);

  setupMotors();
  setupEncoders();
  setupSensors();
}

// ========== Motor test ==========
// Runs each motor forward and backward, first at full power with plain
// digitalWrite, then at 60% through driveMotor (PWM, same decay mode as
// driving). Prints encoder ticks.
// If nothing moves at full power, check the DRV8833's VM, GND, and nSLEEP.
// If full power works but PWM doesn't, the pins don't support PWM.

// Both inputs low coasts on the DRV8833. Stop with the same method used to
// start, so PWM doesn't keep running.
void coastMotors(bool usePwm) {
  int pins[] = {LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2};
  for (int pin : pins) {
    if (usePwm) analogWrite(pin, 0);
    else digitalWrite(pin, LOW);
  }
}

void testStep(const char* label, int in1, int in2, int power, bool usePwm) {
  noInterrupts();
  ticksLeft = ticksRight = 0;
  interrupts();

  Serial.print("TEST: ");
  Serial.println(label);
  if (usePwm) {
    driveMotor(in1, in2, power);
  } else {
    pinMode(in1, OUTPUT);  // take the pins back from PWM after a repeat
    pinMode(in2, OUTPUT);
    digitalWrite(in1, power > 0 ? HIGH : LOW);
    digitalWrite(in2, power < 0 ? HIGH : LOW);
  }
  delay(2000);
  coastMotors(usePwm);

  Serial.print("  encoder ticks  left=");
  Serial.print(ticksLeft);
  Serial.print("  right=");
  Serial.println(ticksRight);
  delay(1000);
}

void motorTest() {
  const int PWM_60 = 153;  // 60% of 255

  testStep("LEFT  forward  (full, digital)", LEFT_IN1, LEFT_IN2, 255, false);
  testStep("LEFT  backward (full, digital)", LEFT_IN1, LEFT_IN2, -255, false);
  testStep("RIGHT forward  (full, digital)", RIGHT_IN1, RIGHT_IN2, 255, false);
  testStep("RIGHT backward (full, digital)", RIGHT_IN1, RIGHT_IN2, -255, false);

  testStep("LEFT  forward  (60% PWM)", LEFT_IN1, LEFT_IN2, PWM_60, true);
  testStep("LEFT  backward (60% PWM)", LEFT_IN1, LEFT_IN2, -PWM_60, true);
  testStep("RIGHT forward  (60% PWM)", RIGHT_IN1, RIGHT_IN2, PWM_60, true);
  testStep("RIGHT backward (60% PWM)", RIGHT_IN1, RIGHT_IN2, -PWM_60, true);

  Serial.println("TEST: done, repeating in 3 s");
  delay(3000);
}

void loop() {
  lidarPoll();

  if (MOTOR_TEST) {
    motorTest();
    return;
  }

  static unsigned long lastControl = 0, lastTelemetry = 0, lastLight = 0;
  unsigned long now = millis();

  if (now - lastLight >= LIGHT_PERIOD_MS) {
    lastLight = now;
    updateLights();
  }

  if (now - lastControl >= CONTROL_PERIOD_MS) {
    float dt = (now - lastControl) / 1000.0;
    lastControl = now;

    float front = lidarSector(LIDAR_ANGLE_OFFSET_DEG);
    frontM = front < 0 ? 10.0 : front;
    updateMeasuredSpeeds(dt);

    // Safety: stop on its own, no matter what Python says.
    // An obstacle brakes. Python going quiet coasts (see driveMotor).
    readTofFront();
    bool tofClose = tofM >= 0 && tofM < STOP_DISTANCE_M;
    bool wantEstop = runMode == "drive" || runMode == "straight_test";
    unsigned long timeout = runMode == "web" ? WEB_TIMEOUT_MS : CMD_TIMEOUT_MS;
    bool pythonSilent = lastCmdMs == 0 || (now - lastCmdMs) > timeout;
    estop = wantEstop && (tofClose || (front >= 0 && front < STOP_DISTANCE_M));
    if (estop || pythonSilent) {
      if (estop) brakeMotors();
      else stopMotors();
      piLeft.integral = piRight.integral = 0;
      straightHoldOn = false;
    } else if (openLoop) {
      driveMotor(LEFT_IN1, LEFT_IN2, dutyLeft);
      driveMotor(RIGHT_IN1, RIGHT_IN2, dutyRight);
      piLeft.integral = piRight.integral = 0;
    } else {
      float left = targetLeft, right = targetRight;
      // Manual driving can still reverse. Duty-cycle tests are openLoop, so they stay exempt.
      if (runMode == "web" && tofClose) {
        if (left > 0) left = 0;
        if (right > 0) right = 0;
      }
      holdStraight(left, right);
      driveMotor(LEFT_IN1, LEFT_IN2, speedToPower(piLeft, KF_LEFT, left, speedLeft, dt));
      driveMotor(RIGHT_IN1, RIGHT_IN2, speedToPower(piRight, KF_RIGHT, right, speedRight, dt));
    }
  }

  if (now - lastTelemetry >= TELEMETRY_PERIOD_MS) {
    lastTelemetry = now;
    Bridge.notify("telemetry", speedLeft, speedRight, distLeft, distRight, frontM, estop);
    Bridge.notify("sensors", tofM, lux, lightsOn);

    float pdL, offL, pdR, offR;
    readTiming(timingLeft, pdL, offL);
    readTiming(timingRight, pdR, offR);
    Bridge.notify("encoder_timing", pdL, offL, pdR, offR);

    // LD19 angles increase clockwise seen from above: 90 = right, 270 = left
    // -1 = nothing seen in that direction
    Bridge.notify("lidar",
                  lidarSector(LIDAR_ANGLE_OFFSET_DEG + 0),
                  lidarSector(LIDAR_ANGLE_OFFSET_DEG + 180),
                  lidarSector(LIDAR_ANGLE_OFFSET_DEG + 270),
                  lidarSector(LIDAR_ANGLE_OFFSET_DEG + 90),
                  lidarGood, lidarBad, lidarBytes);
  }
}

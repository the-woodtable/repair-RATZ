/*
  pipe_robot_firmware.ino — EDI PIPE CAM main controller, hardware rev 3
  ======================================================================
  ESP32-S3 N16R8. Hardware this revision supports:
    * 1x NEMA 17 stepper via DRV8825      (drive: forward/backward)
    * 1x 12V linear actuator via TB6612   (lining deployment)
    * 1x NB-M005 micro servo              (string unhook for lining alignment)
    * 1x MPU-6050-family IMU on I2C       (roll/pitch + slip detection)
    * 2x ESP32-CAM on UART1/UART2         (stereo pair, bridged to USB)

  ---------------------------------------------------------------------
  CAMERA CHANGES (Yue Xuan, integrating the stereo pair) — 3 edits only,
  all inside CamBridge. Motor / actuator / servo / IMU / telemetry code is
  untouched. Search "CAMERA EDIT" to review them.

    1. Header format: cameras send  AA 55 | len(4) | JPEG  (no id byte).
       The camera firmware is IDENTICAL on both boards — a camera does not
       know if it is left or right. Identity comes from WHICH UART it is
       plugged into, assigned here. Previously this parser expected an id
       byte from the camera, which no camera sends -> zero frames.
    2. Camera TX lines kept (GPIO 15 / 17) and now carry LIVE SETTINGS:
       the PC can retune JPEG quality, brightness, contrast, resolution and
       the flash LED on either camera without reflashing anything. Sent as
       a 3-byte escape "@" + target + cmd — see CAM COMMANDS in loop().
    3. Fairness: poll() now forwards at most ONE frame per call. Draining
       one camera completely lets it hog the loop while the other's buffer
       overflows -> the two feeds visibly take turns freezing.

  The USB output protocol (id 0 / 1 / 2) is UNCHANGED.
  ---------------------------------------------------------------------

  ODOMETRY DESIGN (read this once, it matters):
    Distance is counted from STEPPER STEPS in the timer ISR, not from the
    IMU. A stepper is a position device — each step is a fixed increment of
    travel — so step counting is exact and drift-free. Integrating IMU
    acceleration twice drifts quadratically and is useless past ~1 second.
    The IMU's real jobs here: (a) roll angle = crack clock position, since
    the helical body rotates; (b) slip detection — if steps advance but the
    IMU registers no motion, the wheels are slipping and step-odometry is
    overcounting. The PC gets both streams and can cross-check.

  CALIBRATION (one-time, in the pipe):
    Drive forward a fixed count (Z to zero, hold F briefly, read "steps" in
    telemetry), measure actual travel with a tape, then set:
        STEPS_PER_MM = steps / measured_mm
    The helix angle of your wheels sets this — it can't be computed reliably
    from geometry alone because of wheel slip and preload, so measure it.

  USB PROTOCOL OUT (921600 baud), all frames:
      0xAA 0x55 | id(1) | len(4, little-endian) | payload
      id 0 = LEFT JPEG   id 1 = RIGHT JPEG   id 2 = telemetry JSON (ASCII)
  COMMANDS IN (single ASCII chars):
      F forward   B backward   S stop drive        (dead-man: repeat < 400 ms)
      E extend    R retract    X stop actuator     (dead-man: repeat < 400 ms)
      H servo -> hook position   U servo -> release position   (latching)
      Z zero the odometer
      @<target><cmd>  relay a live setting to a camera (see CAM COMMANDS)
*/

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

// ============================ PINS ============================
#define PIN_STEP   4
#define PIN_DIR    5
#define PIN_EN     6      // DRV8825 ENABLE, LOW = driver on
#define PIN_AIN1   7      // TB6612
#define PIN_AIN2   8
#define PIN_PWMA   9
#define PIN_STBY   10
#define PIN_SERVO  11
#define PIN_SDA    13
#define PIN_SCL    14
#define CAMR_TX    15     // UART2 -> right cam U0R  (live camera settings)
#define CAMR_RX    16     // UART2 <- right cam U0T  (video)
#define CAML_TX    17     // UART1 -> left  cam U0R  (live camera settings)
#define CAML_RX    18     // UART1 <- left  cam U0T  (video)
// 4 wires per camera: 5V, GND, U0T -> CAMx_RX, U0R <- CAMx_TX.
// The TX line carries live setting changes (quality/brightness/resolution)
// so the cameras never need reflashing to be retuned. See CAM COMMANDS below.
// Avoided: 0/45/46 (strapping), 19/20 (USB D-/D+), 26-32 (flash/PSRAM)

// ============================ TUNING ============================
#define STEP_HZ          800      // full-step pulses per second
#define STEPS_PER_MM     16.0f    // <-- CALIBRATE (see header)
#define WATCHDOG_MS      400      // stop drive+actuator if no repeat cmd
#define TELEM_MS         200      // telemetry period
#define SERVO_HOOK_DEG   180       // adjust after servo_test.ino session
#define SERVO_RELEASE_DEG 0
#define ACT_PWM          255      // actuator speed 0-255
#define DIR_FWD_LEVEL    HIGH     // flip if "F" drives the wrong way
#define CAM_BAUD         460800   // MUST match Serial.begin() in
                                  // camera_firmware.ino (currently 460800).
                                  // Mismatch = garbage or no frames at all.

// ============================ STEPPER (ISR) ============================
hw_timer_t* stepTimer = nullptr;
volatile int8_t  motorDir  = 0;      // 0 stop, +1 fwd, -1 back
volatile bool    stepLevel = false;
volatile int32_t stepCount = 0;      // signed odometer, +fwd

void IRAM_ATTR onStepTimer() {
  if (motorDir == 0) { if (stepLevel) { digitalWrite(PIN_STEP, LOW); stepLevel = false; } return; }
  stepLevel = !stepLevel;
  digitalWrite(PIN_STEP, stepLevel);
  if (stepLevel) stepCount += motorDir;   // count on rising edge
}

void setDrive(int8_t dir) {
  if (dir != 0) digitalWrite(PIN_DIR, (dir > 0) ? DIR_FWD_LEVEL : !DIR_FWD_LEVEL);
  motorDir = dir;
  // EN stays LOW permanently: holding torque keeps lining tension when stopped.
  // If the driver runs hot at standstill, lower its Vref rather than disabling.
}

// ============================ ACTUATOR ============================
int8_t actState = 0;   // 0 idle, +1 extend, -1 retract
void setActuator(int8_t s) {
  actState = s;
  if (s == 0)      { analogWrite(PIN_PWMA, 0);       digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, LOW); }
  else if (s > 0)  { digitalWrite(PIN_AIN1, HIGH);   digitalWrite(PIN_AIN2, LOW);  analogWrite(PIN_PWMA, ACT_PWM); }
  else             { digitalWrite(PIN_AIN1, LOW);    digitalWrite(PIN_AIN2, HIGH); analogWrite(PIN_PWMA, ACT_PWM); }
}

// ============================ SERVO ============================
Servo hookServo;
bool servoHooked = true;
void setServo(bool hooked) {
  servoHooked = hooked;
  hookServo.write(hooked ? SERVO_HOOK_DEG : SERVO_RELEASE_DEG);
}

// ============================ IMU (MPU-6050 family, raw) ============================
uint8_t imuAddr = 0;
float pitch = 0, roll = 0, aMag = 1.0f;
uint32_t imuLastUs = 0;

void imuWrite(uint8_t r, uint8_t v) { Wire.beginTransmission(imuAddr); Wire.write(r); Wire.write(v); Wire.endTransmission(); }

void imuInit() {
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  for (uint8_t a : {0x68, 0x69}) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { imuAddr = a; break; }
  }
  if (!imuAddr) return;
  imuWrite(0x6B, 0x00);   // wake
  imuWrite(0x1C, 0x00);   // +/-2g
  imuWrite(0x1B, 0x00);   // +/-250 dps
}

void imuUpdate() {
  if (!imuAddr) return;
  Wire.beginTransmission(imuAddr); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom((int)imuAddr, 14);
  if (Wire.available() < 14) return;
  int16_t raw[7];
  for (int i = 0; i < 7; i++) raw[i] = (Wire.read() << 8) | Wire.read();
  float ax = raw[0] / 16384.0f, ay = raw[1] / 16384.0f, az = raw[2] / 16384.0f;
  float gx = raw[4] / 131.0f,   gy = raw[5] / 131.0f;

  uint32_t now = micros();
  float dt = (now - imuLastUs) * 1e-6f; imuLastUs = now;
  if (dt <= 0 || dt > 0.5f) dt = 0.01f;

  aMag = sqrtf(ax * ax + ay * ay + az * az);   // ~1.0 g at rest; deviations = motion
  float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2958f;
  float accRoll  = atan2f( ay, az) * 57.2958f;
  pitch = 0.98f * (pitch + gy * dt) + 0.02f * accPitch;
  roll  = 0.98f * (roll  + gx * dt) + 0.02f * accRoll;
}

// ============================ CAMERA BRIDGE ============================
// Byte-level passthrough fails with two cameras (frames interleave), so each
// UART gets its own state machine that assembles a complete frame, then the
// whole frame is written to USB atomically.
//
// IN  (from camera): AA 55 | len(4 LE) | JPEG          <- no id, see CAMERA EDIT 1
// OUT (to PC):       AA 55 | id | len(4 LE) | JPEG     <- id assigned here
struct CamBridge {
  HardwareSerial* ser;
  uint8_t  id, state = 0, hdrIdx = 0, hdr[4];
  uint32_t len = 0, got = 0, cap = 0;
  uint8_t* buf = nullptr;

  void begin(HardwareSerial* s, int rxPin, int txPin, uint8_t camId) {
    ser = s; id = camId;
    ser->setRxBufferSize(16384);              // must be set BEFORE begin()
    ser->begin(CAM_BAUD, SERIAL_8N1, rxPin, txPin);
    cap = 200000; buf = (uint8_t*)ps_malloc(cap);          // PSRAM on N16R8
    if (!buf) { cap = 60000; buf = (uint8_t*)malloc(cap); } // fallback
  }

  void poll() {
    while (ser->available()) {
      uint8_t b = ser->read();
      switch (state) {
        case 0: state = (b == 0xAA) ? 1 : 0; break;
        case 1: if (b == 0x55) { state = 2; hdrIdx = 0; } else state = (b == 0xAA) ? 1 : 0; break;
        case 2:
          // CAMERA EDIT 1: 4-byte length only. The camera sends no id byte.
          hdr[hdrIdx++] = b;
          if (hdrIdx == 4) {
            len = (uint32_t)hdr[0] | ((uint32_t)hdr[1] << 8) | ((uint32_t)hdr[2] << 16) | ((uint32_t)hdr[3] << 24);
            got = 0;
            state = (len > 0 && len < cap) ? 3 : 0;
          }
          break;
        case 3:
          buf[got++] = b;
          // CAMERA EDIT 3: return after one frame so the other camera gets
          // a turn — otherwise the feeds alternate between freezing.
          if (got == len) { emit(); state = 0; return; }
          break;
      }
    }
  }

  void emit() {
    uint8_t h[7] = { 0xAA, 0x55, id,
                     (uint8_t)len, (uint8_t)(len >> 8), (uint8_t)(len >> 16), (uint8_t)(len >> 24) };
    Serial.write(h, 7);
    Serial.write(buf, len);
  }
};

CamBridge camL, camR;

// ============================ TELEMETRY ============================
void sendTelemetry() {
  int32_t steps;
  noInterrupts(); steps = stepCount; interrupts();
  char js[240];
  int n = snprintf(js, sizeof(js),
    "{\"t\":%lu,\"steps\":%ld,\"mm\":%.1f,\"pitch\":%.1f,\"roll\":%.1f,"
    "\"amag\":%.2f,\"drv\":%d,\"act\":%d,\"hook\":%d,\"imu\":%d}",
    (unsigned long)millis(), (long)steps, steps / STEPS_PER_MM,
    pitch, roll, aMag, motorDir, actState, servoHooked ? 1 : 0, imuAddr ? 1 : 0);
  uint8_t h[7] = { 0xAA, 0x55, 2, (uint8_t)n, (uint8_t)(n >> 8), 0, 0 };
  Serial.write(h, 7);
  Serial.write((uint8_t*)js, n);
}

// ============================ SETUP / LOOP ============================
uint32_t lastDriveCmd = 0, lastActCmd = 0, lastTelem = 0, lastImu = 0;

void setup() {
  Serial.begin(921600);   // USB CDC  (Tools -> USB CDC On Boot: Enabled)

  pinMode(PIN_STEP, OUTPUT); pinMode(PIN_DIR, OUTPUT); pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, LOW);                    // driver enabled
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);
  setActuator(0);

  hookServo.setPeriodHertz(50);
  hookServo.attach(PIN_SERVO, 500, 2500);       // narrow after servo_test session
  setServo(true);                               // boot in HOOK position

  imuInit();

  // Left/right identity is decided HERE, by which pin each camera is on.
  camL.begin(&Serial1, CAML_RX, CAML_TX, 0);   // id 0 = LEFT
  camR.begin(&Serial2, CAMR_RX, CAMR_TX, 1);   // id 1 = RIGHT

  // Hardware timer -> jitter-free step pulses even during big USB writes.
  // (Arduino-ESP32 core 3.x API. On core 2.x use:
  //   stepTimer = timerBegin(0, 80, true);
  //   timerAttachInterrupt(stepTimer, &onStepTimer, true);
  //   timerAlarmWrite(stepTimer, 1000000UL / (STEP_HZ * 2), true);
  //   timerAlarmEnable(stepTimer); )
  stepTimer = timerBegin(1000000);              // 1 MHz base
  timerAttachInterrupt(stepTimer, &onStepTimer);
  timerAlarm(stepTimer, 1000000UL / (STEP_HZ * 2), true, 0);  // toggle at 2x
}

void loop() {
  camL.poll();
  camR.poll();

  // ---- commands ----
  while (Serial.available()) {
    char c = Serial.read();

    // CAM COMMANDS: 3-byte escape "@" + target + cmd, relayed to a camera.
    // Escaped because camera letters (F, B, R...) collide with drive letters.
    //   target: 'l' left   'r' right   'a' both
    //   cmd:    q/Q quality  b/B brightness  c/C contrast
    //           1-4 resolution   f/F flash LED
    // e.g. "@aQ" = both cameras, better JPEG quality.
    if (c == '@') {
      while (Serial.available() < 2) { /* wait for the 2 remaining bytes */ }
      char target = Serial.read();
      char cmd    = Serial.read();
      if (target == 'l' || target == 'a') Serial1.write(cmd);
      if (target == 'r' || target == 'a') Serial2.write(cmd);
      continue;
    }

    switch (c) {
      case 'F': setDrive(+1); lastDriveCmd = millis(); break;
      case 'B': setDrive(-1); lastDriveCmd = millis(); break;
      case 'S': setDrive(0);  break;
      case 'E': setActuator(+1); lastActCmd = millis(); break;
      case 'R': setActuator(-1); lastActCmd = millis(); break;
      case 'X': setActuator(0);  break;
      case 'H': setServo(true);  break;   // latching — no watchdog
      case 'U': setServo(false); break;
      case 'A': hookServo.write(90); break;   // assembly position — direct, no state tracking
      case 'Z': noInterrupts(); stepCount = 0; interrupts(); break;
    }
  }

  // ---- dead-man watchdogs (UI repeats F/B/E/R every 150 ms while held) ----
  if (motorDir != 0 && millis() - lastDriveCmd > WATCHDOG_MS) setDrive(0);
  if (actState != 0 && millis() - lastActCmd  > WATCHDOG_MS) setActuator(0);

  // ---- IMU @ ~100 Hz ----
  if (millis() - lastImu >= 10) { lastImu = millis(); imuUpdate(); }

  // ---- telemetry @ 5 Hz ----
  if (millis() - lastTelem >= TELEM_MS) { lastTelem = millis(); sendTelemetry(); }
}

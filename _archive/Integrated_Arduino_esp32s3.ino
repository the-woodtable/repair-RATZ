/*
 * EDI PIPE CAM — ESP32-S3 Main Controller (Stepper Edition)
 * -----------------------------------------------------------
 * Board settings (Tools menu):
 *   Board: "ESP32S3 Dev Module"
 *   USB CDC On Boot: "Enabled"   <-- REQUIRED
 *   PSRAM: "OPI PSRAM", Flash Size: "16MB"
 *
 * Drive:    2x NEMA 17 via DRV8825 (STEP/DIR/EN)
 * Deploy:   12V linear actuator via TB6612FNG
 * Video:    ESP32-CAM on UART1 @460800, framed JPEG (0xAA55 + LE u32 len),
 *           forwarded byte-for-byte to USB serial for the Python panel.
 *
 * Command protocol (single chars from PC over USB):
 *   'F' drive forward   (M1 CW, M2 CCW)
 *   'B' drive backward  (M1 CCW, M2 CW)
 *   'S' stop drive
 *   'D' actuator deploy (extend)   -- press-and-hold
 *   'R' actuator retract           -- press-and-hold
 *   'X' stop actuator
 *   'K' keepalive (refreshes watchdog without changing state)
 *
 * Dead-man watchdog: the Python panel re-sends the active command every
 * 100 ms. If nothing arrives for CMD_TIMEOUT_MS while anything is moving,
 * everything stops. Yanked cable / crashed app = robot stops.
 */

// ---------------- Pin map ----------------
// DRV8825 #1 (Motor 1)
#define M1_STEP_PIN 4
#define M1_DIR_PIN  5
#define M1_EN_PIN   6   // LOW = enabled
// DRV8825 #2 (Motor 2)
#define M2_STEP_PIN 7
#define M2_DIR_PIN  15
#define M2_EN_PIN   16  // LOW = enabled

// TB6612FNG (linear actuator on channel A)
#define ACT_IN1_PIN  12
#define ACT_IN2_PIN  13
#define ACT_PWM_PIN  14
#define ACT_STBY_PIN 47  // HIGH = driver awake

// ESP32-CAM UART link
#define CAM_RX_PIN 18    // <- CAM GPIO13 (CAM TX)
#define CAM_TX_PIN 17    // -> CAM GPIO14 (CAM RX)
#define CAM_BAUD   460800

// ---------------- Tuning ----------------
// DIR pin level that makes each motor turn its "forward" way.
// Forward convention: M1 clockwise, M2 counterclockwise (shaft end view).
// If a motor spins the wrong way on the bench, flip its constant here.
#define M1_FWD_LEVEL HIGH
#define M2_FWD_LEVEL LOW

// Microseconds between step pulses. Full-step mode, 200 steps/rev:
// 800 us -> 1250 steps/s -> ~6.25 rev/s. Raise this number if motors stall.
const uint32_t STEP_INTERVAL_US = 4000;

// Dead-man timeout
const uint32_t CMD_TIMEOUT_MS = 400;

// Actuator PWM duty (0-255). 255 = full speed.
const uint8_t ACT_SPEED = 255;

// ---------------- State ----------------
int8_t   driveState = 0;   // 1 = forward, -1 = backward, 0 = stopped
int8_t   actState   = 0;   // 1 = deploy, -1 = retract, 0 = stopped
uint32_t lastCmdMs  = 0;
uint32_t lastStepUs = 0;

uint8_t camBuf[512];

// ---------------- Helpers ----------------
void setDrive(int8_t s) {
  driveState = s;
  if (s == 0) {
    // De-energize: EN high = coils released, drivers cool.
    digitalWrite(M1_EN_PIN, HIGH);
    digitalWrite(M2_EN_PIN, HIGH);
    return;
  }
  bool fwd = (s > 0);
  digitalWrite(M1_DIR_PIN, fwd ? M1_FWD_LEVEL : (M1_FWD_LEVEL == HIGH ? LOW : HIGH));
  digitalWrite(M2_DIR_PIN, fwd ? M2_FWD_LEVEL : (M2_FWD_LEVEL == HIGH ? LOW : HIGH));
  digitalWrite(M1_EN_PIN, LOW);
  digitalWrite(M2_EN_PIN, LOW);
  lastStepUs = micros();
}

void setActuator(int8_t s) {
  actState = s;
  if (s > 0) {          // deploy / extend
    digitalWrite(ACT_IN1_PIN, HIGH);
    digitalWrite(ACT_IN2_PIN, LOW);
    analogWrite(ACT_PWM_PIN, ACT_SPEED);
  } else if (s < 0) {   // retract
    digitalWrite(ACT_IN1_PIN, LOW);
    digitalWrite(ACT_IN2_PIN, HIGH);
    analogWrite(ACT_PWM_PIN, ACT_SPEED);
  } else {              // stop (coast)
    digitalWrite(ACT_IN1_PIN, LOW);
    digitalWrite(ACT_IN2_PIN, LOW);
    analogWrite(ACT_PWM_PIN, 0);
  }
}

void stopAll() {
  setDrive(0);
  setActuator(0);
}

// ---------------- Setup ----------------
void setup() {
  // Steppers
  pinMode(M1_STEP_PIN, OUTPUT);
  pinMode(M1_DIR_PIN,  OUTPUT);
  pinMode(M1_EN_PIN,   OUTPUT);
  pinMode(M2_STEP_PIN, OUTPUT);
  pinMode(M2_DIR_PIN,  OUTPUT);
  pinMode(M2_EN_PIN,   OUTPUT);
  digitalWrite(M1_EN_PIN, HIGH);   // disabled at boot
  digitalWrite(M2_EN_PIN, HIGH);
  digitalWrite(M1_STEP_PIN, LOW);
  digitalWrite(M2_STEP_PIN, LOW);

  // Actuator
  pinMode(ACT_IN1_PIN,  OUTPUT);
  pinMode(ACT_IN2_PIN,  OUTPUT);
  pinMode(ACT_PWM_PIN,  OUTPUT);
  pinMode(ACT_STBY_PIN, OUTPUT);
  digitalWrite(ACT_STBY_PIN, HIGH); // wake TB6612
  setActuator(0);

  // USB serial to PC (baud value ignored by native USB CDC, set anyway)
  Serial.begin(921600);

  // UART1 to ESP32-CAM. Big RX buffer so JPEG frames don't overrun
  // while we're busy stepping.
  Serial1.setRxBufferSize(16384);
  Serial1.begin(CAM_BAUD, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);

  lastCmdMs = millis();
}

// ---------------- Main loop ----------------
void loop() {
  // 1) Commands from PC
  while (Serial.available()) {
    char c = (char)Serial.read();
    lastCmdMs = millis();
    switch (c) {
      case 'F': if (driveState != 1)  setDrive(1);   break;
      case 'B': if (driveState != -1) setDrive(-1);  break;
      case 'S': setDrive(0);                         break;
      case 'D': if (actState != 1)  setActuator(1);  break;
      case 'R': if (actState != -1) setActuator(-1); break;
      case 'X': setActuator(0);                      break;
      case 'K': /* keepalive only */                 break;
      default:  break; // ignore anything else
    }
  }

  // 2) Dead-man watchdog
  if ((driveState != 0 || actState != 0) &&
      (millis() - lastCmdMs > CMD_TIMEOUT_MS)) {
    stopAll();
  }

  // 3) Stepper pulse generation (non-blocking; unsigned math handles
  //    micros() rollover correctly)
  if (driveState != 0) {
    uint32_t now = micros();
    if (now - lastStepUs >= STEP_INTERVAL_US) {
      lastStepUs = now;
      digitalWrite(M1_STEP_PIN, HIGH);
      digitalWrite(M2_STEP_PIN, HIGH);
      delayMicroseconds(3);           // DRV8825 needs >1.9 us pulse
      digitalWrite(M1_STEP_PIN, LOW);
      digitalWrite(M2_STEP_PIN, LOW);
    }
  }

  // 4) Camera passthrough: CAM UART -> USB, verbatim.
  int n = Serial1.available();
  if (n > 0) {
    if (n > (int)sizeof(camBuf)) n = sizeof(camBuf);
    n = Serial1.read(camBuf, n);
    if (n > 0) Serial.write(camBuf, n);
  }
}

/*
  esp32s3_hub.ino
  EDI Pipe Cam - central hub firmware for the ESP32-S3

  Responsibilities:
    1. Receive framed JPEG streams from two ESP32-CAM boards over UART1/UART2
    2. Re-frame each with a camera-ID byte and relay over native USB (Serial)
       to the laptop running crack_detection_panel.py
    3. Parse incoming single-char commands from the laptop over the same USB
       link: F/B/S (drive) and D/R/X (actuator)
    4. Drive the TB6612FNG (two N20 motors) and the 12V actuator accordingly
    5. Enforce the ~400ms dead-man watchdog - if no command arrives within
       that window, stop all motion

  Wire protocol laptop <- hub (camera frames):
    [0xAA][0x55][cam_id: uint8][len: uint32 LE][ <len> bytes JPEG ]
    cam_id 0 = Cam1 (UART1), cam_id 1 = Cam2 (UART2)

    NOTE: this adds one byte (cam_id) versus the single-camera protocol.
    crack_detection_panel.py's parser needs to be updated to read that byte
    and demux frames per camera before the two streams will show up
    correctly on the PC side.

  Wire protocol laptop -> hub (commands): unchanged, single ASCII char per
  command: 'F'/'B'/'S' drive, 'D'/'R'/'X' actuator, sent every ~100ms as
  keepalive from crack_detection_panel.py.
*/

#include <HardwareSerial.h>

// ---- UART links to the two cameras ----
HardwareSerial CamSerial1(1);   // UART1 -> Cam1
HardwareSerial CamSerial2(2);   // UART2 -> Cam2

static const int CAM1_RX_PIN = 17;   // <- Cam1 IO13 (TX, frames out)
static const int CAM1_TX_PIN = 18;   // -> Cam1 IO15 (RX, unused/reserved)
static const int CAM2_RX_PIN = 8;    // <- Cam2 IO13 (TX, frames out)
static const int CAM2_TX_PIN = 9;    // -> Cam2 IO15 (RX, unused/reserved)
static const uint32_t CAM_UART_BAUD = 921600;  // must match esp32cam_stream.ino

// ---- TB6612FNG motor driver pins ----
// NOTE: BIN2/PWMB moved off GPIO8/9 - those are now taken by the Cam2 UART link above.
static const int AIN1 = 4, AIN2 = 5, PWMA = 6;
static const int BIN1 = 7, BIN2 = 35, PWMB = 36;
static const int STBY = 10;

// ---- Actuator driver pins (D/R/X) ----
static const int ACT_IN1 = 11, ACT_IN2 = 12, ACT_EN = 13;

static const uint8_t MAGIC0 = 0xAA;
static const uint8_t MAGIC1 = 0x55;
static const uint32_t WATCHDOG_TIMEOUT_MS = 400;

// ---- Per-camera frame reassembly state ----
struct CamFrameState {
  HardwareSerial *port;
  uint8_t camId;
  enum { WAIT_MAGIC0, WAIT_MAGIC1, READ_LEN, READ_PAYLOAD } state = WAIT_MAGIC0;
  uint8_t lenBuf[4];
  uint8_t lenIdx = 0;
  uint32_t payloadLen = 0;
  uint32_t payloadIdx = 0;
  uint8_t buf[32000];   // adjust upward if you raise JPEG quality/frame size
};

CamFrameState cam1State{ &CamSerial1, 0 };
CamFrameState cam2State{ &CamSerial2, 1 };

unsigned long lastCommandMs = 0;

void motorsStop() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); analogWrite(PWMA, 0);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, 0);
}

void motorsForward() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, 200);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, 200);
}

void motorsBackward() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, 200);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, 200);
}

void actuatorStop()    { digitalWrite(ACT_EN, LOW); }
void actuatorDeploy()  { digitalWrite(ACT_IN1, HIGH); digitalWrite(ACT_IN2, LOW);  digitalWrite(ACT_EN, HIGH); }
void actuatorRetract() { digitalWrite(ACT_IN1, LOW);  digitalWrite(ACT_IN2, HIGH); digitalWrite(ACT_EN, HIGH); }

void handleCommandByte(char c) {
  lastCommandMs = millis();
  switch (c) {
    case 'F': motorsForward();  break;
    case 'B': motorsBackward(); break;
    case 'S': motorsStop();     break;
    case 'D': actuatorDeploy();  break;
    case 'R': actuatorRetract(); break;
    case 'X': actuatorStop();    break;
    default: break;   // ignore unknown/keepalive-only bytes
  }
}

void pumpCameraFrames(CamFrameState &s) {
  while (s.port->available()) {
    uint8_t b = s.port->read();
    switch (s.state) {
      case CamFrameState::WAIT_MAGIC0:
        if (b == MAGIC0) s.state = CamFrameState::WAIT_MAGIC1;
        break;
      case CamFrameState::WAIT_MAGIC1:
        s.state = (b == MAGIC1) ? CamFrameState::READ_LEN : CamFrameState::WAIT_MAGIC0;
        s.lenIdx = 0;
        break;
      case CamFrameState::READ_LEN:
        s.lenBuf[s.lenIdx++] = b;
        if (s.lenIdx == 4) {
          s.payloadLen = (uint32_t)s.lenBuf[0] | ((uint32_t)s.lenBuf[1] << 8) |
                         ((uint32_t)s.lenBuf[2] << 16) | ((uint32_t)s.lenBuf[3] << 24);
          s.payloadIdx = 0;
          if (s.payloadLen == 0 || s.payloadLen > sizeof(s.buf)) {
            s.state = CamFrameState::WAIT_MAGIC0;   // bogus length, resync
          } else {
            s.state = CamFrameState::READ_PAYLOAD;
          }
        }
        break;
      case CamFrameState::READ_PAYLOAD:
        s.buf[s.payloadIdx++] = b;
        if (s.payloadIdx == s.payloadLen) {
          // Full frame in hand - relay to PC with cam-ID byte inserted
          uint8_t header[7];
          header[0] = MAGIC0;
          header[1] = MAGIC1;
          header[2] = s.camId;
          header[3] = (uint8_t)(s.payloadLen & 0xFF);
          header[4] = (uint8_t)((s.payloadLen >> 8) & 0xFF);
          header[5] = (uint8_t)((s.payloadLen >> 16) & 0xFF);
          header[6] = (uint8_t)((s.payloadLen >> 24) & 0xFF);
          Serial.write(header, sizeof(header));
          Serial.write(s.buf, s.payloadLen);
          s.state = CamFrameState::WAIT_MAGIC0;
        }
        break;
    }
  }
}

void setup() {
  Serial.begin(2000000);   // native USB link to laptop

  CamSerial1.begin(CAM_UART_BAUD, SERIAL_8N1, CAM1_RX_PIN, CAM1_TX_PIN);
  CamSerial2.begin(CAM_UART_BAUD, SERIAL_8N1, CAM2_RX_PIN, CAM2_TX_PIN);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);   // take driver out of standby

  pinMode(ACT_IN1, OUTPUT); pinMode(ACT_IN2, OUTPUT); pinMode(ACT_EN, OUTPUT);

  motorsStop();
  actuatorStop();
  lastCommandMs = millis();
}

void loop() {
  // 1. Pump both camera links -> USB
  pumpCameraFrames(cam1State);
  pumpCameraFrames(cam2State);

  // 2. Read any pending command bytes from the laptop
  while (Serial.available()) {
    handleCommandByte((char)Serial.read());
  }

  // 3. Dead-man watchdog
  if (millis() - lastCommandMs > WATCHDOG_TIMEOUT_MS) {
    motorsStop();
    actuatorStop();
  }
}

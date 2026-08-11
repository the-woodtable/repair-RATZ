#include <Arduino.h>
#include "HookServo.h"
#include "Stepper.h"
#include "Actuator.h"
#include "StereoCameras.h"
#include "IMU.h"
#include "LED.h"

//PINS
int PIN_SERVO = 11;
int PIN_SPOOL = 15;
int PIN_STEP =  4;
int PIN_DIR = 5;
int PIN_EN =  6;

int PIN_ACT_AIN1 = 7;
int PIN_ACT_AIN2 = 8;
int PIN_ACT_PWM  = 9;    // TB6612 PWMA — set to your actual pin
int PIN_ACT_STBY = 10;   // TB6612 STBY — set to your actual pin

int CAML_RX = 18;   // left cam U0T  -> S3
int CAMR_RX = 16;   // right cam U0T -> S3
 
int PIN_SDA = 13;
int PIN_SCL = 14;

int PIN_LED = 12;

//OBJECTS FROM CLASSES
HookServo hookServo;
HookServo spoolServo;
Stepper stepper;
Actuator actuator;
StereoCameras cameras; 
IMU imu;
LED led;

//STEPPER VARIABLES
int STEP_HZ = 100;

 
//ACTUATOR VARIABLES
int ACT_SPEED = 255;   // 0-255, default speed for jog commands
 

//OTHER VARIABLES
bool setupSuccessful = true;

const uint32_t TELEM_INTERVAL_MS = 200;   // 5 Hz
uint32_t lastTelemetry = 0;



// ===========================================================================
// EXPERIMENTAL: automated spool-retract sequence            press 'P' to run
// ---------------------------------------------------------------------------
// Drive forward a little -> turn the spool one round -> repeat, until the
// string is wound in.
//
// TO TURN OFF: change the 1 below to 0. Nothing else needs touching.
// TO DELETE:  remove this whole block plus the three short #if blocks marked
//             "spool sequence" in setup(), loop() and the command switch.
//
// Existing 'T' and 'Y' spool keys are untouched and still work by hand.
//
// WHY A STATE MACHINE AND NOT delay():
//   loop() must keep calling cameras.update(). A delay() here would stall
//   camera forwarding for the whole sequence, overflow the UART buffers and
//   bring back exactly the torn/truncated frames we spent days removing.
//   So each step just records a timestamp and returns immediately.
#define ENABLE_SPOOL_SEQUENCE 1

#if ENABLE_SPOOL_SEQUENCE

// THE SPOOL SERVO IS CONTINUOUS ROTATION. setAngle() therefore sets SPEED
// AND DIRECTION, not a position:
//     90  = stop
//     180 = full speed one way
//     0   = full speed the other way
//     values in between = proportionally slower
// So there is no "turn to X degrees" — you start it spinning, wait, and stop
// it. How far it turns is speed x time, which is why SPOOL_SPIN_MS below has
// to be measured rather than guessed.

// ---- tune these ----
static const uint8_t  SPOOL_CYCLES   = 6;    // how many drive+spin repeats
static const uint32_t SPOOL_DRIVE_MS = 400;  // forward time per step
static const int      SPOOL_SPIN     = 180;  // wind-in speed; 0 = other way
static const int      SPOOL_STOP     = 90;   // servo neutral = stopped
// TIME FOR ONE REVOLUTION — MEASURE THIS, don't guess. Mark the spool, press
// 'T', time one full turn, press 'Y'. A typical continuous servo runs about
// 50-60 rpm unloaded, so one turn is roughly 1000-1200 ms, and it gets slower
// as the string load builds.
static const uint32_t SPOOL_SPIN_MS  = 1000;
static const int      SPOOL_DIR      = +1;   // +1 forward, -1 reverse
// --------------------

enum SpoolState : uint8_t { SPOOL_IDLE, SPOOL_DRIVING, SPOOL_SPINNING };
static SpoolState spoolState = SPOOL_IDLE;
static uint32_t   spoolT0    = 0;
static uint8_t    spoolCycle = 0;

static void spoolSeqAbort() {
  if (spoolState == SPOOL_IDLE) return;
  stepper.setDrive(0);                 // stop the motor FIRST
  spoolServo.setAngle(SPOOL_STOP);     // 90 = stop, not "go to 90 degrees"
  spoolState = SPOOL_IDLE;
  Serial.println("{\"spool_seq\":0}");
}

static void spoolSeqStart() {
  spoolCycle = 0;
  spoolState = SPOOL_DRIVING;
  spoolT0    = millis();
  stepper.setDrive(SPOOL_DIR);
  Serial.println("{\"spool_seq\":1}");
}

static void spoolSeqUpdate() {
  if (spoolState == SPOOL_IDLE) return;
  const uint32_t now = millis();

  switch (spoolState) {
    case SPOOL_DRIVING:
      if (now - spoolT0 >= SPOOL_DRIVE_MS) {
        stepper.setDrive(0);
        spoolServo.setAngle(SPOOL_SPIN);   // START spinning
        spoolT0 = now;
        spoolState = SPOOL_SPINNING;
      }
      break;

    case SPOOL_SPINNING:
      if (now - spoolT0 >= SPOOL_SPIN_MS) {
        spoolServo.setAngle(SPOOL_STOP);   // STOP spinning — one round done
        if (++spoolCycle >= SPOOL_CYCLES) {
          spoolState = SPOOL_IDLE;
          Serial.println("{\"spool_seq\":0}");
        } else {
          stepper.setDrive(SPOOL_DIR);
          spoolT0 = now;
          spoolState = SPOOL_DRIVING;
        }
      }
      break;

    default:
      break;
  }
}
#endif  // ENABLE_SPOOL_SEQUENCE
// ===========================================================================


void setup() {
  Serial.begin(576000);

  // RESERVE LEDC TIMERS FOR THE SERVOS BEFORE ANY OF THEM ATTACH.
  // ESP32Servo allocates channels dynamically and cannot see raw ledcSetup()
  // calls made elsewhere, so it handed channel 0 to hookServo and actuator.cpp
  // then reconfigured that same channel underneath it — which is why moving a
  // servo drove the actuator and driving a motor moved a servo. Pinning the
  // servos to timers 0 and 1 keeps them clear of the LED (timer 2) and the
  // actuator (timer 3). See the channel map at the top of actuator.cpp.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  if (!hookServo.begin(PIN_SERVO, 180)) {
    Serial.println("ERROR: hook servo failed to attach!");
    setupSuccessful = false;
  }

  if (!spoolServo.begin(PIN_SPOOL, 90)) {
    Serial.println("ERROR: spool servo failed to attach!");
    setupSuccessful = false;
  }

   if (!stepper.begin(PIN_STEP, PIN_DIR, PIN_EN, STEP_HZ)) {
    Serial.println("ERROR: stepper timer failed to start!");
    setupSuccessful = false;
  }
    if (!actuator.begin(PIN_ACT_AIN1, PIN_ACT_AIN2, PIN_ACT_PWM, PIN_ACT_STBY)) {
    Serial.println("ERROR: actuator failed to attach!");
    setupSuccessful = false;
  }
    if (!cameras.begin(CAML_RX, -1, CAMR_RX, -1)) { 
    Serial.println("ERROR: camera buffers failed to attach!");
    setupSuccessful = false; 

  } if (!imu.begin(PIN_SDA, PIN_SCL)) {
  Serial.println("ERROR: IMU not found!");
  setupSuccessful = false;}


  if (!led.begin(PIN_LED)) {
    Serial.println("ERROR: LED failed to attach!");
    setupSuccessful = false;
  }


    // Blink LED twice to confirm setup completed successfully
  if (true) {
    for (int i = 0; i < 2; i++) {
      led.on();
      delay(500);
      led.off();
      delay(500);
    }
  }
  
  
}
void sendTelemetry() {
  Serial.print("{\"servo_angle\":");
  Serial.print(hookServo.getCurrentAngle());

  Serial.print("{\"steps\":");
  Serial.print(stepper.getStepCount());
  
  Serial.print("{\"stepper_drive_dir\":");
  Serial.print(stepper.getDrive());
  
  Serial.print("{\"actuator\":");
  Serial.print(actuator.getState());

  Serial.print("{\"imu_pos\":");
  Serial.print(imu.getPosition());

  Serial.print("{\"imu_vel\":");
  Serial.print(imu.getVelocity());
  
  Serial.print("{\"imu_bias\":");
  Serial.print(imu.getBias());
  
  Serial.print("{\"slip\":");
  Serial.print(imu.isSlipping());

  Serial.print("{\"led\":");
  Serial.print(led.isOn() ? led.brightness() : 0);

  Serial.println("}");
}

void loop() {

  cameras.update();
  imu.setDriveActive(stepper.getDrive() != 0);
  imu.update();

  while (Serial.available()) {
    char c = Serial.read();

    //blink light if setup successful

#if ENABLE_SPOOL_SEQUENCE
    // spool sequence: ANY other key aborts a run in progress, so you never
    // have to hunt for the right button to stop it. Checked BEFORE the
    // switch so it applies to every command including STOP.
    if (c != 'P') spoolSeqAbort();
#endif

    switch (c) {

#if ENABLE_SPOOL_SEQUENCE
      // spool sequence: start the automated wind-in
      case 'P': spoolSeqStart(); break;
#endif

      //-- servo --
      case 'H': hookServo.setAngle(180); break;
      case 'U': hookServo.setAngle(0); break;
      case 'A': hookServo.setAngle(90); break;

      // -- spool --
      case 'T': spoolServo.setAngle(180); break;
      case 'Y': spoolServo.setAngle(90); break;

      // -- stepper--
      case 'F': stepper.setDrive(+1); break;
      case 'B': stepper.setDrive(-1); break;
      case 'S': stepper.setDrive(0);  break;
      
      // -- actuator --
      case 'D': actuator.extend(ACT_SPEED);  break;  // deploy
      case 'R': actuator.retract(ACT_SPEED); break;  // retract
      case 'X': actuator.stop();             break;

      // -- IMU --
      case 'Z': stepper.zero(); imu.zero(); break;

     // -- LED --
      case 'L': led.on();  break;
      case 'O': led.off(); break;

      default:
        // Digits set lamp brightness: '0' = off ... '9' = full.
        // Single characters keep the protocol as-is — no length-prefixed
        // parameter parsing needed, and it stays typeable by hand.
        if (c >= '0' && c <= '9') {
          led.setBrightness((uint8_t)((c - '0') * 255 / 9));
        }
        break;   // ignore anything else
    }
  }

  actuator.update();

#if ENABLE_SPOOL_SEQUENCE
  spoolSeqUpdate();   // spool sequence: advances one step, never blocks
#endif

  if (millis() - lastTelemetry >= TELEM_INTERVAL_MS) {
     lastTelemetry = millis();
     sendTelemetry();
  }
}


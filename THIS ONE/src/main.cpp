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

// SWAPPED (was L=18, R=16). The panes were the wrong way round: the camera
// mounted on the left was arriving as id 1 and showing in the right pane.
// Fixing it here rather than in a panel means every consumer — both panels,
// stereo_calibrate.py, the recordings — agrees, instead of one being patched
// and the rest staying wrong.
//
// NOTE: ROTATE_DEG had to swap to match. Rotation belongs to the physical
// camera (how it is mounted), so when the ids swap the rotations swap with
// them: (90, 270) -> (270, 90) in all four Python files.
int CAML_RX = 18;   // left cam U0T  -> S3 GPIO 18
int CAMR_RX = 16;   // right cam U0T -> S3 GPIO 16

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
int STEP_HZ = 50;

// Calibrated in the pipe with the robot fully assembled, 7 runs from
// 32cm-73cm each. Weighted average (total steps / total distance):
// 12925 steps / 4012mm = 3.222. Individual run ratios ranged 3.13-3.31,
// no outliers discarded. Re-calibrate if wheels, gearing, or the pipe
// surface change (this is a real physical measurement, not derived from
// geometry, since wheel slip/preload can't be computed reliably).
float STEPS_PER_MM = 3.222f;

//ACTUATOR VARIABLES
int ACT_SPEED = 50;   // 0-255, default speed for jog commands

//OTHER VARIABLES
bool setupSuccessful = true;

const uint32_t TELEM_INTERVAL_MS = 200;   // 5 Hz
uint32_t lastTelemetry = 0;


void setup() {
  Serial.begin(460800);

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


  // ---- startup summary ----
  // Printed once so "it isn't working" can be answered by reading the boot
  // log instead of guessing. Watch it with:  python3 robot_console.py
  // then press the S3 reset button.
  Serial.println("---- EDI PIPE CAM boot ----");
  Serial.printf("hook servo  GPIO %2d  LEDC ch %d\n",
                PIN_SERVO, hookServo.getChannel());
  Serial.printf("spool servo GPIO %2d  LEDC ch %d\n",
                PIN_SPOOL, spoolServo.getChannel());
  Serial.printf("LED         GPIO %2d  LEDC ch 4  (timer 2)\n", PIN_LED);
  Serial.printf("actuator    GPIO %2d  LEDC ch 6  (timer 3)\n", PIN_ACT_PWM);
  Serial.printf("cameras     L GPIO %d, R GPIO %d @ %lu baud\n",
                CAML_RX, CAMR_RX, (unsigned long)StereoCameras::CAM_BAUD);
  // A servo channel of -1 means attach failed; two servos on the SAME channel,
  // or a servo on 4 or 6, means something is stealing the other's output.
  if (hookServo.getChannel() == spoolServo.getChannel()) {
    Serial.println("*** BOTH SERVOS ON THE SAME LEDC CHANNEL ***");
  }
  if (hookServo.getChannel() == 4 || hookServo.getChannel() == 6 ||
      spoolServo.getChannel() == 4 || spoolServo.getChannel() == 6) {
    Serial.println("*** A SERVO IS ON THE LED OR ACTUATOR CHANNEL ***");
  }
  Serial.println(setupSuccessful ? "setup OK" : "SETUP HAD ERRORS (above)");
  Serial.println("---------------------------");

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

  Serial.print("{\"pos_mm\":");
  Serial.print(stepper.getStepCount() / STEPS_PER_MM, 1);

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

    switch (c) {

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


  if (millis() - lastTelemetry >= TELEM_INTERVAL_MS) {
     lastTelemetry = millis();
     sendTelemetry();
  }
}
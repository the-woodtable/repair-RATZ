#include <Arduino.h>
#include "HookServo.h"
#include "Stepper.h"
#include "Actuator.h"
#include "StereoCameras.h"
#include "IMU.h"

//PINS
int PIN_SERVO = 11;
int PIN_STEP =  4;
int PIN_DIR = 5;
int PIN_EN =  6;

int PIN_ACT_AIN1 = 7;
int PIN_ACT_AIN2 = 8;
int PIN_ACT_PWM  = 9;    // TB6612 PWMA — set to your actual pin
int PIN_ACT_STBY = 10;   // TB6612 STBY — set to your actual pin

int CAML_RX = 18;   // left cam U0T  -> S3
int CAMR_RX = 16;   // right cam U0T -> S3
 
int PIN_SDA = 8;   
int PIN_SCL = 9;  

//OBJECTS FROM CLASSES
HookServo hookServo;
Stepper stepper;
Actuator actuator;
StereoCameras cameras; 
IMU imu;


//STEPPER VARIABLES
int STEP_HZ = 800;

 
//ACTUATOR VARIABLES
int ACT_SPEED = 200;   // 0-255, default speed for jog commands
 

//OTHER VARIABLES
bool setupSuccessful = true;

const uint32_t TELEM_INTERVAL_MS = 200;   // 5 Hz
uint32_t lastTelemetry = 0;



void setup() {
  Serial.begin(460800);

  if (!hookServo.begin(PIN_SERVO)) {
    Serial.println("ERROR: hook servo failed to attach!");
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
  
  
}

void sendTelemetry() {
  char js[160];
  snprintf(js, sizeof(js),
    "{\"servo_angle\":%d,\"steps\":%ld,\"drv\":%d,\"act\":%d,"
    "\"imu_pos\":%.4f,\"imu_vel\":%.4f,\"imu_bias\":%.5f,\"slip\":%d}",
    hookServo.getCurrentAngle(), (long)stepper.getStepCount(),
    stepper.getDrive(), actuator.getState(),
    imu.getPosition(), imu.getVelocity(), imu.getBias(), imu.isSlipping());
  cameras.sendTelemetryFrame(js);
}

void loop() {
  cameras.update();
  imu.setDriveActive(stepper.getDrive() != 0);
  imu.update();

  while (Serial.available()) {
    char c = Serial.read();

    //blink light if setup successful
    if (cameras.handleChar(c)) continue;
    if (!setupSuccessful) continue;

    switch (c) {

      //servo
      case 'H': hookServo.setAngle(180); break;
      case 'U': hookServo.setAngle(0); break;
      case 'A': hookServo.setAngle(90); break;

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
 

      default:
        break;   // ignore unrecognized characters
    }
  }

  if (millis() - lastTelemetry >= TELEM_INTERVAL_MS) {
    lastTelemetry = millis();
    sendTelemetry();
  }
}


#include <Arduino.h>
#include "HookServo.h"
#include "Stepper.h"

//PINS
int PIN_SERVO = 11;
int PIN_STEP =  4;
int PIN_DIR = 5;
int PIN_EN =  6;


//OBJECTS FROM CLASSES
HookServo hookServo;
Stepper stepper;


//STEPPER VARIABLES
int STEP_HZ = 800;



//OTHER VARIABLES
bool setupSuccessful = true;

const uint32_t TELEM_INTERVAL_MS = 200;   // 5 Hz
uint32_t lastTelemetry = 0;



void setup() {
  Serial.begin(921600);

  if (!hookServo.begin(PIN_SERVO)) {
    Serial.println("ERROR: hook servo failed to attach!");
    setupSuccessful = false;
  }

   if (!stepper.begin(PIN_STEP, PIN_DIR, PIN_EN, STEP_HZ)) {
    Serial.println("ERROR: stepper timer failed to start!");
    setupSuccessful = false;
  }
  
}

void sendTelemetry() {
  Serial.print("{\"servo_angle\":");
  Serial.print(hookServo.getCurrentAngle());
  Serial.print(",\"stepper steps\":");
  Serial.print(stepper.getStepCount());
  Serial.print(",\"stepper direction\":");
  Serial.print(stepper.getDrive());
  Serial.println("}");
}

void loop() {

  while (Serial.available()) {
    char c = Serial.read();

    //blink light if setup successful
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
      case 'Z': stepper.zero(); break;

      default:
        break;   // ignore unrecognized characters
    }
  }

  if (millis() - lastTelemetry >= TELEM_INTERVAL_MS) {
    lastTelemetry = millis();
    sendTelemetry();
  }
}


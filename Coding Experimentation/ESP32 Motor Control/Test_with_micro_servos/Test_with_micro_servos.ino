#include <Arduino.h>
#include <ESP32Servo.h>

Servo leftServo;
Servo rightServo;

int leftServoPin = 5;
int rightServoPin = 6;

int leftAngle = 90;   // start neutral
int rightAngle = 90;

void setup() {
  Serial.begin(115200);
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
  leftServo.write(leftAngle);
  rightServo.write(rightAngle);
  Serial.println("ESP32 ready. Commands: F=Step same, B=Step opposite, L=Left step, R=Right step, S=Stop");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'F') { // step both forward together
      leftAngle = constrain(leftAngle + 5, 0, 180);
      rightAngle = constrain(rightAngle + 5, 0, 180);
    }
    if (cmd == 'B') { // step opposite directions
      leftAngle = constrain(leftAngle + 5, 0, 180);
      rightAngle = constrain(rightAngle - 5, 0, 180);
    }
    if (cmd == 'L') { // step left only
      leftAngle = constrain(leftAngle + 5, 0, 180);
    }
    if (cmd == 'R') { // step right only
      rightAngle = constrain(rightAngle + 5, 0, 180);
    }
    if (cmd == 'S') { // reset to neutral
      leftAngle = 90;
      rightAngle = 90;
    }

    leftServo.write(leftAngle);
    rightServo.write(rightAngle);
    Serial.printf("Angles: L=%d, R=%d\n", leftAngle, rightAngle);
  }
}

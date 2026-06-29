#include <Arduino.h>
#include <ESP32Servo.h>

Servo leftServo;
Servo rightServo;

int leftServoPin = 5;   // adjust to your wiring
int rightServoPin = 6;  // adjust to your wiring

void setup() {
  Serial.begin(115200);
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
  stopCar();
  Serial.println("ESP32 ready. Commands: F=Same, B=Opposite, L=Left only, R=Right only, S=Stop");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'F') sweepSameDirection();
    if (cmd == 'B') sweepOppositeDirection();
    if (cmd == 'L') sweepLeftOnly();
    if (cmd == 'R') sweepRightOnly();
    if (cmd == 'S') stopCar();
  }
}

// Sweep both servos together in same direction
void sweepSameDirection() {
  Serial.println("Sweep same direction");
  for (int pos = 0; pos <= 180; pos++) {
    leftServo.write(pos);
    rightServo.write(pos);
    delay(15);
  }
  for (int pos = 180; pos >= 0; pos--) {
    leftServo.write(pos);
    rightServo.write(pos);
    delay(15);
  }
}

// Sweep both servos together in opposite directions
void sweepOppositeDirection() {
  Serial.println("Sweep opposite direction");
  for (int pos = 0; pos <= 180; pos++) {
    leftServo.write(pos);
    rightServo.write(180 - pos);
    delay(15);
  }
  for (int pos = 180; pos >= 0; pos--) {
    leftServo.write(pos);
    rightServo.write(180 - pos);
    delay(15);
  }
}

// Sweep left motor only
void sweepLeftOnly() {
  Serial.println("Sweep left motor only");
  for (int pos = 0; pos <= 180; pos++) {
    leftServo.write(pos);
    delay(15);
  }
  for (int pos = 180; pos >= 0; pos--) {
    leftServo.write(pos);
    delay(15);
  }
}

// Sweep right motor only
void sweepRightOnly() {
  Serial.println("Sweep right motor only");
  for (int pos = 0; pos <= 180; pos++) {
    rightServo.write(pos);
    delay(15);
  }
  for (int pos = 180; pos >= 0; pos--) {
    rightServo.write(pos);
    delay(15);
  }
}

// Stop both motors
void stopCar() {
  Serial.println("Stop");
  leftServo.write(90);
  rightServo.write(90);
}

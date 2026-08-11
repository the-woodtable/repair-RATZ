/*
  servo_test.ino — isolate ONE servo, nothing else running
  =========================================================
  Flash this to the S3 instead of the robot firmware. No cameras, no stepper,
  no actuator, no LED — so if the servo misbehaves here, it is the servo, its
  wiring, or its power, and nothing to do with the rest of the project.

  USE THE ESP32Servo LIBRARY, NOT Arduino's bundled "Servo".
    Arduino IDE -> Tools -> Manage Libraries -> search "ESP32Servo"
                   by Kevin Harrington / John K. Bennett -> Install

  Arduino's own Servo library lists avr, samd, stm32... but NOT esp32, which
  is what that warning was telling you. The copy on your machine also fails
  to compile against your core because SOC_LEDC_TIMER_BIT_WIDE_NUM was
  renamed to SOC_LEDC_TIMER_BIT_WIDTH in newer arduino-esp32 versions. Do not
  try to patch it; just use ESP32Servo, which is what the robot firmware uses.

  WIRING — a servo needs its own power, not the S3's 3.3 V:
      servo signal (orange/white) -> GPIO 11   (SERVO_PIN below)
      servo +      (red)          -> 5 V, external supply
      servo -      (brown/black)  -> that supply's ground
      supply ground -------------- S3 GND      <- REQUIRED, common reference

  A servo can pull 500 mA+ when it stalls. Powered from a weak rail it will
  twitch, buzz, or sit still while everything in software looks perfect.

  Open Serial Monitor at 115200. It prints the channel it was given and the
  angle it is commanding, so you can tell "not commanded" from "commanded but
  not moving" — which are completely different faults.
*/

#include <ESP32Servo.h>

const int SERVO_PIN = 11;    // hook servo. Use 15 to test the spool servo.
const int MIN_US    = 500;   // matches HookServo::begin in the robot firmware
const int MAX_US    = 2500;

Servo servo;
int channel = -1;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- servo isolation test ---");

  // Reserve LEDC timers for the servo library, exactly as main.cpp does.
  // Without this ESP32Servo can hand out a channel something else has taken.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  servo.setPeriodHertz(50);              // standard hobby servo frame
  channel = servo.attach(SERVO_PIN, MIN_US, MAX_US);

  // ESP32Servo returns 0 on failure, NOT -1 — the mistake that was hiding
  // this exact problem in the robot firmware.
  if (channel <= 0) {
    Serial.printf("ATTACH FAILED on GPIO %d (returned %d)\n",
                  SERVO_PIN, channel);
    Serial.println("-> no free PWM channel, or the pin cannot output.");
    Serial.println("-> This is a SOFTWARE fault. Try another GPIO.");
    return;
  }
  Serial.printf("attached: GPIO %d, LEDC channel %d, 50 Hz, %d-%d us\n",
                SERVO_PIN, channel, MIN_US, MAX_US);
  Serial.println("sweeping 0 -> 180 -> 0. Watch the horn.\n");
}

void loop() {
  if (channel <= 0) {          // attach failed; don't pretend to sweep
    delay(1000);
    return;
  }

  // Step in 30-degree jumps with a pause, rather than a smooth sweep: a
  // discrete move is obvious even if the servo is slow, weak or juddering,
  // and the pause lets you hear whether it is straining.
  for (int a = 0; a <= 180; a += 30) {
    Serial.printf("angle %3d\n", a);
    servo.write(a);
    delay(700);
  }
  for (int a = 150; a > 0; a -= 30) {
    Serial.printf("angle %3d\n", a);
    servo.write(a);
    delay(700);
  }
}

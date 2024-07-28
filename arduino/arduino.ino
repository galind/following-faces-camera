#include <Servo.h>

Servo myServo;
int servoPin = 9;
int angle = 90; // Start at a neutral position

void setup() {
  myServo.attach(servoPin);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    angle = Serial.parseInt(); // Read angle value from serial
    angle = constrain(angle, 0, 180); // Constrain to valid range
    myServo.write(angle);
  }
}
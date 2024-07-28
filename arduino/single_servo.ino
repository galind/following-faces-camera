#include <Servo.h>

Servo myServo;  // Create a servo object
int currentAngle = 90;  // Starting position of the servo (assumed to be the center)

void setup() {
  myServo.attach(9);  // Attach the servo to pin 9
  myServo.write(currentAngle);  // Set initial position
  Serial.begin(9600); // Start serial communication at 9600 baud
}

void loop() {
  if (Serial.available() > 0) {
    String angleStr = Serial.readStringUntil('\n');  // Read the incoming serial data until newline
    int targetAngle = angleStr.toInt();  // Convert the string to an integer

    // Move the servo in small increments for smoother movement
    while (currentAngle != targetAngle) {
      if (currentAngle < targetAngle) {
        currentAngle++;
      } else if (currentAngle > targetAngle) {
        currentAngle--;
      }
      myServo.write(currentAngle);  // Set the servo to the current angle
      delay(10);  // Wait for a short period to make the movement smooth
    }
  }
}

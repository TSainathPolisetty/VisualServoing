#include <Servo.h>

Servo panServo;
Servo tiltServo;

const int panPin = 9;
const int tiltPin = 10;
int centerX = 320; // Half of the camera's width resolution (640 / 2)
int centerY = 240; // Half of the camera's height resolution (480 / 2)

int panCenter = 90; // Pan servo center position
int tiltCenter = 90; // Tilt servo center position

void setup() {
  Serial.begin(9600);
  panServo.attach(panPin);
  tiltServo.attach(tiltPin);
  panServo.write(panCenter);
  tiltServo.write(tiltCenter);
}

void loop() {
  if (Serial.available()) {
    int objectX = Serial.parseInt();
    int objectY = Serial.parseInt();

    if (Serial.read() == '\n') {
      int deltaX = centerX - objectX;
      int deltaY = centerY - objectY;

      int panAngle = panCenter + (deltaX / 10); // Modify the divisor (10) to adjust the servo's speed
      int tiltAngle = tiltCenter + (deltaY / 10); // Modify the divisor (10) to adjust the servo's speed

      // Limit the angles to the servo's range (0-180)
      panAngle = constrain(panAngle, 0, 180);
      tiltAngle = constrain(tiltAngle, 0, 180);

      // panServo.write(panAngle);
      // tiltServo.write(tiltAngle);

      // Print the servo angles
      Serial.print(panAngle);
      Serial.print(',');
      Serial.print(tiltAngle);
      Serial.print('\n');


    }
  }
}



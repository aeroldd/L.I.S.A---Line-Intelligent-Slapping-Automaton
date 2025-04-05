/*
SLAPPER CALIBRATION
This sketch is used to calibrate the minimum and maximum angle for slapping in L.I.S.A's obstacle avoidance system.
The right minimum and maximum angles must be determined so that the internal support structures in the chasis wouldnt get slapped on the way.

This sketch uses a potentiometer to set the angle and a button to confirm the angle
Once the program is finished running, the program prints out the minimum and maximum angles of the slap.
These values will be used in the slapper arm in the final L.I.S.A source code.
*/

#include <Servo.h>
float angle;

// Global variables

Servo servo;
const int servoPin = 3;

float minAngle = 0;
float maxAngle = 180;

int button;

void setMinAngle();
void setMaxAngle();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  servo.attach(servoPin);

  while(!button) button = digitalRead(8);
  setMinAngle();
  setMaxAngle();
  Serial.println("slapper calibration finished");
  Serial.print("Minimum angle: ");
  Serial.println(minAngle + 10);
  Serial.print("Maximum angle: ");
  Serial.println(maxAngle - 10);

}

void setMinAngle() {
  Serial.println("Set the minimum angle on the potentiometer. Enter c to calibrate.");
  delay(1000);
  int calibrated = 0;
  while(!calibrated) {
    button = digitalRead(8);
    minAngle = (analogRead(2)/1023.0) * 180.0;
    Serial.print("reading: ");
    Serial.println(minAngle);

    servo.write(minAngle);

    if(button) calibrated = 1;
    
    delay(50);
  }
  Serial.print("Minimum angle set to ");
  Serial.println(minAngle);
}

void setMaxAngle() {
  Serial.println("Set the maximum angle on the potentiometer. Enter c to calibrate.");
  int calibrated = 0;
  delay(1000);
  while(!calibrated) {
    button = digitalRead(8);
    maxAngle = (analogRead(2)/1023.0) * 180.0;
    Serial.print("reading: ");
    Serial.println(maxAngle);

        if(button) calibrated = 1;

    servo.write(maxAngle);
    
    delay(50);
  }
  Serial.print("Maxmimum angle set to ");
  Serial.println(maxAngle);
}

void slap() {
  Serial.println("slapping");
  servo.write(minAngle);
  delay(1000);
  servo.write(maxAngle);
  delay(1000);
  servo.write(minAngle);
  delay(1000);
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 's') {
      slap();
    }
  }
}

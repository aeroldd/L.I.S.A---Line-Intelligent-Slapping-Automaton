/*
This sketch was used in testing the slapping mechanism of L.I.S.A.

Uses an ultrasonic sensor -> gets the distance -> if the distance is in the sapping threshold, it will slap the object infront of it.
*/

#include <NewPing.h>
#include <Servo.h>
 
#define TRIGGER_PIN 12
#define ECHO_PIN 13
#define MAX_DISTANCE 200

#define STOP_THRESHOLD 10
 
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
 
Servo servo;
int distance;

void setup() {
   Serial.begin(9600);
   servo.attach(3);
  servo.write(0);
}

void slap() {
    servo.write(0);
    Serial.println((servo.read()));
    delay(2000);
    servo.write(180);
    Serial.println((servo.read()));
    delay(2000);
}
 
void loop() {
  delay(200);
  distance = sonar.ping_cm(); // Gets the distance from the obstacle
  Serial.print(distance); // Prints out the distance from the obstacle
  Serial.println("cm");

  if(distance < STOP_THRESHOLD && (distance != 0)) { // If the distance is in the slapping threshold it will slap the object
    slap();
  }
}
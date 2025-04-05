/*
Read 3 IR sensors for callibration
*/
// Read 3 IR sensors for callibration
//
/*
  AnalogReadSerial

  Reads an analog input on pin 3,4 &5, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  
  Adapted from the example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogReadSerial
*/

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensorValueLeft = analogRead(A5);
  delay(2);
   int sensorValueCentre = analogRead(A4);
  delay(2);
   int sensorValueRight = analogRead(A3);
  delay(2);
  // print out the value you read:
  Serial.print("");
  Serial.print(sensorValueLeft);
   Serial.print(",");
  Serial.print(sensorValueCentre);
   Serial.print(",");
  Serial.println(sensorValueRight);
  delay(1000);        // delay in between reads 
}
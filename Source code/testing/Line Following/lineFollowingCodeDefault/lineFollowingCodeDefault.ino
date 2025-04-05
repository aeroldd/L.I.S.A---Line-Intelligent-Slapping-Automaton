/*
Line following control of two Bi-directional Motors
*/
// WheelDrive - move a pair of DC motors at varying rate and direction
//
// Copyright (c) 2016, Garth Zeglin.  All rights reserved. Licensed under the
// terms of the BSD 3-clause license as included in LICENSE.
//
// This program assumes that:
//
//  1. A DRV8833 dual DC motor driver module is connected to pins 5, 6, 9, and 10.
//  2. A pair of motors is attached to the driver.
//  3. The serial console on the Arduino IDE is set to 9600 baud communications speed.

// ================================================================================
// Define constant values and global variables.

// Define the pin numbers on which the outputs are generated.
#define MOT_A1_PIN 5
#define MOT_A2_PIN 6
#define MOT_B1_PIN 9
#define MOT_B2_PIN 10

boolean reverseL = 0; 
boolean reverseR = 0;

/* variables to keep track of current speed of motors */
int leftServoSpeed = 0;
int rightServoSpeed = 0;

/* Define the pins for the IR sensors */
const int irPins[3] = {A5, A4, A3};

/* Define values for the IR Sensor readings */
int irSensorDigital[3] = {0, 0, 0};

int threshold = 600; // IR sensor threshold value for line detection


const int maxSpeed = 180; // the range for speed is(0,255)

// binary representation of the sensor reading
// 1 when the sensor detects the line, 0 otherwise
int irSensors = B000;

// A score to determine deviation from the line [-180 ; +180].
// Negative means the robot is left of the line.
int error = 0;

int errorLast = 0;  //  store the last value of error


// ================================================================================
/// Configure the hardware once after booting up.  This runs once after pressing
//// reset or powering up the board.
void setup(void)
{
  // Initialize the stepper driver control pins to output drive mode.
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);
  pinMode(MOT_B1_PIN, OUTPUT);
  pinMode(MOT_B2_PIN, OUTPUT);

  // Start with drivers off, motors coasting.
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);
  digitalWrite(MOT_B1_PIN, LOW);
  digitalWrite(MOT_B2_PIN, LOW);

  // Initialize the serial UART at 9600 bits per second.
  Serial.begin(9600);
}
// ================================================================================
/// Set the current on a motor channel using PWM and directional logic.
/// Changing the current will affect the motor speed, but please note this is
/// not a calibrated speed control.  This function will configure the pin output
/// state and return.
///
/// \param pwm    PWM duty cycle ranging from -255 full reverse to 255 full forward
/// \param IN1_PIN  pin number xIN1 for the given channel
/// \param IN2_PIN  pin number xIN2 for the given channel

void set_motor_pwm(int pwm, int IN1_PIN, int IN2_PIN)
{
  if (pwm < 0) {  // reverse speeds
    analogWrite(IN1_PIN, -pwm);
    digitalWrite(IN2_PIN, LOW);

  } else { // stop or forward
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, pwm);
  }
}
// ================================================================================
/// Set the current on both motors.
///
/// \param pwm_A  motor A PWM, -255 to 255
/// \param pwm_B  motor B PWM, -255 to 255

void set_motor_currents(int pwm_A, int pwm_B)
{
  set_motor_pwm(pwm_A, MOT_A1_PIN, MOT_A2_PIN);
  set_motor_pwm(pwm_B, MOT_B1_PIN, MOT_B2_PIN);

  // Print a status message to the console.
  // Serial.print("Set motor A PWM = ");
  // Serial.print(pwm_A);
  // Serial.print(" motor B PWM = ");
  // Serial.println(pwm_B);
}

// ================================================================================
/// Simple primitive for the motion sequence to set a speed and wait for an interval.
///
/// \param pwm_A  motor A PWM, -255 to 255
/// \param pwm_B  motor B PWM, -255 to 255
/// \param duration delay in milliseconds
void spin_and_wait(int pwm_A, int pwm_B, int duration)
{
  set_motor_currents(pwm_A, pwm_B);
  delay(duration);
}

void Scan() {
  // Initialize the sensors
    irSensors = B000;

  for (int i = 0; i < 3; i++) {
      int sensorValue = analogRead(irPins[i]);
      if (sensorValue >= threshold) {
        irSensorDigital[i] = 1;
        Serial.print((sensorValue > threshold));
        Serial.print(" ");
      }

    else {
      irSensorDigital[i] = 0;
      Serial.print(sensorValue > threshold);
      Serial.print(" ");
    }

    int b = 2-i;
    irSensors = irSensors + (irSensorDigital[i]<<b);
    }
    //Serial.println();
}

void UpdateDirection() {

  errorLast = error;

  switch (irSensors) {

    case B000: // no sensor detects the line
       if (errorLast < 0) { error = -130;}
       else if (errorLast > 0) {error = 130;}
       break;

     case B100: // left sensor on the line
       error = -80;
       break;

     case B110:
       error = -40;
       break;

     case B010:
       error = 0;
       break;

     case B011:
       error = 40;
       break;

     case B001: // right sensor on the line
       error = 80;
       break;

     case B111:
       error = 1;
       break;

     default:
       error = errorLast;
  }

    if (error >= 0) {
      leftServoSpeed = maxSpeed;
      rightServoSpeed = maxSpeed - error;
    }

    else if (error < 0) {
      leftServoSpeed = maxSpeed + error;
      rightServoSpeed = maxSpeed;
    }
}


void loop()
{
Scan();
UpdateDirection();
spin_and_wait(leftServoSpeed,rightServoSpeed,10); // sets speed for 0.01 sec
Serial.print("left: ");
Serial.print(leftServoSpeed);
Serial.print(" right: ");
Serial.print(rightServoSpeed);
Serial.println();

}
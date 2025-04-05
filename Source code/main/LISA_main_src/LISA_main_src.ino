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

#include <NewPing.h>
#include <Servo.h> 
#include <DFMiniMp3.h>
#include <SoftwareSerial.h>

// ===== MOTOR CONTROL PINS =====
#define MOT_A1_PIN 5
#define MOT_A2_PIN 6
#define MOT_B1_PIN 9
#define MOT_B2_PIN 10

// ===== SENSOR PINS =====
const int irPins[3] = { A5, A4, A3 };
#define TRIGGER_PIN 12
#define ECHO_PIN 13

// ===== SERVO CONTROL PIN =====
#define SERVO_PIN 3

// ===== LED CONTROL PIN =====
#define LED_CONTROL_PIN 7

// ===== AUDIO PINS =====
#define AUDIO_RX 2
#define AUDIO_TX 4

// ===== CONSTANTS =====
int threshold = 400;  // IR sensor threshold value for line detection
const int maxSpeed = 200;  // the range for speed is(0,255)
#define MAX_DISTANCE 200
#define STOP_THRESHOLD 10 // the buggy stops and slaps when the object is 15 cm or closer to the sensor

#define SPEAKER_VOLUME 30

#define WARNING_DURATION 250

// ===== GLOBAL VARIABLES =====
// motor global variables
boolean reverseL = 0;
boolean reverseR = 0;
int leftServoSpeed = 0;
int rightServoSpeed = 0;
const int motorOffset = 12; 

// binary representation of the sensor reading
// 1 when the sensor detects the line, 0 otherwise
int irSensorDigital[3] = { 0, 0, 0 };
int irSensors = B000;

// A score to determine deviation from the line [-180 ; +180].
// Negative means the robot is left of the line.
int error = 0;
int errorLast = 0;  //  store the last value of error

// Ultrasonic sensor distance global variables
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
int distance;

// Define servo
Servo servo;

// Audio things
SoftwareSerial secondarySerial(4,2); // RX, TX
class Mp3Notify; 
typedef DFMiniMp3<SoftwareSerial, Mp3Notify> DfMp3;
DfMp3 dfmp3(secondarySerial);
int audioFinished;

// Class used to handle notifying stuff to the serial monitor

class Mp3Notify
{
public:
  static void PrintlnSourceAction(DfMp3_PlaySources source, const char* action)
  {
    if (source & DfMp3_PlaySources_Sd) 
    {
        Serial.print("SD Card, ");
    }
    if (source & DfMp3_PlaySources_Usb) 
    {
        Serial.print("USB Disk, ");
    }
    if (source & DfMp3_PlaySources_Flash) 
    {
        Serial.print("Flash, ");
    }
    Serial.println(action);
  }
  static void OnError([[maybe_unused]] DfMp3& mp3, uint16_t errorCode)
  {
    // see DfMp3_Error for code meaning
    Serial.println();
    Serial.print("Com Error ");
    Serial.println(errorCode);
  }
  static void OnPlayFinished([[maybe_unused]] DfMp3& mp3, [[maybe_unused]] DfMp3_PlaySources source, uint16_t track)
  {
    Serial.print("tester");
    Serial.print("Play finished for #");
    Serial.println(track);  
    audioFinished = 1;
  }
  static void OnPlaySourceOnline([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source)
  {
    PrintlnSourceAction(source, "online");
  }
  static void OnPlaySourceInserted([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source)
  {
    PrintlnSourceAction(source, "inserted");
  }
  static void OnPlaySourceRemoved([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source)
  {
    PrintlnSourceAction(source, "removed");
  }
};

// ===== BUGGY STATES =====
#define MOVING 0
#define WARNING 1
#define SLAPPING 2

int state;
int previousState;
int warningCount = WARNING_DURATION;

// ================================================================================
void setup(void) {
  // Initialize the stepper driver control pins to output drive mode.
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);
  pinMode(MOT_B1_PIN, OUTPUT);
  pinMode(MOT_B2_PIN, OUTPUT);

  // Start with drivers off
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);
  digitalWrite(MOT_B1_PIN, LOW);
  digitalWrite(MOT_B2_PIN, LOW);

  // Initialize the serial UART at 9600 bits per second.
  Serial.begin(9600);

  // Initialize the servo motor to the 0 position
  pinMode(SERVO_PIN, OUTPUT);
  initServoMotors();

  // Initialize the ultrasonic sensors
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);


  // Initialize the LEDS
  pinMode(LED_CONTROL_PIN, OUTPUT);
  // Blink once as a test
  digitalWrite(LED_CONTROL_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_CONTROL_PIN, LOW);

  // Initialise the DFPlayerMini
  initDFPlayer();

  Serial.println("Buggy initialized.");
}

void initServoMotors() {
  newServoAttach(SERVO_PIN);
  servo.write(0);
  newServoDetach();
}

void initDFPlayer() {
  Serial.println("Initialising DFPlayerMini audio chip...");
  dfmp3.begin();

  uint16_t version = dfmp3.getSoftwareVersion();
  Serial.print("version ");
  Serial.println(version);

  uint16_t volume = dfmp3.getVolume();
  Serial.print("volume ");
  Serial.println(volume);
  dfmp3.setVolume(SPEAKER_VOLUME);
  
  uint16_t count = dfmp3.getTotalTrackCount(DfMp3_PlaySource_Sd);
  Serial.print("files ");
  Serial.println(count);

  playInit();
}

// ===== AUDIO PLAYING FUNCTIONS =====

int playInit() {
  playStartUp();
  playGreeting();
  delay(200);
}

int playStartUp() {
  audioFinished = 0;
  Serial.println("play start up noise");
  dfmp3.playFolderTrack16(1, 1);
  delay(100);
  waitForAudioToFinish();
  return 1;
}

int playGreeting() {
  audioFinished = 0;
  Serial.println("play greeting");
  dfmp3.playFolderTrack16(1,2);
    delay(100);
  waitForAudioToFinish();
  return 1;
}

int playLineFollowing() {
  audioFinished = 0;
  Serial.println("play line following");
  dfmp3.playFolderTrack16(2,1);
  delay(100);
  waitForAudioToFinish();
  return 1;
}

int playBeep() {
  audioFinished = 0;
  Serial.println("play beep");
  dfmp3.playFolderTrack16(3,1);
  return 1;
}

int playWarning() {
  audioFinished = 0;
  Serial.println("play warning");
  dfmp3.playFolderTrack16(3,2);
  delay(100);
  //waitForAudioToFinish();
  return 1;
}

int playSlapStart() {
  audioFinished = 0;
  Serial.println("play slap start");
  dfmp3.playFolderTrack16(3,3);
  delay(100);
  waitForAudioToFinish();
  return 1;
}

int playSlapFinish() {
  Serial.println("play slap finish");
  dfmp3.playFolderTrack16(3,4);
  delay(100);
  waitForAudioToFinish();
  return 1;
}

void waitForAudioToFinish() {
  DfMp3_Status status = dfmp3.getStatus(); // Get playback status
  Serial.println(status.state); // Print status as an integer
  if(status.state==0) {
    Serial.println("broken");
    status = dfmp3.getStatus();
  }
  while(status.state){
    status = dfmp3.getStatus();
    Serial.println("waiting for the track to finish");
    delay(100);
    dfmp3.loop();
  }
  dfmp3.loop();
  delay(100);
}

// ================================================================================
/// Set the current on a motor channel using PWM and directional logic.
/// Changing the current will affect the motor speed, but please note this is
/// not 3a calibrated speed control.  This function will configure the pin output
/// state and return.
///
/// \param pwm    PWM duty cycle ranging from -255 full reverse to 255 full forward
/// \param IN1_PIN  pin number xIN1 for the given channel
/// \param IN2_PIN  pin number xIN2 for the given channel

void set_motor_pwm(int pwm, int IN1_PIN, int IN2_PIN) {
  if (pwm < 0) {  // reverse speeds
    analogWrite(IN1_PIN, -pwm);
    digitalWrite(IN2_PIN, LOW);

  } else {  // stop or forward
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, pwm);
  }
}

// ================================================================================
/// Set the current on both motors.
///
/// \param pwm_A  motor A PWM, -255 to 255
/// \param pwm_B  motor B PWM, -255 to 255

void set_motor_currents(int pwm_A, int pwm_B) {
  set_motor_pwm(pwm_A, MOT_A1_PIN, MOT_A2_PIN);
  set_motor_pwm(pwm_B, MOT_B1_PIN, MOT_B2_PIN);

  //Print a status message to the console.
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
void spin_and_wait(int pwm_A, int pwm_B, int duration) {
  set_motor_currents(pwm_A, pwm_B);
  delay(duration);
}

// IR sensor scanning - updates the irSensors global variable
void scanIR() {
  irSensors = B000;

  for (int i = 0; i < 3; i++) {
    int sensorValue = analogRead(irPins[i]);
    if (sensorValue >= threshold) {
      irSensorDigital[i] = 1;
      // Serial.print(1);
      // Serial.print(" ");
    }

    else {
      irSensorDigital[i] = 0;
      // Serial.print(0);
      // Serial.print(" ");
    }

    int b = 2 - i;
    irSensors = irSensors + (irSensorDigital[i] << b);
  }
}

// Ultrasonic sensor scanning - updates the distance global variable
void scanUltrasonic() {
  // Ultrasonic sernsor scanning
  distance = sonar.ping_cm();
  // Serial.print("distance:");
  // Serial.print(distance);
  // Serial.print(" ");
}

// PID Constants (Tune these values)
float Kp = 0.9;  // Proportional gain
float Ki = 0.0;  // Integral gain (usually small or 0)
float Kd = 0.2;  // Derivative gain

float integral = 0;
float derivative = 0;

void UpdateDirection() {
  
  errorLast = error;

  switch (irSensors) {
    case B000: // No sensor detects the line
       error = (errorLast < 0) ? -130 : 130;
       break;
    case B100: error = -100; break; // Leftmost sensor
    case B110: error = -50; break;
    case B010: error = 0; break; // Centered
    case B011: error = 50; break;
    case B001: error = 100; break; // Rightmost sensor
    case B111: error = 0; break; // Perfectly centered
    default:   error = errorLast;
  }

  // PID Calculation
  integral += error;                      // Accumulate integral (sum of past errors)
  derivative = error - errorLast;         // Rate of change of error
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Adjust Motor Speeds
  leftServoSpeed = maxSpeed + correction;
  rightServoSpeed = maxSpeed - correction;

  // Keep Speeds within Valid Range
  leftServoSpeed = constrain(leftServoSpeed + motorOffset, 0, maxSpeed + motorOffset);
  rightServoSpeed = constrain(rightServoSpeed - motorOffset, 0, maxSpeed - motorOffset);
}

// Updates the buggy state global variable
void updateState() {
    // Check if an object is within the stop threshold
    previousState = state; // Update previous state for next cycle
    bool objectDetected = (distance <= STOP_THRESHOLD && !(distance == 0)); // If the object is in the stop threshold and not 0 (aka above the sensing range) it returns 1
    
    switch (previousState) {
        case MOVING:
            if (objectDetected) {
                state = WARNING;
            }
            break;
        
        case WARNING:
            if (!objectDetected) {
              state = MOVING; // Return to moving if object is gone
            } else if (warningCount == 0) {
              state = SLAPPING;
              playSlapStart();
            }
            break;
        
        case SLAPPING:
            if (!objectDetected) {
                state = MOVING; // Return to moving once object is gone
                playSlapFinish();
                //delay(2000);
                playLineFollowing();
                Serial.println("I am line following");
                warningCount = WARNING_DURATION;
            }
            else if (objectDetected) {
              state = SLAPPING;
              playSlapStart();
            }
            break;
    }
    
}

// When attaching a servo to the Arduino, the PWM pins 9 and 10 don't work. So this will attach the servo seperately only when the object is detected.
void newServoAttach(int pin) {
  // turn the motors off
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);
  digitalWrite(MOT_B1_PIN, LOW);
  digitalWrite(MOT_B2_PIN, LOW);
  servo.attach(pin);
}

void newServoDetach() {
  servo.detach();
  // Reinitialize the stepper driver control pins to output drive mode.
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);
  pinMode(MOT_B1_PIN, OUTPUT);
  pinMode(MOT_B2_PIN, OUTPUT);

  // Start with drivers off, motors coasting.
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);
  digitalWrite(MOT_B1_PIN, LOW);
  digitalWrite(MOT_B2_PIN, LOW);

  // got this code from chatgpt but basically
  // the Servo.attach() function disables the PWM on pins 9 and 10 which controls the motors - so when
  // the servo is attached to the Arduino, the motors driving the buggy stop working
  // the code below reenables the PWM functionality of those pins.

  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(WGM12) | _BV(CS11);
}

void slap() {
  // Attach the servo to pin 3
  newServoAttach(3);

  // Set the Servo to 180 degrees
  servo.write(180-40);

  // Slap the object for 1 second
  delay(1000);

  // Bring the servo back to the starting position
  servo.write(0+40);

  // 1 second delay
  delay(1000);

  // detach the servo from the pin and re-enable the PWM on pins 9 and 10
  newServoDetach();

  Serial.println("Object slapped!");
}

void lineFollowing() {
  // keep the same motor speed
  // check to see if the state was changed to MOVING - if so, the line following sound will play
  Serial.print("left: ");
  Serial.print(leftServoSpeed);
  Serial.print(" right: ");
  Serial.print(rightServoSpeed);
  spin_and_wait(leftServoSpeed, rightServoSpeed, 10);
  if(previousState != state) {
    Serial.println("change!");
    //playLineFollowing();
  }
}

void warning() {
  // stop buggy
  if(previousState != WARNING) {
    //playLineFollowing();
  }

  if(warningCount == WARNING_DURATION) {
    playWarning();
    //waitForAudioToFinish();
  }

  if(warningCount % 50 == 0) {
    ledFlip(7);
    playBeep();
  }

  spin_and_wait(0, 0, 10);
  if(previousState != state) {
  Serial.println("change!");

  }
  // decrement the warningCount by 1
  warningCount -= 1;
}

void slapping() {
  // stop the buggy
  spin_and_wait(0, 0, 10);
  slap();
}

void ledOn(int pin) {
  digitalWrite(pin, HIGH);
}

void ledOff(int pin) {
  digitalWrite(pin, LOW);
}

void ledFlip(int pin) {
  if(digitalRead(pin)) {
    digitalWrite(pin, LOW);
  }
  else digitalWrite(pin, HIGH);
}

void loop() {
  // // get data from the IR and ultrasonic sensors
  scanIR();
  scanUltrasonic();

  // // update the direction of the buggy depending on the sensor's data
  UpdateDirection();

  // // update the buggy's state based on the sensor's data
  updateState();

  // //Serial.println(distance);


  // // do different functions based on the buggy's state machine
  switch (state) {
    case MOVING: {
      lineFollowing();
      Serial.print("FOllowing line");
      ledOn(7);
      break;
    }

    case WARNING: {
      warning();
      Serial.println(warningCount);
      Serial.print("Warning");
      break;
    }

    case SLAPPING: {
      // Slap the object!
      slap();
      Serial.println("slapping");
            ledOff(7);
      break;
    }
  }

  Serial.println(distance);


  dfmp3.loop();
}

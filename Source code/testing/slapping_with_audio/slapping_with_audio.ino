/*
A sketch which added audio functionality to the previously made slapping code.

This program:
- Detects if an obstacle is in slapping radius
- If an obstacle is detected in the slapping radius:
	- play a warning message
	- play a beep sound every second until the countdown goes down
	- play the slapping start sound
	- slap the object
	- play the slapping finished sound
	- continue line following sound
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

// Sound things

// For the sound system used in this project, the DFPlayer Mini is used.
// The following libraries are included in this project
#include <DFMiniMp3.h>
#include <SoftwareSerial.h>

class Mp3Notify; 
// Initialising the Serial connection between the board and the sound chip
SoftwareSerial secondarySerial(4,2); // RX, TX
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

void setup() {
  // Initialising the servo
  servo.attach(3);
  servo.write(0);

  // Initialising the DFPlayer Mini chip
  Serial.begin(115200);

  Serial.println("Initialising chip.");
  
  dfmp3.begin();


  uint16_t version = dfmp3.getSoftwareVersion();
  Serial.println("test");
  Serial.print("version ");
  Serial.println(version);

  uint16_t volume = dfmp3.getVolume();
  Serial.print("volume ");
  Serial.println(volume);
  dfmp3.setVolume(24);
  
  uint16_t count = dfmp3.getTotalTrackCount(DfMp3_PlaySource_Sd);
  Serial.print("files ");
  Serial.println(count);
  
  Serial.println("starting...");
  playAllTest();

  playInit();
}

int getDistance(){
  return sonar.ping_cm();
}

int countdown = 5;

int playInit() {
  delay(1000);
  playStartUp();
  delay(200);
  playGreeting();
  delay(200);
}

int playStartUp() {
  audioFinished = 0;
  Serial.println("play start up noise");
  dfmp3.playFolderTrack16(1, 1);
  waitForAudioToFinish();
  return 1;
}

int playGreeting() {
  audioFinished = 0;
  Serial.println("play greeting");
  dfmp3.playFolderTrack16(1,2);
  waitForAudioToFinish();
  return 1;
}

int playLineFollowing() {
  audioFinished = 0;
  Serial.println("play line following");
  dfmp3.playFolderTrack16(2,1);
  waitForAudioToFinish();
  return 1;
}

int playBeep() {
  audioFinished = 0;
  Serial.println("play beep");
  dfmp3.playFolderTrack16(3,1);
  waitForAudioToFinish();
  return 1;
}

int playWarning() {
  audioFinished = 0;
  Serial.println("play warning");
  dfmp3.playFolderTrack16(3,2);
  waitForAudioToFinish();
  return 1;
}

int playSlapStart() {
  audioFinished = 0;
  Serial.println("play slap start");
  dfmp3.playFolderTrack16(3,3);
  waitForAudioToFinish();
  return 1;
}

int playSlapFinish() {
  audioFinished = 0;
  Serial.println("play slap finish");
  dfmp3.playFolderTrack16(3,4);
  waitForAudioToFinish();
  return 1;
}

// Plays all the audio files just for testing reasons
int playAllTest() {
  Serial.println("reached");
  playStartUp();
  playGreeting();
  playLineFollowing();
  playBeep();
  playWarning();
  playSlapStart();
  playSlapFinish();
}

void dfmp3Loop() {
  dfmp3.loop();
  delay(50);
}

void waitForAudioToFinish() {
  while(!audioFinished) dfmp3Loop();
}

void slap() {
  servo.write(0);
  Serial.println((servo.read()));
  delay(1000);
  servo.write(180);
  Serial.println((servo.read()));
}

void loop() {
  delay(200);
  distance = getDistance(); // update the distance read by the ultrasonic sensor

  Serial.print(distance);
  Serial.println("cm");

  Serial.println(countdown);

  if(distance < STOP_THRESHOLD && (distance != 0)) { // If an obstacle is detected in the detection radius
    if(countdown == 5) { // Play the warning if the countdown is at 5 (countdown started)
      playWarning();
    }
    if(countdown > 0) { // Play a beep sound while the countdown goes down
      countdown -= 1; 
      playBeep();
    }
    if(countdown == 0) {
      playSlapStart();
      delay(100);
      slap();
      playSlapFinish();
      countdown = 5;
      playLineFollowing();
      delay(2000);
    }
    delay(500);
  dfmp3.loop();
  delay(50);
  }
}


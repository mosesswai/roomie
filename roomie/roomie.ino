// Room Monitor and Notification System
// Uses Feather Huzzah ESP8266 WiFi board and Music Maker FeatherWing
// along with Adafruit IO and IFTTT
// by Moses Swai

// mode
#define DEBUGGING

#include "config.h"

// include SPI, MP3 and SD libraries
#include <SPI.h>
#include <SD.h>
#include <Adafruit_VS1053.h>
// include IR libraries
#include <IRrecv.h>
#include <IRutils.h>
// include sensor libraries
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
// include servo library
#include <Servo.h>

/***** IR remote definitions *****/
int RECV_PIN = 4; //an IR detector/demodulatord is connected to GPIO pin 2
IRrecv irrecv(RECV_PIN);

//Define Buttons
#define VOLUME_UP     0xfd40bf
#define VOLUME_DOWN   0xfd00ff
#define PLAY_PAUSE    0xFD807F
#define RIGHT_ARROW   0xfd50af
#define LEFT_ARROW    0xfd10ef
#define SELECT_BUTTON 0xfd906f
#define UP_ARROW      0xfda05f
#define DOWN_ARROW    0xfdb04f
#define BUTTON_0      0xfd30cf
#define BUTTON_1      0xfd08f7
#define BUTTON_2      0xfd8877
#define BUTTON_3      0xfd48b7
#define BUTTON_4      0xfd28d7
#define BUTTON_5      0xfda857
#define BUTTON_6      0xfd6897
#define BUTTON_7      0xfd18e7
#define BUTTON_8      0xfd9867
#define BUTTON_9      0xfd58a7

/***** Music Maker definitions *****/
// These are the pins used ofr Feather Huzzah ESP8266
#define VS1053_RESET   -1     // VS1053 reset pin (not used!)
#define VS1053_CS      16     // VS1053 chip select pin (output)
#define VS1053_DCS     15     // VS1053 Data/command select pin (output)
#define CARDCS          2     // Card chip select pin
#define VS1053_DREQ     0     // VS1053 Data request, ideally an Interrupt pin

Adafruit_VS1053_FilePlayer musicPlayer = 
  Adafruit_VS1053_FilePlayer(VS1053_RESET, VS1053_CS, VS1053_DCS, VS1053_DREQ, CARDCS);

/***** Define IO and controls pins *****/
#define SERVO_PIN     5
#define LED_PIN       1
#define DOOR_PIN      3
#define MOTION_PIN    A0
#define TEMP_PIN      13

// create DHT22 instance
DHT_Unified dht(TEMP_PIN, DHT22);

// create an instance of the servo class
Servo servo;

#define ON_SERVO_ANGLE        45
#define OFF_SERVO_ANGLE       180
#define NEUTRAL_SERVO_ANGLE   100

/***** Define Feeds *****/
// set up the 'notification' feed
AdafruitIO_Feed *notification = io.feed("notification");
// set up the 'alert' feed
AdafruitIO_Feed *alert = io.feed("room.alert");
// set up the 'temperature' feed
AdafruitIO_Feed *state_feed = io.feed("room.state");

// the name of what we're going to play
char foundname[20];
boolean isPaused = false;
uint8_t volume = 10;
int lastRemoteVal = 0;

/***** State Control *****/
// Main States
typedef enum {
  ARMED, DISARMED, WAITING, DISTURBED, RESTORING, BREACHED
}RoomState;

// Room State
RoomState state;
bool listening = false;
int alarmPending = 0; 
int job = 0;

// code
int code[] = {6, 7, 8};
int input[3];
int pos = 0;
int trials = 0;
#define MAX_TRIALS        10
#define MAX_ARMED_TRIALS  5

// door
int doorState;

// motion
int pirState; 

// temperature
float fahrenheit;
float celsius;

// timers
unsigned long timer_start;
unsigned long previous_time;
unsigned long previous_motion_time;
unsigned long prev_listening_time;
unsigned long ledTimer;
unsigned long previous_job_time;

// timer control times
#define MOTION_DETECT_TIME      10000
#define DISTURBED_TIME          20000
#define DEPART_TIME             40000
#define RESTORING_TIME          20000
#define ARMING_TIME             120000        // Time to leave the room after arming
#define ALARM_SETOFF_TIME       120000
#define LISTENING_TIME          120000        // Time to stay in listening mode

#define SNOOZE_TIME             30000         // Time to wait before checking if I am awake

// jobs
#define MORNING           1

// alarm numbers
#define LOCK_TOGGLE       2
#define WRONG_CODE        3
#define LEAVE_ROOM        4
#define BREACHED_ALARM    5
#define MORNING_ALARM     6
#define UBER_ARRIVING     7


void setup() {
  Serial.begin(115200);

  // Set pin modes
  #ifndef DEBUGGING 
    pinMode(LED_PIN, INPUT_PULLUP);
  #endif
  pinMode(DOOR_PIN, INPUT_PULLUP);
  pinMode(MOTION_PIN, INPUT);
  

  Serial.println("\n\nWelcome to Moses' Room Monitor System!");
  
  if (! musicPlayer.begin()) { // initialise the music player
     Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
     while (1);
  }

  Serial.println(F("VS1053 found"));
 
//  musicPlayer.sineTest(0x44, 500);    // Make a tone to indicate VS1053 is working
  
  if (!SD.begin(CARDCS)) {
    Serial.println(F("SD failed, or not present"));
    while (1);  // don't do anything more
  }
  Serial.println("SD OK!");

  
  // list files
  printDirectory(SD.open("/"), 0);

  // If DREQ is on an interrupt pin we can do background audio playing
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ int

  // connect to io.adafruit.com
  Serial.print("Connecting to Adafruit IO");
  io.connect();

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());

  // initialize dht22
//  dht.begin();

  // Start the receiver
  irrecv.enableIRIn();

  // tell the servo class which pin we are using
  servo.attach(SERVO_PIN);

  // create message handlers
  notification->onMessage(handleNotification);
//  state_feed->onMessage(handleState);

  // resets in disarmed state
  state = DISARMED;

  musicPlayer.sineTest(0x74, 500);    // Make a tone to indicate successful setup

  // Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(volume,volume);

  // light LED
  ledOn();
}

void loop() {
  // Adafruit IO
  yield();
  io.run();

  // from the IR remote
  decode_results results;
  
  monitorDoor();
  monitorMotion();

  monitorListening();
  monitorWaiting();
  monitorRestoring();

  // monitors remaining events
  monitorJobs();

  // checks whether the system has a pending alarm to play
  monitorAlarm();

 if(digitalRead(VS1053_DREQ) && !musicPlayer.stopped() && !isPaused) {
    musicPlayer.feedBuffer();
  } 

  // look for a message!
  if (irrecv.decode(&results)) {
//    Serial.println(results.value);
    irrecv.resume(); // Receive the next value

    // handle repeat codes!
    if (results.value == 0xFFFFFFFF) {
      // only for vol+ or vol-
      if ( (lastRemoteVal == 0xFD40BF) || (lastRemoteVal == 0xFD00FF))
         results.value = lastRemoteVal;
    } else {
      lastRemoteVal = results.value;
    }
    
    // entering code
    if (results.value == BUTTON_1) {
       if (listening) {
        enterCode(1);
      }
    }
    if (results.value == BUTTON_2) {
      if (listening) {
        enterCode(2);
      }
    }
    if (results.value == BUTTON_3) {
      if (listening) {
        enterCode(3);
      } else {
        lightsOn();
      }
    }
    if (results.value == BUTTON_4) {
      if (listening) {
        enterCode(4);
      } else {
        lightsOff();
      }
    }
    if (results.value == BUTTON_5) {
      if (listening) {
        enterCode(5);
      }
    }
    if (results.value == BUTTON_6) {
      if (listening) {
        enterCode(6);
      } else {
        job = MORNING;
        previous_job_time = millis();
      }
    }
    if (results.value == BUTTON_7) {
      if (listening) {
        enterCode(7);
      } else {
        alarmPending = UBER_ARRIVING;
      }
    }
    if (results.value == BUTTON_8) {
      if (listening) {
        enterCode(8);
      } else {
        alarmPending = LEAVE_ROOM;
      }
    }
    if (results.value == BUTTON_9) {
      if (listening) {
        enterCode(9);
      } else {
        alarmPending = BREACHED_ALARM;
      }
    }
    // toggle listening mode
    if (results.value == BUTTON_0) {
      musicPlayer.startPlayingFile("track003.mp3");
      listening = true;
      prev_listening_time = millis();
      Serial.println("Listening On");
    }
//    if (results.value == SELECT_BUTTON) {
//       Serial.println("playing track #selevtt");
//    }
//
    if (results.value == VOLUME_UP) { //vol+
      Serial.println("Vol+");
      if (volume > 0) {
         volume--;
         musicPlayer.setVolume(volume,volume);
      }
    }
    if (results.value == VOLUME_DOWN) { //vol-
      Serial.println("Vol-");
      if (volume < 100) {
         volume++;
         musicPlayer.setVolume(volume,volume);
      }
    }
//
//    if (results.value == PLAY_PAUSE) { // playpause
//      Serial.println("Play/Pause");
//      isPaused = !isPaused; // toggle!
//    } 
  }
  
  yield();
  delay(1);
}


boolean findFileStartingWith(char *start) {
  File root;
  root = SD.open("/");
  root.rewindDirectory();
  while (true) {
    File entry =  root.openNextFile();
    if (! entry) {
      return false;
    }
    String filename = entry.name();
    Serial.print(filename);
    if (entry.isDirectory()) {
      Serial.println("/");
    } else {
      Serial.println();
      if (filename.startsWith(start)) {
        filename.toCharArray(foundname, 20); 
        entry.close();
        root.close();
        return true;
      }
    }
    entry.close();
  }
}


/// File listing helper
void printDirectory(File dir, int numTabs) {
   while(true) {
     
     File entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       //Serial.println("**nomorefiles**");
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');
     }
     Serial.print(entry.name());
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       // files have sizes, directories do not
       Serial.print("\t\t");
       Serial.println(entry.size(), DEC);
     }
     entry.close();
   }
}

// CODE
void enterCode(int num) {
  // play beep sound
  musicPlayer.startPlayingFile("track003.mp3");     
  input[pos++] = num;
  Serial.print("Entered Code: "); 
  Serial.print(input[0]); Serial.print(input[1]); Serial.println(input[2]);

  
  // verify code
  if(pos == 3) {
    bool correct_code = checkInput();
    resetInput();
    
    // when disarmed
    if (state == DISARMED) {
      if (correct_code) {
        Serial.println("Correct code! WAITING - leave the room in 2 minutes!");
        state_feed->save("WAITING");
        // play correct code tone
        alarmPending = LOCK_TOGGLE;
        // wait to leave the room
        trials = 0;
        listening = false;
        previous_time = millis();
        state = WAITING;
      } else {
        if(++trials == MAX_TRIALS) alarmPending = WRONG_CODE;
        Serial.print("Incorrect code! Trial: "); Serial.println(trials);
        alarmPending = WRONG_CODE;
      }
    }

    // when armed
    else if (state == DISTURBED || state == ARMED) {
      if (correct_code) {
        Serial.println("Correct code! DISARMED");
        state_feed->save("DISARMED");
        // play correct code tone
        alarmPending = LOCK_TOGGLE;
        // disarm the room
        trials = 0;
        listening = false;
        state = DISARMED;
      } else {
        if(++trials == MAX_ARMED_TRIALS) alarmPending = WRONG_CODE;
        Serial.print("Incorrect code! Trial: "); Serial.println(trials);
        alarmPending = WRONG_CODE;
      }
    }
  }
}

bool checkInput() {
  if(input[0] != code[0]) return false;
  if(input[1] != code[1]) return false;
  if(input[2] != code[2]) return false;
  return true;
}

void resetInput() {
  pos = 0;
  input[0] = input[1] = input[2] = 0;
}


// ALARM
void monitorAlarm() {
  switch(alarmPending) {
    case 0: break;
    default: ringAlarm(); break;
  }
}

void ringAlarm () {
  musicPlayer.stopPlaying();
  switch(alarmPending) {
    case LOCK_TOGGLE:   musicPlayer.startPlayingFile("track004.mp3"); break;
    case WRONG_CODE:    musicPlayer.startPlayingFile("track005.mp3"); break;
    case MORNING_ALARM: musicPlayer.startPlayingFile("track006.mp3"); break;
    case UBER_ARRIVING: musicPlayer.startPlayingFile("track007.mp3"); break;
    case BREACHED_ALARM:musicPlayer.startPlayingFile("track009.mp3"); break;
    case LEAVE_ROOM:    musicPlayer.startPlayingFile("track008.mp3"); break;
    default: break;
  }
  alarmPending = 0;
}


// LISTENING
void monitorListening() {
  if (listening) {
    if (millis() - prev_listening_time >= LISTENING_TIME) {
      Serial.println("Listening Off");
      listening = false;
      resetInput();
    }
  }
}


// WAITING
void monitorWaiting() {
  if (state == WAITING) {
    if (millis() - previous_time >= ARMING_TIME) {
      Serial.println("ARMED!");
      state_feed->save("ARMED");
      state = ARMED;
    }
  }
}


// RESTORING
void monitorRestoring() {
  if (state == DISTURBED) {
    if (millis() - timer_start >= ALARM_SETOFF_TIME) {
      Serial.println("SECURITY COMPROMISED!!");   
      alert->save("Breached");
      state = BREACHED;
      alarmPending = BREACHED_ALARM;
    }
  } else if (state == RESTORING) {
    if (motionDetected() || doorIsOpen()) {
      Serial.println("Room disturbed again");
      state = DISTURBED;
      previous_time = previous_motion_time = millis();
    }
    if (millis() - previous_time >= RESTORING_TIME) {
      Serial.println("Room ARMED again");
      state = ARMED;
    }
  } else if (state == BREACHED) {
    
  }
}


// DOOR
void monitorDoor() {
  // Changes state to disturbed if door is opened
  if (state == ARMED) {
    if (doorIsOpen()) {
      Serial.println("Door Opened, state changed to DISTURBED");
      alert->save("Door Opened");
      state = DISTURBED;
      timer_start = previous_time = previous_motion_time = millis();
    } 
  }
  // Tries to restore the room if door is closed and no more motion is detected
  if (state == DISTURBED) {
    if(!doorIsOpen() && millis()-previous_motion_time >= DEPART_TIME) {
      Serial.println("Door closed, room restoring...");
      state = RESTORING;
      previous_time = millis();
    }
  }
}

bool doorIsOpen() {
  return digitalRead(DOOR_PIN);
}


// MOTION
void monitorMotion() {
  if (state == DISTURBED) {
    unsigned long current_time = millis();
    
    if (motionDetected()) {            
      if (current_time-previous_motion_time >= MOTION_DETECT_TIME) {
        Serial.println("Motion Detected! Alert to leave room");
        previous_time = previous_motion_time = current_time;
        //alarm here
        alarmPending = LEAVE_ROOM;
      }
    } else {
      if (current_time-previous_time >= DISTURBED_TIME){
        Serial.println("Alert to leave room");
        previous_time = current_time;
        //alarm here
        alarmPending = LEAVE_ROOM;
      }
    }
  } 
}

bool motionDetected() {
  return analogRead(MOTION_PIN) >= 1000;
}


// LED
void monitorLED() {
  
}


// LED Toggle funcitons
void ledOn() {
  digitalWrite(LED_PIN, LOW);
}

void ledOff() {
  digitalWrite(LED_PIN, HIGH);
}


// Room lights
void lightsOn() {
  Serial.println("Lights ON");
  servo.write(ON_SERVO_ANGLE);
  delay(500);
  servo.write(NEUTRAL_SERVO_ANGLE);
}

void lightsOff() {
  Serial.println("Lights OFF");
  servo.write(OFF_SERVO_ANGLE);
  delay(500);
  servo.write(NEUTRAL_SERVO_ANGLE);
}


// Monitor alarm and Uber
void monitorJobs () {
  unsigned long current_time = millis();  
  switch (job) {
    case MORNING:
      if(motionDetected() || doorIsOpen()) job = 0;
      if (current_time - previous_job_time > SNOOZE_TIME) {
        alarmPending = MORNING_ALARM;
        previous_job_time = current_time;
      }
      break;
    default: break;
  }
}


// IO handlers
void handleNotification( AdafruitIO_Data *data) {
  // convert dat to String
  String st = data->toString();

  if (st == "lights off") {
    lightsOff();
    
  } else if (st == "lights on") {
    lightsOn();
    
  } else if (st == "leaving room") {
    // play correct code tone
    alarmPending = LOCK_TOGGLE;
    // wait to leave the room
    previous_time = millis();
    state = WAITING;
    // turn off lights
    lightsOff();

  } else if (st == "arm room") {
    Serial.println("Armed Remotely");
    // play correct code tone
    alarmPending = LOCK_TOGGLE;
    // arm room
    state = ARMED;
    

  } else if (st == "disarm room") {
    Serial.println("Disarmed Remotely");
    // play correct code tone
    alarmPending = LOCK_TOGGLE;
    // disarm the room
    state = DISARMED;
    
  } else if (st == "returning room") {
    // play correct code tone
    alarmPending = LOCK_TOGGLE;
    // disarm the room
    state = DISARMED;
    // turn on lights
    lightsOn();
    
  } else if (st == "uber arriving") {
    Serial.println("Uber Notification");
    alarmPending = UBER_ARRIVING;
    
  } else if (st == "morning") {
    Serial.println("Wake up time");
    job = MORNING;
    previous_job_time = millis();
  }
}

//void handleState( AdafruitIO_Data *data) {

//  // convert dat to String
//  String st = data->toString();
//  
//  if (st == "DISARMED") {
//    Serial.println("Disarmed Remotely");
//    state = DISARMED;
//  } else if (st == "ARMED") {
//    Serial.println("Armed Remotely");
//    state = ARMED;
//  } else if (st == "ON") {
//    lightsOn();
//  } else if (st == "OFF") {
//    lightsOff();
//  }
//}



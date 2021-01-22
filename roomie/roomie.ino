/* Room monitor adapted from Adabox 003 using the Feather
 *  Huzzah ESP8266
 *  by Moses.
 */

#include "config.h"

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_SSD1306.h>

#define BATTERY_PIN 12
#define DOOR_PIN 16
#define LIGHT_PIN A0
#define MOTION_PIN 15

// pin connected to DH22 data line
#define DATA_PIN 2

/***** misc *****/
// oled display
Adafruit_SSD1306 oled = Adafruit_SSD1306();

// create DHT22 instance
DHT_Unified dht(DATA_PIN, DHT22);

/***** state values *****/
int battery_state;
int battery_state_init = 0;

// door
int door_state;
int door_last = 1;

// humidity
float relative_humidity;

// light
int light_current = 0;
int light_last = -1;

// motion
int pirState = LOW; 

// temperature
float fahrenheit;
float celsius;


/***** set up feeds *****/
// battery
AdafruitIO_Feed *battery = io.feed("battery");
// light
AdafruitIO_Feed *door = io.feed("door");
// light
AdafruitIO_Feed *light = io.feed("light");
// temperature
AdafruitIO_Feed *temperature = io.feed("temperature");
// humidity
AdafruitIO_Feed *humidity = io.feed("humidity");


/***** setup function *****/
void setup() {
  // initialize with the I2C addr 0x3C (for the 128x32)
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
  oled.display();

  pinMode(BATTERY_PIN, INPUT);
  
  // start the serial connection
  Serial.begin(115200);

  // wait for serial monitor to open
  while(! Serial);

  // initialize dht22
  dht.begin();

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

  // text display tests
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  
}

/***** loop function *****/
void loop() {

  // keeps the client connected to io.adafruit.com, and processes any incoming data.
  io.run();

  //update feeds
  monitorBattery();
  
  monitorDoor();
  
  monitorLight();

  sensors_event_t event;
  monitorTemperature(event);
  monitorHumidity(event);
  
  // print to OLED
  printOLED();

  // wait one second (1000 milliseconds == 1 second)
  delay(3000);
  
}

//BATTERY
void monitorBattery(){
  battery_state = digitalRead(BATTERY_PIN);
  // only change if battery is low
  if(battery_state != battery_state_init){
    battery->save(battery_state);
    battery_state_init = battery_state;
  }
}

// DOOR
void monitorDoor() {
  door_state = digitalRead(DOOR_PIN);
  // only report if door state changes
  if (door_state != door_last) {
    door->save(door_state);
    door_last = door_state;
  } 
  
}

// LIGHT INTENSITY
void monitorLight() {
  // grab the current state of the photocell
  light_current = analogRead(LIGHT_PIN);
  
  light_current = map(light_current, 0, 1024, 0, 100);

  // return if the value hasn't changed
  if(light_current == light_last)
    return;

  // save the current state to the analog feed
//  Serial.print("sending light -> ");
//  Serial.println(light_current);
  light->save(light_current);

  // store last photocell state
  light_last = light_current;

}

// TEMPERATURE
void monitorTemperature(sensors_event_t &event){
  dht.temperature().getEvent(&event);

  celsius = event.temperature;
  fahrenheit = (celsius * 1.8) + 32;

//  Serial.print("celsius: ");
//  Serial.print(celsius);
//  Serial.println("C");

//  Serial.print("fahrenheit: ");
//  Serial.print(fahrenheit);
//  Serial.println("F");

  // save fahrenheit (or celsius) to Adafruit IO
  temperature->save(fahrenheit);

}

// HUMIDITY
void monitorHumidity(sensors_event_t &event) {
  dht.humidity().getEvent(&event);
  relative_humidity = event.relative_humidity;
  
//  Serial.print("humidity: ");
//  Serial.print(relative_humidity);
//  Serial.println("%");

  // save humidity to Adafruit IO
  humidity->save(relative_humidity);

}

// MOTION
void monitorMotion() {
//  int val = digitalRead(MOTION_PIN);  // read input value
//  if (val == HIGH) {            // check if the input is HIGH
//    if (pirState == LOW) {
//      // we have just turned on
//      Serial.println("Motion detected!");
//      // We only want to print on the output change, not state
//      pirState = HIGH;
//    }
//  } else {
//    if (pirState == HIGH){
//      // we have just turned of
//      Serial.println("Motion ended!");
//      // We only want to print on the output change, not state
//      pirState = LOW;
//    }
//  }
}

void printOLED() {
  oled.clearDisplay();
  
  oled.setCursor(0,0);
  oled.print("Battery: ");
  if(battery_state){
    oled.println("HIGH");
  } else {
    oled.println("LOW");
  }
  
  oled.print("Temp: "); oled.print(fahrenheit,0); oled.print(" *F / ");
  oled.print(celsius,0); oled.println(" *C ");
  
  oled.print("Hum: "); oled.print(relative_humidity,0); oled.print(" % ");
  
  oled.print("Light: "); oled.println(light_current);
  
  oled.print("Door: ");
  if(door_state){
    oled.println("CLOSED");
  } else {
    oled.println("OPEN");
  }
  
  oled.display();
}


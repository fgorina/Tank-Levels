#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266mDNS.h>
#include <ArduinoWebsockets.h>
#include <ESP_EEPROM.h>
#include <Wire.h>
#include <VL53L1X.h>

#define DEBUG false
#define DEBUG_1 true

#define EEPROM_SIZE 512

#define TRIGGER_GPIO 16
#define END_GPIO 14

#define ONBOARD_LED 2


// WiFi and SignalK Connections

#define metaUpdate "{\"updates\": [{\"meta\":[{\"path\":\"steering.rudderAngle\", \"value\": {\"units\": \"m\"}}]}]}"

#define update1 "{ \"context\": \""
#define update2 "\", \"updates\": [ {  \"source\": {\"label\": \"rudderAngleSensor\" }, \"values\": [ { \"path\": \"steering.rudderAngle\",\"value\":"
#define update4 " } ] } ]}"
#define update5 "\", \"updates\": [ {  \"source\": {\"label\": \"Tank 1 Level Sensor\" }, \"values\": [ { \"path\": \"tanks.freshWater.1.currentLevel\",\"value\":"
#define update6 "}, { \"path\": \"tanks.freshWater.1.currentVolume\",\"value\":"

char ssid[20] = "Yamato";
char password[20] = "ailataN1991";
char device_name[20] = "tank_1";
char skserver[20] = "";
int skport = 0; // It is 4 bytes
char skpath[100] = "/signalk/v1/stream?subscribe=none";



// Tank definition. Sizes in m


double height = 0.45; // 80 Gal
double width = 0.29;  // 80 Gal
double length = 0.24; // 80 Gal
const double sound_speed = 340.0;
float level = 0.5; // %0% Full

double max_height_time = height * 2.0 / sound_speed * 1e6;

// Kalman filter for tank level
// All units are fractions

// x_predicted == 0 -> inici
// First measure is not processed
// but initializes x_predicted, x_filtered

double x_predicted = 0.0;
double p_predicted = 1.0;
double K = 0.0;
double x_filtered = 0.0;
double p_computed = 1.0;
double r_measure = 0.001; 
double r_process = 0.00005;


unsigned int counter = 0;

// mDNS & cia

using namespace websockets;

bool mdnsDone = false; // Will be true when we have a server

bool wifi_connect = false;
int enabled = 0; // 0 Deshabilita les accions fins que s'ha rebut un command
WebsocketsClient client;
int socketState = -4; // -5 does not use WiFi, -4 -> Before connecting to WiFi, -3, -2.Connectingau


String me = "vessels.self";
char token[256] = "";
char bigBuffer[1024] = "";

// Laser Sensor

VL53L1X sensor;

bool useLaser = true;   // If true uses LASER else ULTRASOUND

int ledState = 0;
int ledOn = 0;
int ledOff = 100;


// LEDs

void clearLed()
{
  ledState = 0;
  digitalWrite(ONBOARD_LED, ledState);
}

void setLed()
{
  ledState = 1;

  digitalWrite(ONBOARD_LED, ledState);
}

void toggleLed()
{
  if (ledState == 0)
  {
    ledState = 1;
  }
  else
  {
    ledState = 0;
  }

  digitalWrite(ONBOARD_LED, ledState);
}

#include "signalk.h"
// Functions

void sendTrigger()
{

  digitalWrite(TRIGGER_GPIO, 0);
  delay(2);
  digitalWrite(TRIGGER_GPIO, 1);
  delay(10);
  digitalWrite(TRIGGER_GPIO, 0);
}

void kalman_step(double value)
{
  x_predicted = x_filtered;
  p_predicted = p_computed ;
  K = p_predicted / (p_predicted + r_measure);
  x_filtered = x_predicted + K * (value - x_predicted);
  p_computed = (1.0 - K) * p_predicted;
}

 // Load Data from EEPROM
void loadEEPROM(){

  float f1;
  float f2;
  char s20[20];

  EEPROM.begin(EEPROM_SIZE);

  EEPROM.get(0, f1);
  EEPROM.get(4, f2);

  if (isnan(f1) || isnan(f2) || f1 == 0.0 || f2 == 0.0 )
  {

    f1 = 1.0;
    f2 = 1.0;
    // Init EEPROM Area
    EEPROM.put(0, f1);
    EEPROM.put(4, f2);
    EEPROM.put(8, ssid);
    EEPROM.put(28, password);
    EEPROM.put(48, device_name);
    EEPROM.put(68, skserver);
    EEPROM.put(88, skport);
    EEPROM.put(92, skpath);
    EEPROM.put(192, token);

    EEPROM.commit();
    if (DEBUG || true)
    {
      Serial.println();
    }
    if (DEBUG || true)
    {
      Serial.println("Written default data to EEPROM");
    }
  }
  else
  {

    if (DEBUG || true)
    {
      Serial.println("EEPROM already vith values");
    }
  }

  EEPROM.get(8, ssid);
  EEPROM.get(28, password);
  EEPROM.get(48, s20);

  if (strlen(s20) != 0)
  {
    strcpy(device_name, s20);
  }

  EEPROM.get(68, skserver);
  EEPROM.get(88, skport);
  EEPROM.get(92, skpath);
  EEPROM.get(192, token);

  if (strlen(skserver) > 0)
  {
    Serial.println("Alreaddy have a server, no need to lookup by mDns");
    mdnsDone = true;
  }
}

void setup() {

 Serial.begin(115200);

 if(useLaser){
  Wire.begin();
  Wire.setClock(400000);
  sensor.setBus(&Wire);
  sensor.setTimeout(500);
 
  if (!sensor.init()) {
      Serial.println("Failed to detect and initialize sensor!");
      while (1){
        delay(1000);
      }
            
  }

  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(50);

 }else{
  pinMode(END_GPIO, INPUT);
  pinMode(TRIGGER_GPIO, OUTPUT);

 }

 loadEEPROM();

}

void loop() {


  double distance = 0.0;
  
  networkTask();

  if (useLaser){
    uint16 ilaser = sensor.read();    // Measure in mm
    if(sensor.timeoutOccurred()){
      distance = -1.0;
    }else{
      distance = double(ilaser) / 1000.0; // Compute distance in m
    }
  }else{
    sendTrigger();
    long delta = pulseIn(END_GPIO, HIGH);  // Microseconds
    if (delta <= max_height_time){
      distance = delta * 1e-6 * sound_speed / 2.0;
    }else{
      distance = -1.0;
    }
  }
 


  if (distance > 0.04 && distance < height)  // Could be 0.04 0r 0.2 depending if Laser of Ultrasound
  {
    double h = height - distance;
    double t_level = h / height;

    if (x_predicted == 0)
    {
      x_predicted = t_level;
      x_filtered = t_level;
    }
    else
    {
      kalman_step(t_level);
    }

    counter++;


    if (DEBUG_1)
    {

      if(useLaser){
        Serial.print("Laser d: ");
      }else{
        
        Serial.print("Ultrasound d: ");
      
      }
      Serial.print(distance);
      Serial.print(" Level: ");
      Serial.print(t_level);
      Serial.print("% Filtered: ");
      Serial.print(x_filtered);
      Serial.print("% P: ");
      Serial.println(p_computed*10000.0);
    }   
    if(counter >= 10){
      counter = 0;
       float volume = height * width * length * x_filtered;
      sendData(x_filtered, volume);
    }
  }
  delay(1);
}

/***************************************************************************
  Authors: /madonius (@madonius)
           GityUpNow
  Organisation: Entropia e.V. ~ CCC Karlsruhe
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <RCSwitch.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>

#include "WiFi_credentials.hpp"
#include "MQTT_credentials.hpp"
#include "OTA_credentials.hpp"

#ifdef OTA_CREDENTIALS_HPP
  #include <ArduinoOTA.h>
#endif

//Global Constants
#define MEASURE 15000
#define PULSE 50

#define DOWN_PIN 13
#define STOP_PIN 12
#define   UP_PIN 14
#define MISC_PIN 16

//Global Variables
unsigned long last = 0;
float bmeData[3];
boolean bmeFound = true;

//Object declarations
Adafruit_BME280 bme;
RCSwitch transceiver =        RCSwitch();
WiFiClient client;
Adafruit_MQTT_Client          mqtt(&client, MQTT_SERVER, MQTT_SERVER_PORT, MQTT_USER, MQTT_PASSWORD);
Adafruit_MQTT_Subscribe   control(&mqtt,  MQTT_CONTROL_TOPIC);
Adafruit_MQTT_Publish       response(&mqtt, MQTT_RESPONSE_TOPIC);
Adafruit_MQTT_Subscribe   common(&mqtt, MQTT_COMMON_CONTROL_TOPIC);
Adafruit_MQTT_Publish       commonResponse(&mqtt, MQTT_COMMON_RESPONSE_TOPIC);

//Functions prototypes
void pulse(byte pin);
void setupWiFi();
void OTA_init();
void readBME(float *bmeData);
void mqtt_connect();
void mqtt_publish(float *bmeData);
void controlCallback(char *str, uint16_t len);
void commonCallback(char *str, uint16_t len);

void setup() {
  Serial.begin(115200);
  setupWiFi();

  if(!bme.begin(0x76)) {
    Serial.println(F("Could not find a BME280 Sensor!"));
    bmeFound = false;
  }

  delay(400);

  //Subscribe to the Leinwand-Controlfeed
  control.setCallback(controlCallback);
  common.setCallback(commonCallback);
  mqtt.subscribe(&control);
  mqtt.subscribe(&common);

  //Pin Setups
  pinMode(DOWN_PIN, OUTPUT);
  pinMode(STOP_PIN, OUTPUT);
  pinMode(UP_PIN, OUTPUT);

  //transceiver setup
  transceiver.enableTransmit(MISC_PIN);

  //OTA (if enabled)
  OTA_init();
}


void loop() {
    mqtt_connect();

    if((millis() - last) >= MEASURE && bmeFound) {
      readBME(bmeData);
      mqtt_publish(bmeData);
      last = millis();
    }

    mqtt.processPackets(10);

    #ifdef OTA_CREDENTIALS_HPP
      ArduinoOTA.handle();
    #endif
}


//Function definitions
void setupWiFi() {
  WiFi.mode(WIFI_STA);

  while ( WiFi.status() != WL_CONNECTED) {
    Serial.println("Attempting to connect network, SSID: ");
    Serial.println(SSID);
    WiFi.begin(SSID, WIFIPW);

    unsigned long start = millis();
    while(millis() - start < 20000 && WiFi.status() != WL_CONNECTED){
        delay(100);
    }
  }

  Serial.println("\nWifi Conected! \\o/ \n IP Address:");
  Serial.println(WiFi.localIP());
}


void readBME(float* bmeData) {
  float T = bme.readTemperature();
  delay(50);

  float P = bme.readPressure();
  delay(50);

  float H = bme.readHumidity();
  delay(50);

  bmeData[0] = T;
  bmeData[1] = P;
  bmeData[2] = H;
}

void mqtt_connect() {
  // Stop if already connected.
  if(mqtt.connected()){
    return;
  }

  int8_t ret;
  uint8_t retries = 3;
  while((ret = mqtt.connect()) != 0){ // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println(F("Retrying MQTT connection in 5 seconds..."));

       mqtt.disconnect();
       delay(5000);  // wait 5 seconds

       retries--;
       if(retries == 0){
         while (1); // basically die and wait for WDT to reset me
       }
  }
  Serial.println(F("MQTT Connected!"));
}

//Publish the BME-Data
void mqtt_publish(float *bmeData) {
  Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, MQTT_TOPIC "/T");
  Serial.println(bmeData[0]);
  temperature.publish(bmeData[0]);

  Adafruit_MQTT_Publish pressure = Adafruit_MQTT_Publish(&mqtt, MQTT_TOPIC "/P");
  Serial.println(bmeData[1]);
  pressure.publish(bmeData[1]);

  Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, MQTT_TOPIC "/H");
  Serial.println(bmeData[2]);
  humidity.publish(bmeData[2]);

  Serial.println("Published!");
  Serial.println();
}

//Handle the leinwand
void controlCallback(char *str, uint16_t len) {
  String command = String(str);

  //Simple "leinwand" functions
  if (command.length() == 1) {
    switch (command.charAt(0)) {
      case 'u':
        pulse(UP_PIN);
      break;
      case 's':
        pulse(STOP_PIN);
      break;
      case 'd':
        pulse(DOWN_PIN);
      break;
    }
  }

  response.publish(command.c_str());
  Serial.println("OK-Response published!");
}

//Handle the 433MHz-Sockets
void commonCallback(char* str, uint16_t len) {
    String command = String(str);

    //REV Ritter
    if (command.length() == 5 || command.length() == 6) {
      char temp = command.charAt(0);

      Serial.println(command.charAt(1) - '0');

      //Rev Ritter format...
      if (temp == 'A' || temp == 'B' || temp == 'C' || temp == 'D') {

        if (command.substring(3).equals("ON")) {
          Serial.println("ON");
          transceiver.switchOn(temp,  command.charAt(1) - '0');
        } else if (command.substring(3).equals("OFF")) {
          transceiver.switchOff(temp, command.charAt(1) - '0');
        }

      }
    //Brennenstuhl
    } else if(command.length() == 13 || command.length() == 14) {
      if (command.endsWith("ON")) {
        transceiver.switchOn(command.substring(0, 5).c_str(), command.substring(5, 10).c_str());
      } else if(command.endsWith("OFF")) {
        transceiver.switchOff(command.substring(0, 5).c_str(), command.substring(5, 10).c_str());
      }
    }

    commonResponse.publish(command.c_str());
    Serial.println("OK-Response published!");
}

//Send a short pulse
void pulse(byte pin){
  digitalWrite(pin, HIGH);
  delay(PULSE);
  digitalWrite(pin, LOW);
}

//Init the OTA-Update
void OTA_init(){
  #ifdef OTA_CREDENTIALS_HPP
    //OTA-Part
    ArduinoOTA.setHostname(OTA_USERNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);

    //OTA-Updatepart
    ArduinoOTA.onStart([]() {
       Serial.println("Start");
     });
     ArduinoOTA.onEnd([]() {
       Serial.println("\nEnd");
     });
     ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
       Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
     });
     ArduinoOTA.onError([](ota_error_t error) {
       Serial.printf("Error[%u]: ", error);
       if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed" + String(OTA_PASSWORD));
       else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
       else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
       else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
       else if (error == OTA_END_ERROR) Serial.println("End Failed");
     });
     ArduinoOTA.begin();
  #endif
}

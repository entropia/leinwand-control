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

#include "WiFi_credentials.hpp"
#include "MQTT_credentials.hpp"

//Global Constants
#define MEASURE 15000
#define PULSE 50

#define DOWN_PIN 13
#define STOP_PIN 12
#define   UP_PIN 14
#define MISC_PIN 16

//Object declarations
Adafruit_BME280 bme;
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_SERVER_PORT, MQTT_USER, MQTT_PASSWORD);
Adafruit_MQTT_Subscribe control(&mqtt, MQTT_CONTROL_TOPIC);

//Global Variables
unsigned long last = 0;

//Functions prototypes
void pulse(byte pin);
void setupWiFi();
void readBME(float *bmeData);
void mqtt_connect();
void mqtt_publish(float *bmeData);
void mqtt_callback(char *str, uint16_t len);

void setup() {
  Serial.begin(115200);
  setupWiFi();

  if(!bme.begin(0x76))
    Serial.println(F("Could not find a BME280 Sensor!"));

  delay(400);

  //Subscribe to the Leinwand-Controlfeed
  control.setCallback(mqtt_callback);
  mqtt.subscribe(&control);
  
  //Pin Setups
  pinMode(DOWN_PIN, OUTPUT);
  pinMode(STOP_PIN, OUTPUT);
  pinMode(UP_PIN, OUTPUT);
  pinMode(MISC_PIN, OUTPUT);
}


void loop() {
    mqtt_connect();    

    float bmeData[3];
    readBME(bmeData);

    mqtt_publish(bmeData);
    last = millis();

    mqtt.processPackets(MEASURE);
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

void mqtt_publish(float *bmeData) {
  Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, MQTT_TOPIC "/T");
  temperature.publish(bmeData[0]);

  Adafruit_MQTT_Publish pressure = Adafruit_MQTT_Publish(&mqtt, MQTT_TOPIC "/P");
  pressure.publish(bmeData[1]);

  Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, MQTT_TOPIC "/H");
  humidity.publish(bmeData[2]);
}


void mqtt_callback(char *str, uint16_t len) {
  if(len == 1) {
    char control = *str;
    
    Serial.println(control);

    switch(control) {
	case('u'):{
          pulse(UP_PIN);    
        }break;
        case('s'):{
          pulse(STOP_PIN);    
        }break;
        case('d'):{
          pulse(DOWN_PIN);    
	}break;
	case('m'):{
          pulse(MISC_PIN);    
	}break;
    }
  }
}

void pulse(byte pin){
  digitalWrite(pin, HIGH);
  delay(PULSE);
  digitalWrite(pin, LOW);
}

/***************************************************************************
  Author: /madonius (@madonius)
  Organisation: Entropia e.V. ~ CCC Karlsruhe
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include "Wifi_credentials.hpp"

Adafruit_BME280 bme;

//Functions prototypes
void setupWiFi();
bool readBME(float *bmeData);

void setup() {
  Serial.begin(115200);
  Serial.println(F("BMP280 test"));

  setupWiFi();
  bme.begin(0x76);
  delay(1337);
}


void loop() {
    float bmeData[3];
    bool bmesuccess = readBME(bmeData);
    Serial.println(bmeData[0]);
    delay(2000);
}

//Function definitions
//Connects to the WiFi
void setupWiFi() {
  WiFi.mode(WIFI_STA);

  while ( WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect network, SSID: ");
    Serial.println(SSID);
    WiFi.begin(SSID, WIFIPW);

    unsigned long start = millis();
    while(millis() - start < 20000 && WiFi.status() != WL_CONNECTED){
        delay(100);
    }
  }

  Serial.printf("\nWifi Conected! \\o/ \n IP Address:");
  Serial.println(WiFi.localIP());
}

//Reads the data from the sensor
bool readBME(float* bmeData) {
  float T = bme.readTemperature();
  delay(50);
  float P = bme.readPressure();
  delay(50);
  float H = bme.readHumidity();
  delay(50);
  bmeData[0] = T;
  bmeData[1] = P;
  bmeData[2] = H;
  return 1;
}

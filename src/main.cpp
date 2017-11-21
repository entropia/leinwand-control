/***************************************************************************
  Author: /madonius (@madonius)
  Organisation: Entropia e.V. ~ CCC Karlsruhe
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>

#include "Wifi_credentials.hpp"
#include "mqtt_credentials.hpp"

Adafruit_BME280 bme;
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_SERVER_PORT, MQTT_USER, MQTT_PASSWORD);

//Functions prototypes
void setupWiFi();
bool readBME(float *bmeData);
void MQTT_connect();
void mqtt_publish(float *bmeData);

void setup() {
  Serial.begin(115200);
  Serial.println(F("BMP280 test"));

  setupWiFi();
  bme.begin(0x76);
  delay(420);
}


void loop() {
    float bmeData[3];
    bool bmesuccess = readBME(bmeData);
    MQTT_connect();
    delay(15000);
    mqtt_publish(bmeData);
}

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

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print(F("Connecting to MQTT... "));

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println(F("Retrying MQTT connection in 5 seconds..."));
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
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

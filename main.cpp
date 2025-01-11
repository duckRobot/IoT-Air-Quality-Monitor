#include <WiFi.h>
#include <WiFiClient.h>
#include "DFRobot_AirQualitySensor.h"

const char* ssid = "";
const char* password = "";
const char* server = "api.thingspeak.com";
const char* apiKey = "";
const int MQ2_PIN = A0;
const int MQ135_PIN = A3;

#define I2C_ADDRESS 0x19
DFRobot_AirQualitySensor particle(&Wire, I2C_ADDRESS);

void setup() {
  Serial.begin(115200);
  pinMode(MQ2_PIN, INPUT);
  pinMode(MQ135_PIN, INPUT);

  while (!particle.begin()) {
    Serial.println();
    Serial.println("No Devices Detected!");
    delay(1000);
  }
  Serial.println();
  Serial.println("Sensor Initialization Success!");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  Serial.println("WiFi connected");
}

void loop() {
  int mq2Value = analogRead(MQ2_PIN);
  int mq135Value = analogRead(MQ135_PIN);
  float mq2Voltage = mq2Value * 5.0 / 1023.0;
  float mq135Voltage = mq135Value * 5.0 / 1023.0;

  uint16_t PM2_5 = particle.gainParticleConcentration_ugm3(PARTICLE_PM2_5_STANDARD);
  uint16_t PM1_0 = particle.gainParticleConcentration_ugm3(PARTICLE_PM1_0_STANDARD);
  uint16_t PM10 = particle.gainParticleConcentration_ugm3(PARTICLE_PM10_STANDARD);

  Serial.print("MQ2 Voltage: "); Serial.println(mq2Voltage);
  Serial.print("MQ135 Voltage: "); Serial.println(mq135Voltage);
  Serial.print("PM2.5: "); Serial.println(PM2_5);
  Serial.print("PM1.0: "); Serial.println(PM1_0);
  Serial.print("PM10: "); Serial.println(PM10);

  String url = "/update?api_key=" + String(apiKey) +
                "&field1=" + String(mq2Voltage) +
                "&field2=" + String(mq135Voltage) +
                "&field3=" + String(PM2_5) +
                "&field4=" + String(PM1_0) +
                "&field5=" + String(PM10);

  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    if (client.connect(server, 80)) {
      client.print("GET " + url + " HTTP/1.1\r\n");
      client.print("Host: api.thingspeak.com\r\n");
      client.print("Connection: close\r\n");
      client.print("\r\n");

      String line;
      bool foundOK = false;
      while (client.connected() || client.available()) {
        line = client.readStringUntil('\n');
        if (line.indexOf("200 OK") != -1) {
          foundOK = true;
          break;
        }
      }

      if (foundOK) {
        Serial.println("HTTP/1.1 200 OK");
      }

      client.stop();
      Serial.println("Connection closed.");
      Serial.println();
    } else {
      Serial.println("Connection to ThingSpeak failed.");
      Serial.println();
    }
  } else {
    Serial.println("WiFi not connected.");
    Serial.println();
  }

  delay(15000);
}

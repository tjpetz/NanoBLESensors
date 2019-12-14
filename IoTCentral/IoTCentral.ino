/*
 * Simple experiment to pair up with the Nano 33 BLE Sense to read the
 * sensor values from it via BLE.
 * 
 * Note as we cannot simultaneously connect the Wifi and BLE we will get a measurement
 * and then stop the BLE connection and connect to WiFi.
 * 
 * History:
 *  13 Dec 2019 - Add WiFi and publish the temp and humidity to an MQTT broker.
 *  8 Dec 2019 19:00Z - Added reading the temperature characteristic.
 */

#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include <ArduinoBLE.h>

#include "arduino_secret.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)


WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "bbsrv02.bblab.tjpetz.com";
int port = 1883;
const char topic[] = "tjpetz/environment";

bool updatedMeasurement = false;
float currentTemperature = 0.0;
float currentHumidity = 0.0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.begin(115200);
  delay(2000);     // Give serial a moment to start

  Serial.println("Starting BLE begin");
  while (!BLE.begin()) {
    Serial.println("Error starting BLE");
    delay(1000);
  }

  Serial.println("Initiating scan...");
  BLE.scan();
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

  String deviceName = "Nano33BLESense-test";

  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  
  if (BLE.scanForName(deviceName)) {
    
    BLEDevice peripheral = BLE.available();

    if(peripheral) {
      Serial.println("=========================================================");
      Serial.print("Local Name: "); Serial.println(peripheral.localName());
      Serial.print("Address: "); Serial.println(peripheral.address());
      Serial.print("RSSI: "); Serial.println(peripheral.rssi());

      BLE.stopScan();
      
      // print the advertised service UUIDs, if present
      if (peripheral.hasAdvertisedServiceUuid()) {
        Serial.print("Advertised Service UUIDs: ");
        for (int i = 0; i < peripheral.advertisedServiceUuidCount(); i++) {
          Serial.print(peripheral.advertisedServiceUuid(i));
          Serial.print(",");
        }
        Serial.println();
      }

      if (peripheral.connect()) {
        Serial.println("Connected.");

        if (peripheral.discoverService("181a")) {
          Serial.println("Humidity Service found");

            // Read the humidity
            BLECharacteristic humidityCharacteristic = peripheral.characteristic("2A6F");
            if (humidityCharacteristic) {
              uint16_t humidityRawValue;
              humidityCharacteristic.readValue(humidityRawValue); 
              currentHumidity = humidityRawValue / 100.0;
              Serial.print("Humidity = "); Serial.print(currentHumidity); Serial.println("%");
            } else {
              Serial.println("Cannot find humidity characteristic");
            }

            // Read the temperature
            BLECharacteristic temperatureCharacteristic = peripheral.characteristic("2A6E");
            if (temperatureCharacteristic) {
              int16_t temperatureRawValue;
              temperatureCharacteristic.readValue(temperatureRawValue);
              currentTemperature = temperatureRawValue / 100.0;
              Serial.print("Temperature = "); Serial.print(currentTemperature); Serial.println("C");
            } else {
              Serial.println("Cannot find temperature characteristic");
            }

        updatedMeasurement = true;    // We have a new measurement
        
        } else {
          Serial.println("Humidity Service not found");
        }
        peripheral.disconnect();
      } else {
        Serial.println("Failed to connect.");
      }
    }
  }

  if (updatedMeasurement) {
    // We have a measurement so connect to WiFi and send it to the MQTT broker.
    Serial.println("Attempting to send measurement");
    
    while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
      Serial.print(".");
      delay(1000);
    }
  
    Serial.println("Connected to the WiFi network");

    Serial.print("Attempting to connect to the MQTT broker: ");
    Serial.println(broker);
  
    if (!mqttClient.connect(broker, port)) {
      Serial.print("MQTT connection failed! Error code = ");
      Serial.println(mqttClient.connectError());
  
      while (1);
    }
  
    Serial.println("You're connected to the MQTT broker!");
    mqttClient.beginMessage(topic);
    mqttClient.print("temperature = ");
    mqttClient.print(currentTemperature);
    mqttClient.print(", humidity = ");
    mqttClient.print(currentHumidity);
    mqttClient.endMessage();
    
    Serial.println();
    delay(2000);
  
    Serial.println("Disconnecting from Wifi");
    WiFi.disconnect();
    WiFi.end();
    updatedMeasurement = false;
  }
  
  delay(5000);
  Serial.println("Restarting BLE scan");
  while (!BLE.begin()) {
      Serial.println("Waiting for BLE");
      delay(500);
  }
  BLE.scan();
}

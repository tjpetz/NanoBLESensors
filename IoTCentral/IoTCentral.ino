/*
 * Simple experiment to pair up with the Nano 33 BLE Sense to read the
 * sensor values from it via BLE.
 * 
 * Note as we cannot simultaneously connect the Wifi and BLE we will get a measurement
 * and then stop the BLE connection and connect to WiFi.
 * 
 * History:
 *  14 Dec 2019 - Very ugly code at this point.  It works but needs serious refactoring.
 *  13 Dec 2019 - Add WiFi and publish the temp and humidity to an MQTT broker.
 *  8 Dec 2019 19:00Z - Added reading the temperature characteristic.
 */

#include <Arduino_DebugUtils.h>

#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include <ArduinoBLE.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <RTCZero.h>
#include <time.h>
#include <ArduinoLowPower.h>

#include "arduino_secret.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)


WiFiClient wifiClient;                  // Our wifi client
MqttClient mqttClient(wifiClient);      // Our MQTT client
RTCZero rtc;                            // Real Time Clock so we can time stamp data


const char broker[] = "bbsrv02.bblab.tjpetz.com";
const int port = 1883;
const char topic[] = "tjpetz/environment";

bool updatedMeasurement = false;
float currentTemperature = 0.0;
float currentHumidity = 0.0;

void setup() {
  Serial.begin(115200);
  delay(500);     // Give serial a moment to start
 
  Debug.setDebugLevel(DBG_VERBOSE);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  
  // Try starting the WiFi to see if it's okay to have it running while using BLE
//  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
//    Debug.print(DBG_WARNING, "Waiting for Wifi to connect.");
//    delay(500);
//  }
//  Debug.print(DBG_INFO, "WiFi Connected, address = %d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);

//  initializeRTC();

  Debug.print(DBG_INFO, "Starting BLE");
  while (!BLE.begin()) {
    Debug.print(DBG_WARNING, "Error starting BLE");
    delay(1000);
  }

  // Try starting the WiFi to see if it's okay to have it running while using BLE
//  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
//    Debug.print(DBG_WARNING, "Waiting for Wifi to connect.");
//    delay(500);
//  }
//  Debug.print(DBG_INFO, "WiFi Connected, address = %d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);

//
////  Debug.print(DBG_INFO, "Initiating scan...");
//  BLE.scan();
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {

//  delay(1000);
//  return;
  
  String deviceName = "Nano33BLESense-test";

  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);

//  Debug.print(DBG_INFO, "Initiating BLE and Scan");
//  while(!BLE.begin()) {
//    Debug.print(DBG_WARNING, "Error starting the BLE");
//    delay(500);
//  }

  Debug.print(DBG_INFO, "Scanning for %s", deviceName.c_str());
  if (BLE.scanForName(deviceName)) {
    
    BLEDevice peripheral = BLE.available();

    if(peripheral) {
      Debug.print(DBG_INFO, "=========================================================");
      Debug.print(DBG_INFO, "Local Name: %s", peripheral.localName().c_str());
      Debug.print(DBG_INFO, "Address: %s", peripheral.address().c_str());
      Debug.print(DBG_INFO, "RSSI: %d", peripheral.rssi());

      BLE.stopScan();   // we have to stop scanning before we can connect to a peripheral
      
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

        // disconnect and end so we can send the measurement
        peripheral.disconnect();
        BLE.end();
        } else {
          Serial.println("Humidity Service not found");
        }
//        delay(500);
        // disconnect so we can start a new scan.
        peripheral.disconnect();
//        delay(500);
//        BLE.end();
      } else {
        Serial.println("Failed to connect.");
      }
    }
  }

//  updatedMeasurement = false;
  if (updatedMeasurement) {
    Serial.println("Wait for the Nina module to reset/restart");
    delay(3500);
    
    // We have a measurement so connect to WiFi and send it to the MQTT broker.
    Serial.println("Attempting to send measurement");

     while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
      Serial.print(".");
      delay(3000);
    }
  
    Serial.println("Connected to the WiFi network");

    Debug.print(DBG_INFO, "Attempting to get the WiFi status");
    uint8_t stat = WiFi.status();
    Serial.print("Status = "); Serial.println(stat);
    
//    if (WiFi.status() != WL_CONNECTED) {
//      Debug.print(DBG_ERROR, "ERROR: the wifi network is not connected!");
//    }
//    
    Debug.print(DBG_INFO, "Attempting to connect to the MQTT broker: %s", broker);
  
    if (!mqttClient.connect(broker, port)) {
      Debug.print(DBG_ERROR, "MQTT connection failed! Error code = %d", mqttClient.connectError());
    } else {
      Debug.print(DBG_INFO, "You're connected to the MQTT broker!");
      Debug.print(DBG_INFO, "Topic = %s", topic);
      mqttClient.beginMessage(topic);
      char dateTime[20];
      sprintf(dateTime, "%04d-%02d-%02dT%02d:%02d:%02d", rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
      Debug.print(DBG_INFO, "Sample Time = %s", dateTime);
      char msg[80];
      sprintf(msg, "{sampleTime=%s,temperature=%.2f,humidity=%.2f}", dateTime, currentTemperature, currentHumidity);
      Debug.print(DBG_INFO, "Message = %s", msg);
//      mqttClient.print("test");
      mqttClient.print(msg);
      mqttClient.endMessage();
    }
//    
//    Serial.println();
    delay(2000);
//  
//    Serial.println("Disconnecting from Wifi");
    WiFi.disconnect();
//    WiFi.end();
    delay(2500);
//    Debug.print(DBG_INFO, "Restarting BLE");
    // Restart BLE
    if (!BLE.begin()) {
      Serial.println("Failed to start BLE");
    } else {
      Serial.println("Restarted BLE");
    }
    updatedMeasurement = false;
  }
  
//  delay(5000);
//  Serial.println("Restarting BLE scan");
//  while (!BLE.begin()) {
//      Serial.println("Waiting for BLE");
//      delay(500);
//  }
//  BLE.scan();
}

// Initialize the RTC by calling NTP and setting the initial time in the RTC
void initializeRTC() {
  // Note these are both locally scoped as we do not need them after the RTC is initialized.
  WiFiUDP ntpUDP;                         // Need UDP when calling NTP server
  NTPClient timeClient(ntpUDP);           // NTP client - used to initialize the RTC when we don't have a battery
  bool wifiConnected = false;             // If WiFi is connected when called then we'll leave it connected on exit, otherwise we'll disconnect it.

  Debug.print(DBG_INFO, "Initializing the RTC via NTP");
  
  wifiConnected = (WiFi.status() == WL_CONNECTED);
  if (!wifiConnected) {
    Debug.print(DBG_INFO, "Starting WiFi for NTP Connection");
    while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
      Debug.print(DBG_WARNING, "Waiting for Wifi to connect.");
      delay(3000);
    }
    Debug.print(DBG_INFO, "WiFi Connected, address = %d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
  }

  timeClient.update();    // get the current time from pool.ntp.com

  time_t ntpTime = timeClient.getEpochTime();
  struct tm* t = gmtime(&ntpTime);    // convert Unix epoch time to tm struct format
  Debug.print(DBG_INFO, "NTP Time = %04d-%02d-%02d %02d:%02d:%02d", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);

  // Setup the RTC
  rtc.begin();
  rtc.setEpoch(ntpTime);
  Debug.print(DBG_INFO, "RTC Time = %04d-%02d-%02d %02d:%02d:%02d", rtc.getYear(), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());

  if(!mqttClient.connect(broker, port)) {
      Serial.println("Error Connecting to mqtt broker");
  } else {
    const char topic[] = "tjpetz/environment";
    char buff[255];
    sprintf(buff, "{status: 'boot at %04d-%02d-%02d %02d:%02d:%02d'}", rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds()); 
    Serial.println("Connected to mqtt broker!");
    mqttClient.beginMessage(topic);
    mqttClient.print(buff);
    mqttClient.endMessage();
    Serial.println("Sent message");
  }

  delay(2500);
  // if the WiFi was connected when called leave it connected.  Otherwise end it.
  if (!wifiConnected) {
    Debug.print(DBG_INFO, "Turn off wifi as we have finished setting the time via NTP.");
    WiFi.end(); 
  }
}

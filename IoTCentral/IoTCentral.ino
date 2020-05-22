/*
 * Simple experiment to pair up with the Nano 33 BLE Sense to read the
 * sensor values from it via BLE.
 * 
 * Note as we cannot simultaneously connect the Wifi and BLE we will get a measurement
 * and then stop the BLE connection and connect to WiFi.
 * 
 * History:
 *  22 May 2020 - Refactored, added the host name for the WiFi connection as there is some
 *    reference on the web that timeouts and rejects on Unifi wifi network may be due to
 *    missing hostname.
 *  27 Dev 2019 - After some experimentation we need a 6 second delay between stoping either 
 *    BLE or Wifi and switching to the other.  The BLE code clearly does an NRESET on the 
 *    Nina module which should take about 2.5S to reset.  However, when I set the delay to
 *    much less than 6 seconds things stop after running a while.  Future fix should include
 *    a watch dog timer and perhaps and update to the BLE code as a full NRESET should not
 *    be necessary to use the BLE module.
 *    Note, we also need a delay after sending our mqtt message before turning off the wifi.
 *    Also starting to develop some old school macros for debugging.  Using Serial.print
 *    or the debugging code both leave the code running in a production release.  This
 *    takes space and consumes some power so I'd perfer they are only enabled when debugging.
 *  14 Dec 2019 - Very ugly code at this point.  It works but needs serious refactoring.
 *  13 Dec 2019 - Add WiFi and publish the temp and humidity to an MQTT broker.
 *  8 Dec 2019 19:00Z - Added reading the temperature characteristic.
 */

#include <Arduino.h>
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

#define _DEBUG_
#ifdef _DEBUG_
#define MAX_DEBUG_BUFF    255
#define DEBUG_PRINTF(...) {char _buff[MAX_DEBUG_BUFF]; snprintf(_buff, MAX_DEBUG_BUFF, __VA_ARGS__); SerialUSB.print(_buff);}
#else
#define DEBUG_PRINTF(buff, fmt, ...)
#endif

WiFiClient wifiClient;                  // Our wifi client
MqttClient mqttClient(wifiClient);      // Our MQTT client
RTCZero rtc;                            // Real Time Clock so we can time stamp data

const int ninaRebootDelay = 3000;       // Time (mS) between ending either Wifi or BLE and starting the other
const int mqttTxDelay = 2500;           // Time (mS) between the last call to mqtt and turning off the wifi.
const unsigned long resetEverymS = 15 * 60 * 1000;      // Reset every 15 minutes to work around instabilities.
unsigned long now = 0;
unsigned long lastMeasureTime = 0;

const char broker[] = "bbsrv02.bblab.tjpetz.com";
const int port = 1883;
const char topic[] = "tjpetz/environment2";

const char hostname[] = "iot_central_001";

bool updatedMeasurement = false;
float currentTemperature = 0.0;
float currentHumidity = 0.0;
float currentPressure = 0.0;
String connectedDeviceName;

const char environmentServiceUUID[] = "181A";       // the standard UUID for the environment service

void setup() {
  Serial.begin(115200);
  delay(2000);     // Give serial a moment to start
 
  DEBUG_PRINTF("RESET Register = 0x%0x\n", PM->RCAUSE.reg);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  initializeRTC();

  DEBUG_PRINTF("Starting BLE\n");
  while (!BLE.begin()) {
    DEBUG_PRINTF("Error starting BLE\n");
    delay(1000);
  }

  now = millis();
  lastMeasureTime = now;

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {

  now = millis();
  if (now - lastMeasureTime >= (3 * 1000)) {
    lastMeasureTime = now;

    DEBUG_PRINTF("Scanning for environmentService (%lu)\n", millis());
    if (BLE.scanForUuid(environmentServiceUUID)) {
      
      BLEDevice peripheral = BLE.available();

      if(peripheral) {
        DEBUG_PRINTF("=========================================================\n");
        DEBUG_PRINTF("\tLocal Name: %s\n", peripheral.localName().c_str());
        DEBUG_PRINTF("\tAddress: %s\n", peripheral.address().c_str());
        DEBUG_PRINTF("\tRSSI: %d\n", peripheral.rssi());

        connectedDeviceName = peripheral.localName();
        
        BLE.stopScan();   // we have to stop scanning before we can connect to a peripheral
        
        #ifdef _DEBUG_
        // print the advertised service UUIDs, if present
        if (peripheral.hasAdvertisedServiceUuid()) {
          DEBUG_PRINTF("Advertised Service UUIDs: ");
          for (int i = 0; i < peripheral.advertisedServiceUuidCount(); i++) {
            DEBUG_PRINTF("%s, ", peripheral.advertisedServiceUuid(i).c_str());
          }
          DEBUG_PRINTF("\n");
        }
        #endif
        
        if (peripheral.connect()) {
          DEBUG_PRINTF("Connected.\n");

          if (peripheral.discoverService("181a")) {
            DEBUG_PRINTF("Environment Service found\n");

            // Read the humidity
            BLECharacteristic humidityCharacteristic = peripheral.characteristic("2A6F");
            if (humidityCharacteristic) {
              uint16_t humidityRawValue;
              humidityCharacteristic.readValue(humidityRawValue); 
              currentHumidity = humidityRawValue / 100.0;
              DEBUG_PRINTF("Humidity = %.2f %%\n", currentHumidity);
            } else {
              DEBUG_PRINTF("Cannot find humidity characteristic\n");
            }

            // Read the temperature
            BLECharacteristic temperatureCharacteristic = peripheral.characteristic("2A6E");
            if (temperatureCharacteristic) {
              int16_t temperatureRawValue;
              temperatureCharacteristic.readValue(temperatureRawValue);
              currentTemperature = temperatureRawValue / 100.0;
              DEBUG_PRINTF("Temperature = %.2f C\n", currentTemperature);
            } else {
              DEBUG_PRINTF("Cannot find temperature characteristic\n");
            }

            // Read the pressure
            BLECharacteristic pressureCharacteristic = peripheral.characteristic("2A6D");
            if (pressureCharacteristic) {
              uint32_t pressureRawValue;
              pressureCharacteristic.readValue(pressureRawValue);
              currentPressure = pressureRawValue * 0.00001450; // convert to PSI
              DEBUG_PRINTF("Pressure = %.2f PSI\n", currentPressure);
            } else {
              DEBUG_PRINTF("Cannot find pressure characteristic\n");
            }

            updatedMeasurement = true;    // We have a new measurement
    
          } else {
            DEBUG_PRINTF("Environment Service not found\n");
          }
          // disconnect so we can start a new scan.
          peripheral.disconnect();
        } else {
          DEBUG_PRINTF("Failed to connect.\n");
        }
      }
    }

    if (updatedMeasurement) {
      // disconnect and end BLE before starting the WiFi
      BLE.disconnect();
      BLE.end();
      
      delay(50);

      // We have a measurement so connect to WiFi and send it to the MQTT broker.
      DEBUG_PRINTF("Attempting to send measurement\n");

      WiFi.setHostname(hostname);
      WiFi.setTimeout(45 * 1000);    // 45 sec connection timeout
      if (WiFi.begin(ssid, pass) == WL_CONNECTED) {
    
        DEBUG_PRINTF("Attempting to connect to the MQTT broker: %s\n", broker);
      
        if (!mqttClient.connect(broker, port)) {
          DEBUG_PRINTF("MQTT connection failed! Error code = %d\n", mqttClient.connectError());
        } else {
          DEBUG_PRINTF("You're connected to the MQTT broker!\n");
          DEBUG_PRINTF("Topic = %s\n", topic);
          mqttClient.beginMessage(topic);
          char dateTime[20];
          snprintf(dateTime, sizeof(dateTime), "%04d-%02d-%02dT%02d:%02d:%02d", rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
          DEBUG_PRINTF("Sample Time = %s\n", dateTime);
          char msg[255];
          snprintf(msg, sizeof(msg), "{ \"sensor\": \"%s\", \"sampleTime\": \"%s\", \"temperature\": %.2f, \"humidity\": %.2f, \"pressure\": %.2f }", connectedDeviceName.c_str(), dateTime, currentTemperature, currentHumidity, currentPressure);
          DEBUG_PRINTF("Message = %s\n", msg);
          mqttClient.print(msg);
          mqttClient.endMessage();
          delay(mqttTxDelay);
        }
        
        DEBUG_PRINTF("Disconnecting from Wifi\n");
        WiFi.disconnect();
        WiFi.end();
      } else {
        DEBUG_PRINTF("Error/timeout connecting to WiFi: %d\n", WiFi.reasonCode());
        WiFi.disconnect();
        WiFi.end();
      }

      // Restart BLE
      if (!BLE.begin()) {
        DEBUG_PRINTF("Failed to start BLE\n");
      } else {
        DEBUG_PRINTF("Restarted BLE\n");
      }
      updatedMeasurement = false;
    }

    delay(50);     // give things a momemnt to start
  }
 }

// Initialize the RTC by calling NTP and setting the initial time in the RTC
void initializeRTC() {
  // Note these are both locally scoped as we do not need them after the RTC is initialized.
  WiFiUDP ntpUDP;                         // Need UDP when calling NTP server
  NTPClient timeClient(ntpUDP);           // NTP client - used to initialize the RTC when we don't have a battery
  bool wifiConnected = false;             // If WiFi is connected when called then we'll leave it connected on exit, otherwise we'll disconnect it.

  DEBUG_PRINTF("Initializing the RTC via NTP\n");
  
  WiFi.setHostname(hostname);

  wifiConnected = (WiFi.status() == WL_CONNECTED);
  if (!wifiConnected) {
    DEBUG_PRINTF("Starting WiFi for NTP Connection\n");
    while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
      DEBUG_PRINTF("Waiting for Wifi to connect.\n");
      delay(3000);
    }
    DEBUG_PRINTF("WiFi Connected, address = %d.%d.%d.%d\n", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
  }

  timeClient.update();    // get the current time from pool.ntp.com

  time_t ntpTime = timeClient.getEpochTime();
  struct tm* t = gmtime(&ntpTime);    // convert Unix epoch time to tm struct format
  DEBUG_PRINTF("NTP Time = %04d-%02d-%02d %02d:%02d:%02d\n", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);

  // Setup the RTC
  rtc.begin();
  rtc.setEpoch(ntpTime);
  DEBUG_PRINTF("RTC Time = %04d-%02d-%02d %02d:%02d:%02d\n", rtc.getYear(), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());

  if(!mqttClient.connect(broker, port)) {
      DEBUG_PRINTF("Error Connecting to mqtt broker\n");
  } else {
    const char topic[] = "tjpetz/environment";
    char buff[255];
    snprintf(buff, sizeof(buff), "{\"status\": \"boot at %04d-%02d-%02d %02d:%02d:%02d\"}", rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds()); 
    DEBUG_PRINTF("Connected to mqtt broker!\n");
    mqttClient.beginMessage(topic);
    mqttClient.print(buff);
    mqttClient.endMessage();
    DEBUG_PRINTF("Sent message\n");
  }

  delay(mqttTxDelay);
  
  // if the WiFi was connected when called leave it connected.  Otherwise end it.
  if (!wifiConnected) {
    DEBUG_PRINTF("Turn off wifi as we have finished setting the time via NTP.\n");
    WiFi.disconnect();
    WiFi.end(); 
  }

}

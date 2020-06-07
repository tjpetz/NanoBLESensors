/*
 * Simple experiment to pair up with the Nano 33 BLE Sense to read the
 * sensor values from it via BLE.
 * 
 * Note as we cannot simultaneously connect the Wifi and BLE we will get a measurement
 * and then stop the BLE connection and connect to WiFi.
 * 
 * History:
 *  6 Jun 2020 - reworked the BLE scan process.  We don't start a new scan each loop.
 *    Rather we continue with the current scan.  When we find a new device we stop
 *    scanning.  TODO: if we do not find the Environment service after connection we
 *    currently fail to correctly restart the scan.  Thus we enter an endless loop.
 *    I'm looking at implementing a proper FSM to deal with this and other conditions.
 *    Also broke out the route to send the measurements via WiFi to the MQTT server.
 *    Retrieve the location setting from the sensor and added the location to the MQTT
 *    message.
 *  4 Jun 2020 - update to use more generic name for the mqtt broker.
 *  31 May 2020 - After a few hours the board is unable to connect anymore to the BLE
 *    Sense.  Rebooting established reconnection.  Try adding a reset counter.  If after
 *    some number of failed connects, reset the board.
 *  22 May 2020 - Refactored, added the host name for the WiFi connection as there is some
 *    reference on the web that timeouts and rejects on Unifi wifi network may be due to
 *    missing hostname.
 *  14 Jan 2019 - Experimenting with driver reset approach as documented on community forum
 *    to deal with switching between WiFi and BLE.  (https://forum.arduino.cc/index.php?topic=657710.0)
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
#include <RTCZero.h>
#include <time.h>
#include <ArduinoLowPower.h>

#include "arduino_secret.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

#define _DEBUG_
#include "Debug.h"

WiFiClient wifiClient;                  // Our wifi client
MqttClient mqttClient(wifiClient);      // Our MQTT client
RTCZero rtc;                            // Real Time Clock so we can time stamp data
int wifiStatus = WL_IDLE_STATUS;        // Keep track of the WiFi status

const int ninaRebootDelay = 3000;       // Time (mS) between ending either Wifi or BLE and starting the other
const int mqttTxDelay = 2500;           // Time (mS) between the last call to mqtt and turning off the wifi.
const unsigned long resetEverymS = 15 * 60 * 1000;      // Reset every 15 minutes to work around instabilities.
unsigned long now = 0;
unsigned long lastMeasureTime = 0;

const char broker[] = "mqtt.bb.tjpetz.com";
const int port = 1883;

const char topicRoot[] = "tjpetz.com/sensor";
const unsigned int MAX_TOPIC_LENGTH = 255;

const char hostname[] = "iot_central_001";


bool updatedMeasurement = false;
float currentTemperature = 0.0;
float currentHumidity = 0.0;
float currentPressure = 0.0;
String currentLocationName;
String connectedDeviceName;
byte macAddress[6];               // MAC address of the Wifi which we'll use in reporting

const char environmentServiceUUID[] = "181A";       // the standard UUID for the environment service

const int maxFailedConnections = 5;             // force a reset after this many failed connections.
int failedConnectionsCounter = 0;               // counter to keep track of failed connections

void setup() {
  
  Serial.begin(115200);
  delay(2000);     // Give serial a moment to start
 
  DEBUG_PRINTF("RESET Register = 0x%0x\n", PM->RCAUSE.reg);

  WiFi.macAddress(macAddress);
  DEBUG_PRINTF("MAC Address = %02x:%02x:%02x:%02x:%02x:%02x\n", macAddress[5], macAddress[4], 
    macAddress[3], macAddress[2], macAddress[1], macAddress[0]);

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

  BLE.setConnectionInterval(8, 40);    // 10 - 50 mS connection interval

  BLE.scanForUuid(environmentServiceUUID);

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {

  now = millis();
  if (now - lastMeasureTime >= (3 * 1000)) {
    lastMeasureTime = now;

    BLEDevice peripheral = BLE.available();

    digitalWrite(LED_BUILTIN, HIGH);
    // Attempt to connect for 2 seconds
    while (!peripheral && (millis() - lastMeasureTime <= 2000)) {

      peripheral = BLE.available();
      if (!peripheral) {
        BLE.poll();  
        delay(50);
      }
    }
    digitalWrite(LED_BUILTIN, LOW);

    // Wait for connection
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
        DEBUG_PRINTF("Connected to %s.\n", peripheral.localName().c_str());

        // reset the failed connection counter
        failedConnectionsCounter = 0;

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

          // Read the location
          BLECharacteristic locationNameCharacteristic = peripheral.characteristic("2AB5");
          if (locationNameCharacteristic) {
            char buff[255];
            memset(buff, 0, sizeof(buff));    // zero out the buffer.
            locationNameCharacteristic.readValue(buff, sizeof(buff));
            currentLocationName = buff;
            DEBUG_PRINTF("Location Name = %s\n", currentLocationName.c_str());
          } else {
            DEBUG_PRINTF("Cannot find locationName characteristic\n");
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
    } else {
        DEBUG_PRINTF("Peripheral not available.\n");
    }
    // } else {
    //   if (failedConnectionsCounter >= maxFailedConnections) {
    //     // Too many failures try a reset.
    //     DEBUG_PRINTF("Forcing a reset due to too many failed scans.\n");
    //     NVIC_SystemReset();
    //   } else {
    //     ++failedConnectionsCounter;
    //   }
    }

  if (updatedMeasurement) {
    // disconnect and end BLE before starting the WiFi
    BLE.disconnect();
    BLE.end();
    
    delay(50);

    sendMeasurementsToMQTT();

    // Restart BLE
    if (!BLE.begin()) {
      DEBUG_PRINTF("Failed to start BLE\n");
    } else {
      DEBUG_PRINTF("Restarted BLE\n");
    }
    updatedMeasurement = false;
    BLE.scanForUuid(environmentServiceUUID);
  }

  delay(50);

}

// Initialize the RTC by calling NTP and setting the initial time in the RTC
void initializeRTC() {
  // Note these are both locally scoped as we do not need them after the RTC is initialized.

  DEBUG_PRINTF("Initializing the RTC via NTP\n");
  
  DEBUG_PRINTF("Starting WiFi for NTP Connection\n");
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    DEBUG_PRINTF("Waiting for Wifi to connect.\n");
    delay(3000);
  }
  DEBUG_PRINTF("WiFi Connected, address = %d.%d.%d.%d\n", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);

  time_t ntpTime = WiFi.getTime();
  int maxTries = 10;
  int tries = 0;

  // try a few times to get the time, it takes a moment to reach the NTP servers.
  while (ntpTime == 0 && tries < maxTries) {
    ntpTime = WiFi.getTime();
    tries++;
    delay(3000);
  }

// TODO: handle the situation if no time is available.

  struct tm* t = gmtime(&ntpTime);    // convert Unix epoch time to tm struct format
  DEBUG_PRINTF("NTP Time = %04d-%02d-%02d %02d:%02d:%02d\n", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);

  // Setup the RTC
  rtc.begin();
  rtc.setEpoch(ntpTime);
  DEBUG_PRINTF("RTC Time = %04d-%02d-%02d %02d:%02d:%02d\n", rtc.getYear(), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());

  if(!mqttClient.connect(broker, port)) {
      DEBUG_PRINTF("Error Connecting to mqtt broker\n");
  } else {
    const char topic[] = "tjpetz/environment/temp";
    char buff[255];
    snprintf(buff, sizeof(buff), "{\"status\": \"boot at %04d-%02d-%02d %02d:%02d:%02d\"}", rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds()); 
    DEBUG_PRINTF("Connected to mqtt broker!\n");
    mqttClient.beginMessage(topic);
    mqttClient.print(buff);
    mqttClient.endMessage();
    DEBUG_PRINTF("Sent message\n");
  }

  mqttClient.stop();
   
  WiFi.disconnect();
  WiFi.end(); 
}

void sendMeasurementsToMQTT() {
  // We have a measurement so connect to WiFi and send it to the MQTT broker.
  DEBUG_PRINTF("Attempting to send measurement\n");

  WiFi.setHostname(hostname);
  WiFi.setTimeout(45 * 1000);    // 45 sec connection timeout
  if (WiFi.begin(ssid, pass) == WL_CONNECTED) {

    DEBUG_PRINTF("Attempting to connect to the MQTT broker: %s\n", broker);
  
    mqttClient.setConnectionTimeout(4000);
    if (!mqttClient.connect(broker, port)) {
      DEBUG_PRINTF("MQTT connection failed! Error code = %d\n", mqttClient.connectError());
    } else {
      DEBUG_PRINTF("You're connected to the MQTT broker!\n");

      char topic[MAX_TOPIC_LENGTH];

      snprintf(topic, sizeof(topic), "%s/%s", topicRoot, connectedDeviceName.c_str());

      DEBUG_PRINTF("Topic = %s\n", topic);
      mqttClient.beginMessage(topic);
      char dateTime[20];
      snprintf(dateTime, sizeof(dateTime), "%04d-%02d-%02dT%02d:%02d:%02d", rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
      DEBUG_PRINTF("Sample Time = %s\n", dateTime);
      char msg[255];
      snprintf(msg, sizeof(msg), "{ \"sensor\": \"%s\", \"location\": \"%s\", \"sampleTime\": \"%s\", \"temperature\": %.2f, \"humidity\": %.2f, \"pressure\": %.2f }", 
        connectedDeviceName.c_str(), currentLocationName.c_str(), dateTime, currentTemperature, currentHumidity, currentPressure);
      DEBUG_PRINTF("Message = %s\n", msg);
      mqttClient.print(msg);
      mqttClient.endMessage();
      delay(mqttTxDelay);
    }
    mqttClient.stop();
    DEBUG_PRINTF("Disconnecting from Wifi\n");
  } 
  else {
    DEBUG_PRINTF("Error/timeout connecting to WiFi: %d\n", WiFi.reasonCode());
  }
  WiFi.disconnect();
  WiFi.end();
}
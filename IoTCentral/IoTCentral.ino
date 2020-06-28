/*
 * Read values from a Nano 33 BLE Sense board and send them to an mqtt broker
 * via WiFi.
 * 
 * Note as we cannot simultaneously connect the Wifi and BLE we will get a measurement
 * and then stop the BLE connection and connect to WiFi.
 */

#include <Arduino.h>
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include <ArduinoBLE.h>
#include <RTCZero.h>
#include <time.h>
#include <ArduinoLowPower.h>
#include <Adafruit_SleepyDog.h>

#include "arduino_secret.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

#define _DEBUG_
#include "Debug.h"

RTCZero rtc;                            // Real Time Clock so we can time stamp data

// WiFi and MQTT
WiFiClient wifiClient;                  // Our wifi client
MqttClient mqttClient(wifiClient);      // Our MQTT client
const int mqttTxDelay = 2500;           // Time (mS) between the last call to mqtt and turning off the wifi.
const char broker[] = "mqtt.bb.tjpetz.com";
const int port = 1883;
const char topicRoot[] = "tjpetz.com/sensor";
const unsigned int MAX_TOPIC_LENGTH = 255;
const char hostname[] = "iot_central_001";
byte macAddress[6];               // MAC address of the Wifi which we'll use in reporting

// The Nano 33 BLE Sense sensor and it's characteristics.
BLEDevice sensorPeripheral;
const char environmentServiceUUID[] = "181A";       // the standard UUID for the environment service
BLECharacteristic humidityCharacteristic;
BLECharacteristic temperatureCharacteristic;
BLECharacteristic pressureCharacteristic;
BLECharacteristic locationNameCharacteristic;

// Current state measurements
float currentTemperature = 0.0;
float currentHumidity = 0.0;
float currentPressure = 0.0;
String currentLocationName;
String connectedDeviceName;

// Retry counters
const int maxTries = 10;             
int retryCounter = 0;               // counter to keep track of failed connections
bool success = false;

// State machine states
typedef enum FSMStates {
    initializing,
    start_scan,
    scanning,
    connecting,
    connected,
    reading_measurements,
    sending_measurements,
    idle,
    restart
} FSMState_t;

#ifdef _DEBUG_
char debugStateNames [9][32] = {
  "initializing", 
  "start_scan", 
  "scanning", 
  "connecting", 
  "connected", 
  "reading_measurements", 
  "sending_measurements",
  "idle",
  "restart" 
};
#endif

FSMState_t currentState = initializing;
FSMState_t nextState = initializing;
FSMState_t previousState = initializing;

const unsigned long idleTime = 15000;             // Time to spend in idle in mS
const int watchdogTimeout = 60000;

unsigned long now = 0;
unsigned long lastMeasureTime = 0;
unsigned long scanStartTime = 0;
unsigned long maxScanTime = 30000;              // Time to scan before going idle

unsigned long startDelayTime = 0;

void setup() {
  
  Serial.begin(115200);
  delay(3000);     // Give serial a moment to start
 
  DEBUG_PRINTF("RESET Register = 0x%0x\n", PM->RCAUSE.reg);

  WiFi.macAddress(macAddress);
  DEBUG_PRINTF("MAC Address = %02x:%02x:%02x:%02x:%02x:%02x\n", macAddress[5], macAddress[4], 
    macAddress[3], macAddress[2], macAddress[1], macAddress[0]);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Watchdog.enable(watchdogTimeout);

  initializeRTC();
  Watchdog.reset();

  DEBUG_PRINTF("Starting BLE\n");
  while (!BLE.begin()) {
    DEBUG_PRINTF("Error starting BLE\n");
    delay(1000);
  }

  now = millis();
  lastMeasureTime = now;

  currentState = start_scan;
  previousState = initializing;

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {

  switch (currentState) {

    case start_scan:
      
      BLE.scanForUuid(environmentServiceUUID);
      scanStartTime = millis();
      nextState = scanning;
      break;
  
    case scanning:

      BLE.poll();

      sensorPeripheral = BLE.available();
      digitalWrite(LED_BUILTIN, HIGH);

      if (sensorPeripheral) {
        digitalWrite(LED_BUILTIN, LOW);
        nextState = connecting;
      } else if (millis() - scanStartTime <= maxScanTime) {
        nextState = scanning;
      } else
      {
        digitalWrite(LED_BUILTIN, LOW);
        nextState = idle;
      }
      
      break;

    case connecting:

      BLE.poll();

      DEBUG_PRINTF("=========================================================\n");
      DEBUG_PRINTF("\tLocal Name: %s\n", sensorPeripheral.localName().c_str());
      DEBUG_PRINTF("\tAddress: %s\n", sensorPeripheral.address().c_str());
      DEBUG_PRINTF("\tRSSI: %d\n", sensorPeripheral.rssi());

      connectedDeviceName = sensorPeripheral.localName();
      
      BLE.stopScan();   // we have to stop scanning before we can connect to a peripheral
      
      #ifdef _DEBUG_
      // print the advertised service UUIDs, if present
      if (sensorPeripheral.hasAdvertisedServiceUuid()) {
        DEBUG_PRINTF("Advertised Service UUIDs: ");
        for (int i = 0; i < sensorPeripheral.advertisedServiceUuidCount(); i++) {
          DEBUG_PRINTF("%s, ", sensorPeripheral.advertisedServiceUuid(i).c_str());
        }
        DEBUG_PRINTF("\n");
      }
      #endif
  
      if (sensorPeripheral.connect()) {
        nextState = connected;
      } else {
        nextState = start_scan;       // Restart the scan if we failed to connect.
      }
      break;

    case connected:

      BLE.poll();

      if (sensorPeripheral.discoverService("181a")) {
        nextState = reading_measurements;
      } else {
        nextState = start_scan;       // The environment service is missing so start the scan again.
      }
      break;

    case reading_measurements:

      BLE.poll();

      readMeasurements();

      // disconnect as we're finished reading.
      sensorPeripheral.disconnect();

      // disconnect and end BLE before starting the WiFi
      BLE.disconnect();
      BLE.end();
      
      delay(1750);      // Delay to allow the Nina radio module to reset

      nextState = sending_measurements;
      break;

    case sending_measurements:

      retryCounter = 0;
      success = sendMeasurementsToMQTT();

      while (!success && (retryCounter < maxTries)) {
        delay(500 * (retryCounter + 1));
        ++retryCounter;
        success = sendMeasurementsToMQTT();
      }

      if (success) {
        nextState = idle;
      } else {
        nextState = restart;
      }
      
      break;

    case idle:

      DEBUG_PRINTF("Entering idle.\n");

      startDelayTime = millis();

      #ifdef _DEBUG_ 
        delay(idleTime);      // delay to prevent USB from being closed
      #else
        LowPower.sleep(idleTime);
      #endif
      
      DEBUG_PRINTF("Waking from idle. Slept for %lu mS.  Restarting BLE.\n", millis() - startDelayTime);

      Watchdog.reset();

      // Restart BLE
      retryCounter = 0;
      success = BLE.begin();
      while (!success && retryCounter < maxTries) {
        ++retryCounter;
        delay(100 * (retryCounter +1));
        success = BLE.begin();
      }
 
      if (success) {
        nextState = start_scan;
      } else {
        DEBUG_PRINTF("Failed to restart BLE so rebooting!\n");
        nextState = restart; 
      }

      break;

    case restart:

      DEBUG_PRINTF("Rebooting!\n");
      Serial.flush();
      delay(3000);  

      NVIC_SystemReset();
      break;
  }

  #ifdef _DEBUG_
  if (nextState != currentState) {
    DEBUG_PRINTF("Changing state from %s(%d) to %s(%d)\n", 
      debugStateNames[currentState], currentState, 
      debugStateNames[nextState], nextState);
  }
  #endif

  previousState = currentState;
  currentState = nextState;

  Watchdog.reset();
}

// Initialize the RTC by calling NTP and setting the initial time in the RTC
void initializeRTC() {
  int maxTries = 10;
  int tries = 0;

  DEBUG_PRINTF("Initializing the RTC via NTP\n");
  DEBUG_PRINTF("Starting WiFi for NTP Connection\n");

  while ((WiFi.begin(ssid, pass) != WL_CONNECTED) && (tries < maxTries)) {
    DEBUG_PRINTF("initializeRTC - Connection failed, reason = %d.\n", WiFi.reasonCode());
    tries++;
    WiFi.disconnect();
    WiFi.end();
    delay(1500 * tries);
  }

  if (WiFi.status() != WL_CONNECTED) {
    DEBUG_PRINTF("Failed to connect to wifi after %d tries.  Rebooting!\n", tries);
    Serial.flush();
    delay(3000);
    NVIC_SystemReset();    
  }

  DEBUG_PRINTF("WiFi Connected, address = %d.%d.%d.%d\n", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);

  time_t ntpTime = WiFi.getTime();

  // try a few times to get the time, it takes a moment to reach the NTP servers.
  while (ntpTime == 0 && tries < maxTries) {
    ntpTime = WiFi.getTime();
    tries++;
    delay(500 * (tries + 1));
  }

  // If we still have failed to get the time reset the board.
  if (ntpTime == 0) {
    DEBUG_PRINTF("Cannot get time from NTP so forcing a reset.\n");
    // gracefully disconnect before rebooting
    WiFi.disconnect();
    WiFi.end();
    Serial.flush();
    delay(3000);
    NVIC_SystemReset();
  }

  struct tm* t = gmtime(&ntpTime);    // convert Unix epoch time to tm struct format
  DEBUG_PRINTF("NTP Time = %04d-%02d-%02d %02d:%02d:%02d\n", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);

  // Setup the RTC
  rtc.begin();
  rtc.setEpoch(ntpTime);
  DEBUG_PRINTF("RTC Time = %04d-%02d-%02d %02d:%02d:%02d\n", rtc.getYear(), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());

  if(!mqttClient.connect(broker, port)) {
      DEBUG_PRINTF("Error Connecting to mqtt broker\n");
  } else {
    char topic[MAX_TOPIC_LENGTH];
    snprintf(topic, sizeof(topic), "%s/%s/boot", topicRoot, hostname);
    DEBUG_PRINTF("Topic = %s\n", topic);

    char msg[128];
    snprintf(msg, sizeof(msg), 
      "{ \"boot\": \"%04d-%02d-%02dT%02d:%02d:%02d\", \"IP\": \"%d.%d.%d.%d\", \"rssi\": %d, \"reset_reason\": %0x }", 
        rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds(),
        WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3],
        WiFi.RSSI(),
        PM->RCAUSE.reg
    ); 
    DEBUG_PRINTF("Sending msg = %s\n", msg);
    mqttClient.beginMessage(topic);
    mqttClient.print(msg);
    mqttClient.endMessage();
    delay(mqttTxDelay);
    DEBUG_PRINTF("Sent message\n");
  }

  mqttClient.stop();
   
  WiFi.disconnect();
  WiFi.end(); 
}


void readMeasurements() {
    // Read the humidity
  humidityCharacteristic = sensorPeripheral.characteristic("2A6F");
  if (humidityCharacteristic) {
    uint16_t humidityRawValue;
    humidityCharacteristic.readValue(humidityRawValue); 
    currentHumidity = humidityRawValue / 100.0;
    DEBUG_PRINTF("Humidity = %.2f %%\n", currentHumidity);
  } else {
    DEBUG_PRINTF("Cannot find humidity characteristic\n");
  }

  // Read the temperature
  temperatureCharacteristic = sensorPeripheral.characteristic("2A6E");
  if (temperatureCharacteristic) {
    int16_t temperatureRawValue;
    temperatureCharacteristic.readValue(temperatureRawValue);
    currentTemperature = temperatureRawValue / 100.0;
    DEBUG_PRINTF("Temperature = %.2f C\n", currentTemperature);
  } else {
    DEBUG_PRINTF("Cannot find temperature characteristic\n");
  }

  // Read the pressure
  pressureCharacteristic = sensorPeripheral.characteristic("2A6D");
  if (pressureCharacteristic) {
    uint32_t pressureRawValue;
    pressureCharacteristic.readValue(pressureRawValue);
    currentPressure = pressureRawValue * 0.00001450; // convert to PSI
    DEBUG_PRINTF("Pressure = %.2f PSI\n", currentPressure);
  } else {
    DEBUG_PRINTF("Cannot find pressure characteristic\n");
  }

  // Read the location
  locationNameCharacteristic = sensorPeripheral.characteristic("2AB5");
  if (locationNameCharacteristic) {
    char buff[255];
    memset(buff, 0, sizeof(buff));    // zero out the buffer.
    locationNameCharacteristic.readValue(buff, sizeof(buff));
    currentLocationName = buff;
    DEBUG_PRINTF("Location Name = %s\n", currentLocationName.c_str());
  } else {
    DEBUG_PRINTF("Cannot find locationName characteristic\n");
  }
}


bool sendMeasurementsToMQTT() {
  // We have a measurement so connect to WiFi and send it to the MQTT broker.
  bool status = false;

  DEBUG_PRINTF("Attempting to send measurement\n");

  if (WiFi.begin(ssid, pass) == WL_CONNECTED) {

    DEBUG_PRINTF("Attempting to connect to the MQTT broker: %s\n", broker);
  
    mqttClient.setConnectionTimeout(4000);
    if (!mqttClient.connect(broker, port)) {
      DEBUG_PRINTF("MQTT connection failed! Error code = %d\n", mqttClient.connectError());
    } else {
      DEBUG_PRINTF("You're connected to the MQTT broker!\n");

      char topic[MAX_TOPIC_LENGTH];

      snprintf(topic, sizeof(topic), "%s/%s/environment", topicRoot, connectedDeviceName.c_str());

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
    status = true;
  } 
  else {
    DEBUG_PRINTF("Error/timeout connecting to WiFi: %d\n", WiFi.reasonCode());
  }

  DEBUG_PRINTF("Disconnecting from Wifi\n");
  WiFi.disconnect();
  WiFi.end();

  return(status);
}

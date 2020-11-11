/**
 * Read values from a Nano 33 BLE Sense board and send them to an mqtt broker
 * via WiFi.
 *
 * Note as we cannot simultaneously connect the Wifi and BLE we will get a
 * measurement and then stop the BLE connection and connect to WiFi.
 */

#include <Adafruit_SleepyDog.h>
#include <Arduino.h>
#include <ArduinoBLE.h>
#include <ArduinoMqttClient.h>
#include <RTCZero.h>
#include <WiFiNINA.h>
#include <time.h>

#include "ConfigService.h"
#include "PagingOLEDDisplay.h"
#include "EnvironmentSensor.h"

#define _DEBUG_
#include "Debug.h"

#undef max
#undef min
#include <map>

#define DEFAULT_HOSTNAME "iot_central_001"
#define DEFAULT_MQTTBROKER "mqtt.bb.tjpetz.com"
#define DEFAULT_MQTTROOTTOPIC "tjpetz.com/sensor"
#define DEFAULT_SAMPLEINTERVAL 60

// We will not attempt connections to peripherals with a less then -90 dBm RSSI.
#define RSSI_LIMIT -90

// Configuration settings managed by BLE and Flash
ConfigService
    config(DEFAULT_HOSTNAME, DEFAULT_MQTTBROKER, DEFAULT_MQTTROOTTOPIC,
           DEFAULT_SAMPLEINTERVAL); // Config settings with sensible defaults

WiFiClient wifiClient;             // Our wifi client
MqttClient mqttClient(wifiClient); // Our MQTT client
const int mqttTxDelay =
    2500; // Time (mS) between the last call to mqtt and turning off the wifi.
const int port = 1883;
const unsigned int MAX_TOPIC_LENGTH = 255;
byte macAddress[6]; // MAC address of the Wifi which we'll use in reporting

RTCZero rtc; // Real Time Clock so we can time stamp data

const int maxTries = 10; // Retry counters
int retryCounter = 0;    // counter to keep track of failed connections
bool success = false;

// State machine states
typedef enum FSMStates {
  initializing,
  start_scan,
  scanning,
  reading_measurements,
  sending_measurements,
  idle,
  restart
} FSMState_t;

char debugStateNames[7][32] = {
    "init", // Initial state, wait here until configured by BLE.
    "str_scn", "scan", "rd_msr", "snd_msr", "idle", "rstrt"};

FSMState_t currentState = initializing;
FSMState_t nextState = initializing;
FSMState_t previousState = initializing;
FSMState_t configurationPriorState = initializing;

const int watchdogTimeout = 16384;    // max timeout is 16.384 S
unsigned long idleTime;               // sleep time in mS, computed from the configuration      

unsigned long now = 0;
unsigned long lastMeasureTime = 0;
unsigned long scanStartTime = 0;
unsigned long maxScanTime = 30000; // Time to scan before going idle

unsigned long startDelayTime = 0;

/** @brief 4 line OLED display for messages */
PagingOLEDDisplay oledDisplay(128, 32, 4, 2);

/** @brief a map of the sensors discovered in each scan period. */
std::map<String, EnvironmentSensor> discoveredSensors;

/** @brief Initialize the RTC by calling NTP and setting the initial time in the
 * RTC */
void initializeRTC() {
  int maxTries = 10;
  int tries = 0;

  DEBUG_PRINTF("Initializing the RTC via NTP\n");
  DEBUG_PRINTF("Starting WiFi for NTP Connection\n");

  while ((WiFi.begin(config.ssid, config.wifiPassword) != WL_CONNECTED) &&
         (tries < maxTries)) {
    DEBUG_PRINTF("initializeRTC - Connection failed, reason = %d.\n",
                 WiFi.reasonCode());
    tries++;
    WiFi.disconnect();
    WiFi.end();
    Watchdog.reset();
    delay(1500 * tries);
  }

  if (WiFi.status() != WL_CONNECTED) {
    DEBUG_PRINTF("Failed to connect to wifi after %d tries.  Rebooting!\n",
                 tries);
    Serial.flush();
    delay(3000);
    NVIC_SystemReset();
  }

  Watchdog.reset();
  DEBUG_PRINTF("WiFi Connected, address = %d.%d.%d.%d\n", WiFi.localIP()[0],
               WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);

  time_t ntpTime = WiFi.getTime();

  // try a few times to get the time, it takes a moment to reach the NTP
  // servers.
  while (ntpTime == 0 && tries < maxTries) {
    ntpTime = WiFi.getTime();
    tries++;
    delay(500 * (tries + 1));
  }

  Watchdog.reset();
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

  struct tm *t =
      gmtime(&ntpTime); // convert Unix epoch time to tm struct format
  DEBUG_PRINTF("NTP Time = %04d-%02d-%02d %02d:%02d:%02d\n", t->tm_year + 1900,
               t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);

  Watchdog.reset();
  // Setup the RTC
  rtc.begin();
  rtc.setEpoch(ntpTime);
  DEBUG_PRINTF("RTC Time = %04d-%02d-%02d %02d:%02d:%02d\n", rtc.getYear(),
               rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(),
               rtc.getSeconds());

  Watchdog.reset();
  if (!mqttClient.connect(config.mqttBroker, port)) {
    DEBUG_PRINTF("Error Connecting to mqtt broker\n");
  } else {
    char topic[MAX_TOPIC_LENGTH];
    snprintf(topic, sizeof(topic), "%s/%s/boot", config.topicRoot,
             config.hostName);
    DEBUG_PRINTF("Topic = %s\n", topic);

    char msg[128];
    snprintf(msg, sizeof(msg),
             "{ \"boot\": \"%04d-%02d-%02dT%02d:%02d:%02d\", \"IP\": "
             "\"%d.%d.%d.%d\", \"rssi\": %ld, \"reset_reason\": %0x }",
             rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(), rtc.getHours(),
             rtc.getMinutes(), rtc.getSeconds(), WiFi.localIP()[0],
             WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3],
             WiFi.RSSI(), PM->RCAUSE.reg);
    DEBUG_PRINTF("Sending msg = %s\n", msg);
    mqttClient.beginMessage(topic);
    mqttClient.print(msg);
    mqttClient.endMessage();
    delay(mqttTxDelay);
    DEBUG_PRINTF("Sent message\n");
  }
  Watchdog.reset();

  mqttClient.stop();

  WiFi.disconnect();
  WiFi.end();
}

/** @brief send the measurements via mqtt
 *  @return true if success */
bool sendMeasurementsToMQTT(float temperature, float humidity, float pressure, 
  const char* name, const char *location) {
  
  bool status = false;

  DEBUG_PRINTF("Attempting to send measurement\n");

  if (WiFi.begin(config.ssid, config.wifiPassword) == WL_CONNECTED) {

    DEBUG_PRINTF("Attempting to connect to the MQTT broker: %s\n",
                 config.mqttBroker);

    // mqttClient.setConnectionTimeout(4000);
    Watchdog.reset();
    if (!mqttClient.connect(config.mqttBroker, port)) {
      DEBUG_PRINTF("MQTT connection failed! Error code = %d\n",
                   mqttClient.connectError());
    } else {
      DEBUG_PRINTF("You're connected to the MQTT broker!\n");

      char topic[MAX_TOPIC_LENGTH];

      snprintf(topic, sizeof(topic), "%s/%s/environment", config.topicRoot,
               name);

      DEBUG_PRINTF("Topic = %s\n", topic);
      mqttClient.beginMessage(topic);
      char dateTime[32];
      snprintf(dateTime, sizeof(dateTime), "%04d-%02d-%02dT%02d:%02d:%02d",
               rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(),
               rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
      DEBUG_PRINTF("Sample Time = %s\n", dateTime);
      char msg[255];
      snprintf(
          msg, sizeof(msg),
          "{ \"sensor\": \"%s\", \"location\": \"%s\", \"sampleTime\": \"%s\", "
          "\"temperature\": %.2f, \"humidity\": %.2f, \"pressure\": %.2f }",
          name, location, dateTime, temperature, humidity, pressure);
      DEBUG_PRINTF("Message = %s\n", msg);
      mqttClient.print(msg);
      mqttClient.endMessage();
      delay(mqttTxDelay);
    }
    mqttClient.stop();
    status = true;
  } else {
    DEBUG_PRINTF("Error/timeout connecting to WiFi: %d\n", WiFi.reasonCode());
  }

  DEBUG_PRINTF("Disconnecting from Wifi\n");
  WiFi.disconnect();
  WiFi.end();

  return status;
}

void onCentralConnected(BLEDevice central) {
  DEBUG_PRINTF("Connection from: %s, rssi = %d, at %lu\n",
               central.address().c_str(), central.rssi(), millis());
  DEBUG_PRINTF("  BLE Central = %s\n", BLE.central().address().c_str());
}

void onCentralDisconnected(BLEDevice central) {
  DEBUG_PRINTF("Disconnected from: %s, at %lu\n", central.address().c_str(),
               millis());
  DEBUG_PRINTF("  BLE Central = %s\n", BLE.central().address().c_str());
}

void setup() {

  Serial.begin(115200);
  delay(3000); // Give serial a moment to start

  DEBUG_PRINTF("RESET Register = 0x%0x\n", PM->RCAUSE.reg);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Watchdog.enable(watchdogTimeout);

  if (!oledDisplay.begin()) {
    DEBUG_PRINTF("Error initializing OLED display\n");
  }

  DEBUG_PRINTF("Starting BLE\n");
  while (!BLE.begin()) {
    DEBUG_PRINTF("Error starting BLE\n");
    delay(1000);
  }

  BLE.setLocalName(config.hostName);
  config.begin();
  BLE.setAdvertisedService(config.getConfigService());
  BLE.advertise();

  now = millis();
  lastMeasureTime = now;

  currentState = initializing;
  previousState = initializing;

  digitalWrite(LED_BUILTIN, LOW);

  Watchdog.reset();
}

void loop() {

  switch (currentState) {

  case initializing:
    // For for initialization
    if (!config.isInitialized) {
      BLE.poll();
    } else {
      // Complete the initialization
      DEBUG_PRINTF("Completing the initialization\n");
      BLE.disconnect();
      BLE.end();
      delay(1750);
      Watchdog.reset();
      initializeRTC();
      Watchdog.reset();
      BLE.begin();
      BLE.addService(config.getConfigService());
      BLE.setAdvertisedService(config.getConfigService());
      // After initialization we advertize less frequently to save power
      BLE.setAdvertisingInterval(3200);     // Advertize ever 2000 ms (1000 / 0.625)
      BLE.advertise();

      BLE.setEventHandler(BLEConnected, onCentralConnected);
      BLE.setEventHandler(BLEDisconnected, onCentralDisconnected);

      nextState = start_scan;
    }
    break;

  case start_scan:

    discoveredSensors.clear();    // empty the list from any previous scan.
    BLE.scanForUuid(EnvironmentSensor::environmentServiceUUID);
    scanStartTime = millis();
    nextState = scanning;
    break;

  case scanning:

    digitalWrite(LED_BUILTIN, HIGH);

    BLE.poll();

    // Add any peripherals we find to our map of discovered devices
    if (BLEDevice peripheral = BLE.available()) {
      DEBUG_PRINTF("while scanning found %s (%d dBm)\n", peripheral.address().c_str(), peripheral.rssi());
      if (peripheral.rssi() >= RSSI_LIMIT) {
        discoveredSensors.insert(std::pair<String, EnvironmentSensor>(peripheral.address(), EnvironmentSensor(peripheral)));
      }
    }

    if (millis() - scanStartTime <= maxScanTime) {
      nextState = scanning;
    } else if (discoveredSensors.size() > 0) {
      DEBUG_PRINTF("Scan finished and found %d sensors\n", discoveredSensors.size());
      digitalWrite(LED_BUILTIN, LOW);
      BLE.stopScan();
      nextState = reading_measurements;
    } else {  // move to idle if we didn't find any sensors.
      digitalWrite(LED_BUILTIN, LOW);
      BLE.stopScan();
      nextState = idle;
    }

    break;

  case reading_measurements:

    DEBUG_PRINTF("Reading the sensors...\n");
    // loop through the sensor and read their values
    for (auto s = discoveredSensors.begin(); s != discoveredSensors.end(); s++) {
      DEBUG_PRINTF(" reading sensor %s\n", s->first.c_str());
      bool success = s->second.readSensor();
      s->second.disconnect();
      if (!success) {
        // we failed to read this sensor, so remove it from this scan
        discoveredSensors.erase(s);
      }
    }

    nextState = sending_measurements;
    break;

  case sending_measurements:

    // If a central device is connected do not send the measurement
    if (BLE.central().address() != "00:00:00:00:00:00") {
      DEBUG_PRINTF("A central device is connected so we're skipping sending "
                   "this measurements.\n");
      nextState = idle; // force a change to idle
    } else {

      if (discoveredSensors.size() == 0) {
        // We don't have any measurements that we successfully received so just go to idle
        DEBUG_PRINTF("No Sensors with measurements to send.\n")
        nextState = idle;
        break;
      }

      DEBUG_PRINTF("Sending Measurements...\n");

      BLE.disconnect();
      BLE.end();

      delay(1750); // Delay to allow the Nina radio module to reset

      // Send each sensor's reading
      for (auto s = discoveredSensors.begin(); s != discoveredSensors.end(); s++) {
        DEBUG_PRINTF("Sending measurement for sensor %s\n", s->first.c_str());
        Watchdog.reset();
        retryCounter = 0;
        success = sendMeasurementsToMQTT(s->second.temperature(), s->second.humidity(),
          s->second.pressure(), s->second.name(), s->second.location());

        while (!success && (retryCounter < maxTries)) {
          Watchdog.reset();
          delay(500 * (retryCounter + 1));
          ++retryCounter;
          success = sendMeasurementsToMQTT(s->second.temperature(), s->second.humidity(),
          s->second.pressure(), s->second.name(), s->second.location());
        }

        if (success) {
          nextState = idle;
        } else {
          nextState = restart;
        }

      }
    
      BLE.begin();
      BLE.addService(config.getConfigService());
      BLE.setAdvertisedService(config.getConfigService());
      BLE.advertise();

    }

    break;

  case idle:

    DEBUG_PRINTF("Entering idle.\n");

    // TODO: need to think about how we can go low power and still respond to BLE...
    //  and do we even want to respond to BLE during this time.
    startDelayTime = millis();
    idleTime = config.sampleInterval * 1000;
    DEBUG_PRINTF("Sleep for %lu mS starting at %lu mS \n", idleTime, startDelayTime);

    while ((millis() - startDelayTime) < idleTime) {
      // We need to sleep in short intervals so we can reset the watchdog timer
      Watchdog.reset();
      unsigned long remainingTime = idleTime - (millis() - startDelayTime);
      DEBUG_PRINTF("Remaining sleep time = %lu mS\n", remainingTime);
      if (remainingTime >= (watchdogTimeout - 1000)) {
        BLE.poll(watchdogTimeout - 1000);
      } else {
        BLE.poll(remainingTime);
      }
      Watchdog.reset();
    }

    DEBUG_PRINTF("Waking from idle. Slept for %lu mS.  Restarting BLE.\n",
                 millis() - startDelayTime);

    nextState = start_scan;
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

  oledDisplay.printf(0, "St: %s -> %s", debugStateNames[currentState],
                     debugStateNames[nextState]);
  oledDisplay.printf(1, "%04d-%02d-%02d %02d:%02d", rtc.getYear() + 2000,
                     rtc.getMonth(), rtc.getDay(), rtc.getHours(),
                     rtc.getMinutes());
  oledDisplay.printf(2, "Host: %s", config.hostName);
  oledDisplay.printf(3, "Topic: %s", config.topicRoot);
  oledDisplay.displayCurrentPage();

  previousState = currentState;
  currentState = nextState;

  Watchdog.reset();
}

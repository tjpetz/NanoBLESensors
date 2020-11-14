/*
 * Experiment with BLE.  In this sketch will simply publish the various sensor
 * values available on the Nano 33 BLE Sense.  Note, this is not secure as we
 * simply allow any client to register and receive the values.
 *
 * Future will look to add security so that clients must register.  Also we'll
 * look to add low power to the code so that it goes into deep sleep and only
 * periodically wakes up.
 *
 * History:
 *  8 Mar 2019 - Adding capabilities to configure the device via BLE
 *  8 Dec 2019 19:00Z - Added the temperature as a second characteristic.
 */

#include <Arduino.h>
#include <ArduinoBLE.h>

#include "BLEConfigure.h"
#include "BLEEnvironmentMonitor.h"
#include "BatteryMonitor.h"
#include "ColorLED.h"
#define _DEBUG_
#include "Debug.h"
#include "PagingOLEDDisplay.h"
#include "Watchdog.h"

long previousMillis = 0; // last time the environment was checked (mS)

// save the name of the sensor so we can detect a change in the disconnect
// hander.
String oldSensorName;

// WDT will reset if we get hung in a loop
const uint32_t watchdogTimeout = 60;

// Event Handlers
void on_BLECentralConnected(BLEDevice central);
void on_BLECentralDisconnected(BLEDevice central);

// counters for checking if we've lost connections
uint32_t connPrevTime = 0;
const int maxZeroRSSI =
    10; // this many times of zero measurement before reseting
int currentZeroRSSICount = 0;

String lastCentralAddr = "";
int lastCentralRSSI = 0;

BatteryMonitor batteryMonitor(A0, D2); // BLE battery service
BLEConfigure configuration;            // BLE configuration service
BLEEnvironmentMonitor environmentMonitor(configuration.location); // BLE environment service

PagingOLEDDisplay oledDisplay(128, 64, 7, D3); // the OLED display
RGBled led; // onboard LEDs to signal humidity in range

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  initializeWDT(watchdogTimeout);

  Serial.begin(115200);
  delay(3000); // wait for serial to initialize but don't fail if there is no
               // terminal.

  if (!oledDisplay.begin()) {
    DEBUG_PRINTF("Error initializing OLED display\n");
  }

  while (!BLE.begin()) {
    DEBUG_PRINTF("Error starting BLE\n");
    delay(1000);
  }

  resetWDT();

  configuration.debug_print_configuration();
  
  DEBUG_PRINTF("sensorName = %s\n", configuration.sensorName);
  DEBUG_PRINTF("sensorLocation = %s\n", configuration.location);
  DEBUG_PRINTF("humidityGreenLimit = %d\n", configuration.humidityGreenLimit);
  DEBUG_PRINTF("humidityAmberLimit = %d\n", configuration.humidityAmberLimit);
  DEBUG_PRINTF("NRF_NVMC->READY = 0x%04lx\n", NRF_NVMC->READY);

  // Configure the BLE settings
  BLE.setLocalName(configuration.sensorName);
  oldSensorName = configuration.sensorName;

  // Start our BLE services.
  configuration.begin();
  batteryMonitor.begin();
  environmentMonitor.begin();

  BLE.addService(configuration.getService());
  BLE.addService(environmentMonitor.getService());
  BLE.addService(batteryMonitor.getService());

  // Add the service callbacks just so we can provide some debug messages
  BLE.setEventHandler(BLEConnected, on_BLECentralConnected);
  BLE.setEventHandler(BLEDisconnected, on_BLECentralDisconnected);

  BLE.setAppearance(5696); // Generic environmental sensor
  BLE.setAdvertisedService(environmentMonitor.getService());
  BLE.setAdvertisingInterval(1600); // Advertise every 250 mS = (400 * 0.625ms)

  NRF_RADIO->TXPOWER = 8;
  
  BLE.advertise();

  DEBUG_PRINTF("BLE initialized, waiting for connections...\n");

  resetWDT();

  // counters
  previousMillis = millis();
  connPrevTime = millis();

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // All the BLE connections will be handled via event handlers
  // Our loop must only monitor the sensors and update the characteristics
  // when they change

  // We don't want to read the sensors too frequentlt to save power
  long currentMillis = millis();
  if (currentMillis - previousMillis >= 60000) {
    previousMillis = currentMillis;

    environmentMonitor.measure();

    if (environmentMonitor.humidity() <= configuration.humidityGreenLimit) {
      led.setColor(0, 25, 0); // GREEN);
    } else if (environmentMonitor.humidity() <=
               configuration.humidityAmberLimit) {
      led.setColor(25, 25, 0); // YELLOW);
    } else {
      led.setColor(25, 0, 0); // RED);
    }

    // Update the battery Voltage
    float currentVoltage = batteryMonitor.measureVoltage();

    oledDisplay.printf(0, "Temperature: %0.2f C",
                       environmentMonitor.temperature() / 100.0);
    oledDisplay.printf(1, "Humidity: %0.2f % RH",
                       environmentMonitor.humidity() / 100.0);
    oledDisplay.printf(2, "Pressure: %0.2f inHg",
                       environmentMonitor.pressure() / 10.0 / 3386.389);
    oledDisplay.printf(3, "Name: %s", configuration.sensorName);
    oledDisplay.printf(4, "Location: %s", configuration.location);
    oledDisplay.printf(5, "Ad: %s", lastCentralAddr.c_str());
    oledDisplay.printf(6, "Battery: %0.2f v", currentVoltage);
    oledDisplay.displayCurrentPage();
  }

  BLE.poll();
  resetWDT();

  // BLE may remain connected if central goes out of range.  When this happens
  // we do not restart and readvertise, so we end up hanging.  Every second
  // check if we're connected and if so that the rssi is less than 0.  If after
  // 5 checks we still have 0 rssi assume the device is out of range and force
  // a disconnect and start adversing again.

  if (millis() - connPrevTime >= 5000) {
    connPrevTime = millis();

    BLEDevice central = BLE.central();

    if (central) {
      DEBUG_PRINTF("central RSSI = %d, zeroRSSIcount = %d\n", central.rssi(),
                   currentZeroRSSICount);
      if (central.connected()) {
        if (central.rssi() < 0) {
          // legit signal so reset the zero counters
          currentZeroRSSICount = 0;
        } else {
          currentZeroRSSICount++;
        }

        if (currentZeroRSSICount > maxZeroRSSI) {
          DEBUG_PRINTF("Zero RSSI count exceeded, forcing disconnect!\n");
          central.disconnect();
          BLE.advertise();
        }
      }
    }
  }
}

void on_BLECentralConnected(BLEDevice central) {
  DEBUG_PRINTF("Connection from: %s, rssi = %d, at %lu\n",
               central.address().c_str(), central.rssi(), millis());
  lastCentralAddr = central.address();
  lastCentralRSSI = central.rssi();
  digitalWrite(LED_BUILTIN, HIGH);
  BLE.advertise();
}

void on_BLECentralDisconnected(BLEDevice central) {
  DEBUG_PRINTF("Disconnected from: %s, at %lu\n", central.address().c_str(),
               millis());
  digitalWrite(LED_BUILTIN, LOW);
  if (oldSensorName != configuration.sensorName) {
    DEBUG_PRINTF("Setting new sensor name = %s\n", configuration.sensorName);
    // the sensor name has changed, we need to update the name while the client
    // is Disconnected and before we restart advertising.
    BLE.setLocalName(configuration.sensorName);
    oldSensorName = configuration.sensorName;
  }
  BLE.advertise(); // Resume advertising.
}

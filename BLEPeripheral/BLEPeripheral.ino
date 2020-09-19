/*
 * Experiment with BLE.  In this sketch will simply publish the various sensor values
 * available on the Nano 33 BLE Sense.  Note, this is not secure as we simply allow any
 * client to register and receive the values.
 * 
 * Future will look to add security so that clients must register.  Also we'll look
 * to add low power to the code so that it goes into deep sleep and only periodically
 * wakes up.
 * 
 * History:
 *  8 Mar 2019 - Adding capabilities to configure the device via BLE
 *  8 Dec 2019 19:00Z - Added the temperature as a second characteristic.
 */

#include <ArduinoBLE.h>
#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>
#define NO_ADAFRUIT_SSD1306_COLOR_COMPATIBILITY
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define _DEBUG_

#include "Debug.h"
#include "ColorLED.h"
#include "BLEConfigure.h"
#include "Watchdog.h"

// These globals are configurable over BLE, we choose reasonable defaults
String sensorName;
String sensorLocation;
uint16_t humidityGreenLimit;     // Green for less than or equal to
uint16_t humidityAmberLimit;     // Amber for greater than green and less than or equal to

// Default configuration information, this is stored in flash memory
extern const flash_config_t flashConfig __attribute__ ((aligned(0x1000))) =
{
    "NanoBLESense",         // sensorName
    "Location",             // location
    4500,                   // green humidity limit
    6500,                   // amber humidity limit
    ""                      // lock password is empty by default
};

// BLE Environment Service
BLEService environmentService("181A");

// BLE Location Name
BLEStringCharacteristic locationNameCharacteristic("2AB5", BLERead | BLENotify, 128);

// BLE Humidity and Temperarture Characterists
BLEUnsignedIntCharacteristic humidityCharacteristic("2A6F", BLERead | BLENotify);  // standard 16-bit UUID and remote client may read.
BLEIntCharacteristic temperatureCharacteristic("2A6E", BLERead | BLENotify | BLEBroadcast);  // standard UUID for temp characteristic in C 0.01
BLEUnsignedLongCharacteristic pressureCharacteristic("2A6D", BLERead | BLENotify); //standard UUID for pressure characteristinc in Pa 0.1

// Most recent measurements
uint16_t oldHumidity = 0;   // last humidity level
int16_t oldTemperature = 0; // last temperature
uint32_t oldPressure = 0;   // last pressure
long previousMillis = 0;    // last time the environment was checked (mS)

// save the name of the sensor so we can detect a change in the disconnect hander.
String oldSensorName;

// Use the onboard LEDs to signal humidity in range
rgbLED led;

// WDT will reset if we get hung in a loop
const uint32_t watchdogTimeout = 60;

// Event Handlers
void on_BLECentralConnected(BLEDevice central);
void on_BLECentralDisconnected(BLEDevice central);

// counters for checking if we've lost connections
uint32_t connPrevTime = 0;
const int maxZeroRSSI = 10;       // this many times of zero measurement before reseting
int currentZeroRSSICount = 0;

// the oled display
Adafruit_SSD1306 display(128, 64);

String lastCentralAddr = "";
int lastCentralRSSI = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  initializeWDT(watchdogTimeout);

  Serial.begin(115200);
  delay(1500);         // wait for serial to initialize but don't fail if there is no terminal.

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    DEBUG_PRINTF("Error initializing OLED display\n");
  }
  
  while (!HTS.begin()) {
    DEBUG_PRINTF("Error initializing HTS sensor\n");
    delay(1000);
  }

  while (!BARO.begin()) {
    DEBUG_PRINTF("Error intializing the LPS22HB sensor");
    delay(1000);
  }

  while (!BLE.begin()) {
    DEBUG_PRINTF("Error starting BLE\n");
    delay(1000);
  }

  resetWDT();

  // Initialize the configuration from flash settings
  sensorName = flashConfig.sensor_name;
  sensorLocation = flashConfig.location;
  humidityGreenLimit = flashConfig.humidityGreenLimit;
  humidityAmberLimit = flashConfig.humidityAmberLimit;

  DEBUG_PRINTF("sensorName = %s\n", sensorName.c_str());
  DEBUG_PRINTF("sensorLocation = %s\n", sensorLocation.c_str());
  DEBUG_PRINTF("humidityGreenLimit = %d\n", humidityGreenLimit);
  DEBUG_PRINTF("humidityAmberLimit = %d\n", humidityAmberLimit);
  DEBUG_PRINTF("flash address = 0x%08x\n", (unsigned int)&flashConfig);
  DEBUG_PRINTF("NRF_NVMC->READY = 0x%04x\n", NRF_NVMC->READY);
    
  // Configure the BLE settings
  BLE.setLocalName(sensorName.c_str());
  oldSensorName = sensorName;
  environmentService.addCharacteristic(humidityCharacteristic);
  environmentService.addCharacteristic(temperatureCharacteristic);
  environmentService.addCharacteristic(pressureCharacteristic);
  environmentService.addCharacteristic(locationNameCharacteristic);

  // Set the initial values
  humidityCharacteristic.writeValue((uint16_t)(HTS.readHumidity() * 100));
  temperatureCharacteristic.writeValue((int16_t)(HTS.readTemperature() * 100));
  pressureCharacteristic.writeValue((uint32_t)(BARO.readPressure() * 10000));
  locationNameCharacteristic.writeValue(sensorLocation);

  // broadcast the temperature in the advertisement
  temperatureCharacteristic.broadcast();
  
  config_configService();
  
  // Add the service callbacks just so we can provide some debug messages
  BLE.setEventHandler(BLEConnected, on_BLECentralConnected);
  BLE.setEventHandler(BLEDisconnected, on_BLECentralDisconnected);

  BLE.addService(configService);
  BLE.addService(environmentService);

  BLE.setAppearance(5696);      // Generic environmental sensor
  BLE.setAdvertisedService(environmentService);
  BLE.setAdvertisingInterval(400);    // Adversive every 250 mS = (400 * 0.625ms)
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
  
  uint16_t currentHumidity = 0;
  int16_t currentTemperature = 0;
  uint32_t currentPressure = 0;

  // We don't want to read the sensors at a greater frequency than 1Hz
  long currentMillis = millis();
  if (currentMillis - previousMillis >= 1000) {
      previousMillis = currentMillis;

    // Update the humidity reading
    currentHumidity = (uint16_t)(HTS.readHumidity() * 100);
    if (currentHumidity != oldHumidity) {
      humidityCharacteristic.writeValue(currentHumidity);
      oldHumidity = currentHumidity;
      if (currentHumidity <= humidityGreenLimit) {
        led.setColor(0,25,0); // GREEN); 
      } else if (currentHumidity <= humidityAmberLimit) {
        led.setColor(25, 25, 0); // YELLOW); 
      } else {
        led.setColor(25, 0, 0); // RED); 
      }
    }
  
    // Update the temperature reading
    currentTemperature = (int16_t)(HTS.readTemperature() * 100);
    if (currentTemperature != oldTemperature) {
      temperatureCharacteristic.writeValue(currentTemperature);
      oldTemperature = currentTemperature;
    }
  
    // Update the pressure reading, reading is in kPa and we need to report in 0.1 pa
    currentPressure = (uint32_t)(BARO.readPressure() * 10000);
    if (currentPressure != oldPressure) {
      pressureCharacteristic.writeValue(currentPressure);
      oldPressure = currentPressure;
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0); display.print("Temperature: "); display.print(currentTemperature / 100.0); display.print(" C");
    display.setCursor(0, 11); display.print("Humidity: "); display.print(currentHumidity / 100.0); display.print(" % RH");
    display.setCursor(0, 22); display.print("Pressure: "); display.print(currentPressure / 10.0 / 3386.389); display.print(" inHg");
    display.setCursor(0, 33); display.print("Name: "); display.print(sensorName);
    display.setCursor(0, 44); display.print("Location: "); display.print(sensorLocation);
    display.setCursor(0, 55); display.print(lastCentralAddr); // display.print(" at "); display.print(lastCentralRSSI);
    display.display();
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
      DEBUG_PRINTF("central RSSI = %d, zeroRSSIcount = %d\n", central.rssi(), currentZeroRSSICount);
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
  DEBUG_PRINTF("Connection from: %s, rssi = %d, at %lu\n", central.address().c_str(), central.rssi(), millis());
  lastCentralAddr = central.address();
  lastCentralRSSI = central.rssi();
  digitalWrite(LED_BUILTIN, HIGH); 
  BLE.stopAdvertise(); // Don't advertise while connected.
}

void on_BLECentralDisconnected(BLEDevice central) {
  DEBUG_PRINTF("Disconnected from: %s, at %lu\n", central.address().c_str(), millis());
  digitalWrite(LED_BUILTIN, LOW);
  if (oldSensorName != sensorName) {
    DEBUG_PRINTF("Setting new sensor name = %s\n", sensorName.c_str());
    // the sensor name has changed, we need to update the name while the client is Disconnected
    // and before we restart advertising.
    BLE.setLocalName(sensorName.c_str());
    oldSensorName = sensorName;
  } 
  BLE.advertise();  // Resume advertising.
}

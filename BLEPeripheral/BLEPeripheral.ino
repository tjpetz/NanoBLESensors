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

#define _DEBUG_

#include "Debug.h"
#include "ColorLED.h"
#include "BLEConfigure.h"

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

// BLE Humidity and Temperarture Characterists
BLEUnsignedIntCharacteristic humidityCharacteristic("2A6F", BLERead | BLENotify);  // standard 16-bit UUID and remote client may read.
BLEIntCharacteristic temperatureCharacteristic("2A6E", BLERead | BLENotify | BLEBroadcast);  // standard UUID for temp characteristic in C 0.01
BLEUnsignedLongCharacteristic pressureCharacteristic("2A6D", BLERead | BLENotify); //standard UUID for pressure characteristinc in Pa 0.1

// Most recent measurements
uint16_t oldHumidity = 0;   // last humidity level
int16_t oldTemperature = 0; // last temperature
uint32_t oldPressure = 0;   // last pressure
long previousMillis = 0;    // last time the environment was checked (mS)

// Use the onboard LEDs to signal humidity in range
rgbLED led;

// WDT will reset if we get hung in a loop
const int wdt_timeout = 5;      // 5 sec timeout

// Event Handlers
void on_BLECentralConnected(BLEDevice central);
void on_BLECentralDisconnected(BLEDevice central);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.begin(115200);
  delay(3500);         // wait for serial to initialize but don't fail if there is no terminal.

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
  
  BLE.setConnectionInterval(0x0006, 0x0c80);    // 7.5 ms to 4 s
  
  // Configure the BLE settings
  BLE.setLocalName(sensorName.c_str());
  BLE.setDeviceName(sensorName.c_str());
  environmentService.addCharacteristic(humidityCharacteristic);
  environmentService.addCharacteristic(temperatureCharacteristic);
  environmentService.addCharacteristic(pressureCharacteristic);
  humidityCharacteristic.writeValue((uint16_t)(HTS.readHumidity() * 100));
  temperatureCharacteristic.writeValue((int16_t)(HTS.readTemperature() * 100));
  pressureCharacteristic.writeValue((uint32_t)(BARO.readPressure() * 10000));

  // broadcast the temperature in the advertisement
  temperatureCharacteristic.broadcast();
  
  config_configService();
  
  // Add the service callbacks just so we can provide some debug messages
  BLE.setEventHandler(BLEConnected, on_BLECentralConnected);
  BLE.setEventHandler(BLEDisconnected, on_BLECentralDisconnected);

  BLE.addService(configService);
  BLE.addService(environmentService);

  BLE.setAdvertisedService(environmentService);
  BLE.setAdvertisingInterval(315);    
  BLE.advertise();

  DEBUG_PRINTF("BLE initialized, waiting for connections...\n");

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
  }
  
  BLE.poll();
}

void on_BLECentralConnected(BLEDevice central) {
  DEBUG_PRINTF("Connection from: %s, rssi = %d, at %lu\n", central.address().c_str(), central.rssi(), millis());
  digitalWrite(LED_BUILTIN, HIGH); 
  BLE.advertise(); 
}

void on_BLECentralDisconnected(BLEDevice central) {
  DEBUG_PRINTF("Disconnected from: %s, at %lu\n", central.address().c_str(), millis());
  digitalWrite(LED_BUILTIN, LOW); 
  BLE.advertise(); 
}

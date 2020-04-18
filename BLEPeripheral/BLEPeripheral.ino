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
#include "ColorLED.h"

#define _DEBUG_
#include "Debug.h"

#include "BLEConfigure.h"

//AppConfiguration appConfig;

// BLE Environment Service
BLEService environmentService("181A");

// BLE Humidity and Temperarture Characterists
BLEUnsignedIntCharacteristic humidityCharacteristic("2A6F", BLERead);  // standard 16-bit UUID and remote client may read.
BLEIntCharacteristic temperatureCharacteristic("2A6E", BLERead);  // standard UUID for temp characteristic in C 0.01
BLEUnsignedLongCharacteristic pressureCharacteristic("2A6D", BLERead); //standard UUID for pressure characteristinc in Pa 0.1
uint16_t oldHumidity = 0;   // last humidity level
int16_t oldTemperature = 0; // last temperature
uint32_t oldPressure = 0;   // last pressure
long previousMillis = 0;    // last time the environment was checked (mS)

// Use the onboard LEDs to signal humidity in range
// Green will be <= 45%, Amber <= 60%, Red > 60%
uint16_t humidityGreenLimit = 4500;
uint16_t humidityAmberLimit = 6500;
rgbLED led;

// WDT will reset if we get hung in a loop
const int wdt_timeout = 5;      // 5 sec timeout


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.begin(230400);
  delay(1000);         // wait for serial to initialize but don't fail if there is no terminal.

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

  // Configure the BLE settings
  BLE.setLocalName("Nano33BLESense-test");
  BLE.setAdvertisedService(environmentService);
  environmentService.addCharacteristic(humidityCharacteristic);
  environmentService.addCharacteristic(temperatureCharacteristic);
  environmentService.addCharacteristic(pressureCharacteristic);
  BLE.addService(environmentService);
  humidityCharacteristic.writeValue((uint16_t)(HTS.readHumidity() * 100));
  temperatureCharacteristic.writeValue((int16_t)(HTS.readTemperature() * 100));
  pressureCharacteristic.writeValue((uint32_t)(BARO.readPressure() * 10000));

//  appConfiguration.config_configService();
  
  BLE.advertise();

  DEBUG_PRINTF("BLE initialized, waiting for connections...\n");

  // Configure the Watch Dog Timer
  NRF_WDT->CONFIG = 0x01;
  NRF_WDT->CRV = wdt_timeout * 32768 + 1;
  NRF_WDT->RREN = 0x01;
  NRF_WDT->TASKS_START = 1;
  
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  uint16_t currentHumidity = 0;
  int16_t currentTemperature = 0;
  uint32_t currentPressure = 0;

  // Reset the WDT
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
  
  // Wait for a central connection
  BLEDevice central = BLE.central();

  if(central) {
    digitalWrite(LED_BUILTIN, HIGH);
    DEBUG_PRINTF("Connected to central: %s\n", central.address().c_str());
    DEBUG_PRINTF(" RSSI: %d\n", central.rssi());
    DEBUG_PRINTF(" Local Name: %s\n", central.localName().c_str());  

    while (central.connected()) {
      long currentMillis = millis();
      if (currentMillis - previousMillis >= 1000) {
          previousMillis = currentMillis;
 
          // Update the humidity reading
          currentHumidity = (uint16_t)(HTS.readHumidity() * 100);
          if (currentHumidity != oldHumidity) {
            humidityCharacteristic.writeValue(currentHumidity);
            oldHumidity = currentHumidity;
            if (currentHumidity <= humidityGreenLimit) {
              led.setColor(GREEN); 
            } else if (currentHumidity <= humidityAmberLimit) {
              led.setColor(YELLOW); 
            } else {
              led.setColor(RED); 
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
    }
   DEBUG_PRINTF("Disconnect from central\n");
   digitalWrite(LED_BUILTIN, LOW);
  } else {
    DEBUG_PRINTF("No central connection\n");
  }

  delay(250);
}

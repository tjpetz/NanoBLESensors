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
 *  8 Dec 2019 19:00Z - Added the temperature as a second characteristic.
 */

#include <ArduinoBLE.h>
#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>
#include "ColorLED.h"

#define _DEBUG_
#ifdef _DEBUG_
#define MAX_DEBUG_BUFF    128
#define DEBUG_PRINTF(...) {char _buff[MAX_DEBUG_BUFF]; snprintf(_buff, MAX_DEBUG_BUFF, __VA_ARGS__); SerialUSB.print(_buff);}
#else
#define DEBUG_PRINTF(buff, fmt, ...)
#endif

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

uint16_t humidityGreenLimit = 4500;
uint16_t humidityAmberLimit = 6500;
rgbLED led;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Use the onboard LEDs to signal humidity in range
  // Green will be <= 45%, Amber <= 60%, Red > 60%
//  pinMode(LEDR, OUTPUT);
//  pinMode(LEDG, OUTPUT);
//  pinMode(LEDB, OUTPUT);
  // turn the LEDs off (Set high)
  
  Serial.begin(230400);
  delay(250);         // wait for serial to initialize but don't fail if there is no terminal.

  while (!HTS.begin()) {
    Serial.println("Error initializing HTS sensor");
    delay(1000);
  }

  while (!BARO.begin()) {
    DEBUG_PRINTF("Error intializing the LPS22HB sensor");
    delay(1000);
  }
  
  while (!BLE.begin()) {
    Serial.println("Error starting BLE");
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

  BLE.advertise();

  Serial.println("BLE initialized, waiting for connections...");
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  uint16_t currentHumidity = 0;
  int16_t currentTemperature = 0;
  uint32_t currentPressure = 0;
  
  // Wait for a central connection
  BLEDevice central = BLE.central();

  if(central) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("Connected to central: "); Serial.println(central.address());
    Serial.print(" RSSI: "); Serial.println(central.rssi());
    Serial.print(" Local Name: "); Serial.println(central.localName());  

    while (central.connected()) {
      long currentMillis = millis();
      if (currentMillis - previousMillis >= 1000) {
          previousMillis = currentMillis;
 
          // Update the humidity reading
          currentHumidity = (uint16_t)(HTS.readHumidity() * 100);
          if (currentHumidity != oldHumidity) {
            humidityCharacteristic.writeValue(currentHumidity);
            oldHumidity = currentHumidity;
            Serial.print("Humidity = "); Serial.println(currentHumidity);
            if (currentHumidity <= humidityGreenLimit) {
              led.setColor(GREEN); // digitalWrite(LEDR, HIGH), digitalWrite(LEDG, LOW); digitalWrite(LEDB, HIGH);
            } else if (currentHumidity <= humidityAmberLimit) {
              led.setColor(YELLOW); // digitalWrite(LEDR, LOW), digitalWrite(LEDG, LOW); digitalWrite(LEDB, HIGH);
            } else {
              led.setColor(RED); // digitalWrite(LEDR, LOW), digitalWrite(LEDG, HIGH); digitalWrite(LEDB, HIGH);
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
   Serial.println("Disconnect from central");
   digitalWrite(LED_BUILTIN, LOW);
  } else {
    DEBUG_PRINTF("No central connection\n");
  }

  delay(500);
}

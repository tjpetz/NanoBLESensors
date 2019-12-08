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

// BLE Environment Service
BLEService environmentService("181A");

// BLE Humidity and Temperarture Characterists
BLEUnsignedIntCharacteristic humidityCharacteristic("2A6F", BLERead | BLENotify);  // standard 16-bit UUID and remote client may read.
BLEIntCharacteristic temperatureCharacteristic("2A6E", BLERead | BLENotify);  // standard UUID for temp characteristic in C 0.01

uint16_t oldHumidity = 0;   // last humidity level
int16_t oldTemperature = 0; // last temperature
long previousMillis = 0;    // last time the environment was checked (mS)

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.begin(115200);
  delay(250);         // wait for serial to initialize but don't fail if there is no terminal.

  while (!HTS.begin()) {
    Serial.println("Error initializing HTS sensor");
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
  BLE.addService(environmentService);
  humidityCharacteristic.writeValue((uint16_t)(HTS.readHumidity() * 100));
  temperatureCharacteristic.writeValue((int16_t)(HTS.readTemperature() * 100));

  BLE.advertise();

  Serial.println("BLE initialized, waiting for connections...");
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  uint16_t currentHumidity = 0;
  int16_t currentTemperature = 0;
  
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
          }

          // Update the temperature reading
          currentTemperature = (int16_t)(HTS.readTemperature() * 100);
          if (currentTemperature != oldTemperature) {
            temperatureCharacteristic.writeValue(currentTemperature);
            oldTemperature = currentTemperature;
          }
      }
    }
   Serial.println("Disconnect from central");
   digitalWrite(LED_BUILTIN, LOW);
  }

  delay(250);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);
}

/*
 * Experiment with BLE.  In this sketch will simply publish the various sensor values
 * available one the Nano 33 BLE Sense.  Note, this is not secure as we simply allow any
 * client to register and receive the values.
 * 
 * Future will look to add security so that clients must register.  Also we'll look
 * to add low power to the code so that it goes into deep sleep and only periodically
 * wakes up.
 * 
 */

#include <ArduinoBLE.h>
#include <Arduino_HTS221.h>

// BLE Environment Service
BLEService environmentService("181A");

// BLE Humidity Characteristing
BLEUnsignedIntCharacteristic humidityCharacteristic("2A6F", BLERead | BLENotify);  // standard 16-bit UUID and remote client may read.

float oldHumidity = 0.0;    // last humidity level
long previousMillis = 0;    // last time the humidity was checked (mS)

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(19200);
  delay(500);         // wait for serial to initialize but don't fail if there is no terminal.

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
  BLE.addService(environmentService);
  humidityCharacteristic.writeValue(HTS.readHumidity());

  BLE.advertise();

  Serial.println("BLE initialized, waiting for connections...");
}

void loop() {
  float currentHumidity = 0.0;
  
  // Wait for a central connection
  BLEDevice central = BLE.central();

  if(central) {
    Serial.print("Connected to central: "); Serial.println(central.address());
    Serial.print(" RSSI: "); Serial.println(central.rssi());
    Serial.print(" Local Name: "); Serial.println(central.localName());  

    while (central.connected()) {
      long currentMillis = millis();
      if (currentMillis - previousMillis >= 1000) {
          previousMillis = currentMillis;
          currentHumidity = (unsigned int)(HTS.readHumidity() * 100);
          if (currentHumidity != oldHumidity) {
            humidityCharacteristic.writeValue(currentHumidity);
            oldHumidity = currentHumidity;
          }
      }
    }
   Serial.println("Disconnect from central");
  }

  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);
}

/*
 * Simple experiment to pair up with the Nano 33 BLE Sense to read the
 * sensor values from it via BLE.
 * 
 * History:
 *  8 Dec 2019 19:00Z - Added reading the temperature characteristic.
 */

#include <ArduinoBLE.h>

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.begin(115200);
  delay(250);     // Give serial a moment to start

  while (!BLE.begin()) {
    Serial.println("Error starting BLE");
    delay(1000);
  }

  Serial.println("Initiating scan...");
  BLE.scan();
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

  String deviceName = "Nano33BLESense-test";

  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  
  if (BLE.scanForName(deviceName)) {
    
    BLEDevice peripheral = BLE.available();

    if(peripheral) {
      Serial.println("=========================================================");
      Serial.print("Local Name: "); Serial.println(peripheral.localName());
      Serial.print("Address: "); Serial.println(peripheral.address());
      Serial.print("RSSI: "); Serial.println(peripheral.rssi());

      BLE.stopScan();
      
      // print the advertised service UUIDs, if present
      if (peripheral.hasAdvertisedServiceUuid()) {
        Serial.print("Advertised Service UUIDs: ");
        for (int i = 0; i < peripheral.advertisedServiceUuidCount(); i++) {
          Serial.print(peripheral.advertisedServiceUuid(i));
          Serial.print(",");
        }
        Serial.println();
      }

      if (peripheral.connect()) {
        Serial.println("Connected.");

        if (peripheral.discoverService("181a")) {
          Serial.println("Humidity Service found");

            // Read the humidity
            BLECharacteristic humidityCharacteristic = peripheral.characteristic("2A6F");
            if (humidityCharacteristic) {
              uint16_t humidityRawValue;
              humidityCharacteristic.readValue(humidityRawValue); 
              float humidity = humidityRawValue / 100.0;
              Serial.print("Humidity = "); Serial.print(humidity); Serial.println("%");
            } else {
              Serial.println("Cannot find humidity characteristic");
            }

            // Read the temperature
            BLECharacteristic temperatureCharacteristic = peripheral.characteristic("2A6E");
            if (temperatureCharacteristic) {
              int16_t temperatureRawValue;
              temperatureCharacteristic.readValue(temperatureRawValue);
              float temperature = temperatureRawValue / 100.0;
              Serial.print("Temperature = "); Serial.print(temperature); Serial.println("C");
            } else {
              Serial.println("Cannot find temperature characteristic");
            }
            
        } else {
          Serial.println("Humidity Service not found");
        }
        peripheral.disconnect();
      } else {
        Serial.println("Failed to connect.");
      }

      Serial.println("Restarting the scan");
      BLE.scan();
    }
  }

  delay(2000);
}

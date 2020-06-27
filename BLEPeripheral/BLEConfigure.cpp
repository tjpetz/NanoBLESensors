/*
 * BLEConfigure.ino
 * 
 * Author: thomas.j.petz@gsk.com
 * 
 * Allow the boards configuration information to be managed via BLE.
 *
 */
 
#include <ArduinoBLE.h>
#include "BLEConfigure.h"
#include "Watchdog.h"

#define _DEBUG_
#include "Debug.h"

extern String sensorName;
extern String sensorLocation;
extern uint16_t humidityGreenLimit;
extern uint16_t humidityAmberLimit;
extern BLEStringCharacteristic locationNameCharacteristic;

extern const flash_config_t flashConfig;

void config_configService();
void updateConfig();
void writeFlashPage(uint32_t *to, uint32_t *from, int len);
void hexDumpMemory(uint8_t *memStart, const unsigned int nbrOfBytes);

String configLockPassword;
bool configIsLocked;

BLEService configService("7f76c1b2-c592-4ace-8089-47bf14d07ced");
// Configurable settings for the application
BLEStringCharacteristic config_sensorName("641aff8f-217e-4ddb-aece-3053b128c27d", BLERead | BLEWrite, 128);
BLEDescriptor config_sensorNameDescriptor("2901", "Sensor Name");

BLEStringCharacteristic config_sensorLocation("52b8d6c4-cd20-4f51-bf71-ea0de788ebb4", BLERead | BLEWrite, 128);
BLEUnsignedIntCharacteristic config_humidityGreenLimit("ebf92cf8-2744-4df2-be70-e856fcaf01a7", BLERead | BLEWrite);
BLEUnsignedIntCharacteristic config_humidityAmberLimit("90441958-8975-4f62-aa13-08bdb86acd16", BLERead | BLEWrite);
// Configuration locking
BLEStringCharacteristic lockConfig("3ffb9262-18a2-4acf-918c-2f8932577c48", BLEWrite, 128);
BLEStringCharacteristic unlockConfig("2b1823e7-2d41-46f8-98d8-fd830ffdd68e", BLEWrite, 128);
BLEBoolCharacteristic config_configIsLocked("32b6a19f-ddac-45db-a1b5-8a7dbe9a76fc", BLERead);


void sensorName_Written(BLEDevice central, BLECharacteristic characteristic) {
  DEBUG_PRINTF("configService.sensorName written: %s\n", config_sensorName.value().c_str());
  if (!configIsLocked) {
    sensorName = config_sensorName.value();
  }
}

void sensorLocation_Written(BLEDevice central, BLECharacteristic characteristic) {
  DEBUG_PRINTF("configService.sensorLocation written: %s\n", config_sensorLocation.value().c_str());
  if (!configIsLocked) {
    sensorLocation = config_sensorLocation.value();
    locationNameCharacteristic.writeValue(sensorLocation);
  }
}

void humidityGreenLimit_Written(BLEDevice central, BLECharacteristic characteristic) {
  DEBUG_PRINTF("configService.humidityGreenLimit written: %d\n", config_humidityGreenLimit.value());
  if (!configIsLocked) {
    humidityGreenLimit = config_humidityGreenLimit.value();
  }
}

void humidityAmberLimit_Written(BLEDevice central, BLECharacteristic characteristic) {
  DEBUG_PRINTF("configService.humidityAmberLimit written: %d\n", config_humidityAmberLimit.value());
  if (!configIsLocked) {
    humidityAmberLimit = config_humidityAmberLimit.value();
  }
}

void lockConfig_Written(BLEDevice central, BLECharacteristic characteristic) {
  DEBUG_PRINTF("configService.lockConfig written - %s\n", lockConfig.value().c_str());
  if (!configIsLocked) {
    configLockPassword = lockConfig.value();
    configIsLocked = true;
    config_configIsLocked.writeValue(true);
    updateConfig();
  }
}

void unlockConfig_Written(BLEDevice central, BLECharacteristic characteristic) {
  DEBUG_PRINTF("configService.unlockConfig written - %s\n", unlockConfig.value().c_str());
  if (!configIsLocked) {
    if (unlockConfig.value() == configLockPassword) {
      configIsLocked = false;
      config_configIsLocked.writeValue(false);
    }
  }
}

void config_configService() {
  DEBUG_PRINTF("Configuring the configService\n");

  config_sensorName.addDescriptor(config_sensorNameDescriptor);
  configService.addCharacteristic(config_sensorName);
  config_sensorName.writeValue(sensorName);
  config_sensorName.setEventHandler(BLEWritten, sensorName_Written);

  configService.addCharacteristic(config_sensorLocation);
  config_sensorLocation.writeValue(sensorLocation);
  config_sensorLocation.setEventHandler(BLEWritten, sensorLocation_Written);

  configService.addCharacteristic(config_humidityGreenLimit);
  config_humidityGreenLimit.writeValue(humidityGreenLimit);
  config_humidityGreenLimit.setEventHandler(BLEWritten, humidityGreenLimit_Written);

  configService.addCharacteristic(config_humidityAmberLimit);
  config_humidityAmberLimit.writeValue(humidityAmberLimit);
  config_humidityAmberLimit.setEventHandler(BLEWritten, humidityAmberLimit_Written);

  configService.addCharacteristic(lockConfig);
  lockConfig.setEventHandler(BLEWritten, lockConfig_Written);

  configService.addCharacteristic(unlockConfig);
  unlockConfig.setEventHandler(BLEWritten, unlockConfig_Written);

  configService.addCharacteristic(config_configIsLocked);
  config_configIsLocked.writeValue(configIsLocked);
  
  DEBUG_PRINTF("configService configured!\n");
}

// Update the flash configuration
void updateConfig() {

  // To update the flash we make a copy of the values we want to write in
  // and then erase and write out the new values.
  DEBUG_PRINTF("Updating the flash config...\n");
  
  flash_config_t newFlash;
  memset(&newFlash, 0, sizeof(newFlash));      // zero the memory
  strcpy(newFlash.sensor_name, sensorName.c_str());
  strcpy(newFlash.location, sensorLocation.c_str());
  newFlash.humidityGreenLimit = humidityGreenLimit;
  newFlash.humidityAmberLimit = humidityAmberLimit;

  DEBUG_PRINTF("newFlash.sensor_name = %s\n", newFlash.sensor_name);
  DEBUG_PRINTF("newFlash.location = %s\n", newFlash.location);
  DEBUG_PRINTF("newFlash.humidityGreenLimit = %d\n", newFlash.humidityGreenLimit);
  DEBUG_PRINTF("newFlash.humidityAmberLimit = %d\n", newFlash.humidityAmberLimit);
  
  resetWDT();
  writeFlashPage((uint32_t *)&flashConfig, (uint32_t *)&newFlash, sizeof(newFlash));
  resetWDT();
    
  DEBUG_PRINTF("... flash updated!\n");
}

void writeFlashPage(uint32_t *to, uint32_t *from, int len) {

  uint32_t *tgt, *src;
  tgt = to; src = from;
  
  DEBUG_PRINTF("Before updating flash...");
  hexDumpMemory((uint8_t *)to, (unsigned int)len);
  hexDumpMemory((uint8_t *)from, (unsigned int)len);
  
  // Erase the page
  if(*(uint32_t *)NVMC_READY == 1) {
    DEBUG_PRINTF("Erasing...");
    *(uint32_t *)NVMC_CONFIG = 0x02;
    *(uint32_t *)NVMC_ERASEPAGE = (uint32_t)(tgt);
    while(*(uint32_t *)NVMC_READY == 0)
      delay(85);
    *(uint32_t *)NVMC_CONFIG = 0x00;
    DEBUG_PRINTF("...Erased\n");

    if(*(uint32_t *)NVMC_READY == 1) {
      DEBUG_PRINTF("Writing to = 0x%08x, from = 0x%08x\n", (uint32_t)tgt, (uint32_t)src);
      // write the values
      *(uint32_t *)NVMC_CONFIG = 0x01;
      for (int i = 0; i < len; i += 4) {
        *tgt++ = *src++;
        while (*(uint32_t *)NVMC_READY == 0)
          delayMicroseconds(41);
      }
      *(uint32_t *)NVMC_CONFIG = 0x00;
    } else {
      DEBUG_PRINTF("... NOT ready to write to flash! ...\n");
    }
    DEBUG_PRINTF("... Memory Flashed ...\n");
  } else {
    DEBUG_PRINTF("... Flash Not Ready to Erase! ...\n");
  }

  hexDumpMemory((uint8_t *)to, (unsigned int)len);

}

void hexDumpMemory(uint8_t *memStart, const unsigned int nbrOfBytes) {
  /* hex dump memory to the serial interface starting at memStart for nbrOfBytes */

  uint8_t *ptr;
  int bytesPerLine = 15;
  int bytesOnLine = 0;
  
  ptr = memStart;

  Serial.print("Memory dump of: "); Serial.println((unsigned long)(memStart), HEX);
  
  for (unsigned int i = 0; i < nbrOfBytes; i++) {
    if (bytesOnLine == 0) {
      Serial.print((unsigned long)(ptr+i), HEX);
      Serial.print(": ");
    }
    
    if (*(ptr+i) < 0x10)  // print a leading 0
      Serial.print("0");
    Serial.print(*(ptr+i), HEX); Serial.print(" ");
    
    if (bytesOnLine == bytesPerLine) {
      Serial.println(" ");
      bytesOnLine = 0;
    } else {
      bytesOnLine += 1;
    }
  }
  Serial.println("");
}

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

#define _DEBUG_
#include "Debug.h"

AppConfiguration::AppConfiguration(String name, String location, uint16_t greenLimit, uint16_t amberLimit):
  configService("7f76c1b2-c592-4ace-8089-47bf14d07ced"),
  config_sensorName("641aff8f-217e-4ddb-aece-3053b128c27d", BLERead | BLEWrite, 128),
  config_sensorLocation("52b8d6c4-cd20-4f51-bf71-ea0de788ebb4", BLERead | BLEWrite, 128),
  config_humidityGreenLimit("ebf92cf8-2744-4df2-be70-e856fcaf01a7", BLERead | BLEWrite),
  config_humidityAmberLimit("90441958-8975-4f62-aa13-08bdb86acd16", BLERead | BLEWrite),
  lockConfig("3ffb9262-18a2-4acf-918c-2f8932577c48", BLEWrite, 128),
  unlockConfig("2b1823e7-2d41-46f8-98d8-fd830ffdd68e", BLEWrite, 128),
  config_configIsLocked("32b6a19f-ddac-45db-a1b5-8a7dbe9a76fc", BLERead)
{
  sensorName = name;
  sensorLocation = location;
  humidityGreenLimit = greenLimit;
  humidityAmberLimit = amberLimit;
}
  
void AppConfiguration::sensorName_Written(BLEDevice central, BLECharacteristic characteristic) {
  DEBUG_PRINTF("configService.sensorName written\n");
  if (!configIsLocked) {
    sensorName = config_sensorName.value();
  }
}

void AppConfiguration::sensorLocation_Written(BLEDevice central, BLECharacteristic characteristic) {
  DEBUG_PRINTF("configService.sensorLocation written\n");
  if (!configIsLocked) {
    sensorLocation = config_sensorLocation.value();
  }
}

void AppConfiguration::humidityGreenLimit_Written(BLEDevice central, BLECharacteristic characteristic) {
  DEBUG_PRINTF("configService.humidityGreenLimit written\n");
  if (!configIsLocked) {
    humidityGreenLimit = config_humidityGreenLimit.value();
  }
}

void AppConfiguration::humidityAmberLimit_Written(BLEDevice central, BLECharacteristic characteristic) {
  DEBUG_PRINTF("configService.humidityAmberLimit written\n");
  if (!configIsLocked) {
    humidityAmberLimit = config_humidityAmberLimit.value();
  }
}

void AppConfiguration::lockConfig_Written(BLEDevice central, BLECharacteristic characteristic) {
  DEBUG_PRINTF("configService.lockConfig written - %s\n", lockConfig.value().c_str());
  if (!configIsLocked) {
    configLockPassword = lockConfig.value();
    configIsLocked = true;
    config_configIsLocked.writeValue(true);
  }
}

void AppConfiguration::unlockConfig_Written(BLEDevice central, BLECharacteristic characteristic) {
  DEBUG_PRINTF("configService.unlockConfig written - %s\n", unlockConfig.value().c_str());
  if (!configIsLocked) {
    if (unlockConfig.value() == configLockPassword) {
      configIsLocked = false;
      config_configIsLocked.writeValue(false);
    }
  }
}

void AppConfiguration::config_configService() {
  DEBUG_PRINTF("Configuring the configService\n");

  configService.addCharacteristic(config_sensorName);
  config_sensorName.setEventHandler(BLEWritten, this->sensorName_Written);

  configService.addCharacteristic(config_sensorLocation);
  config_sensorLocation.setEventHandler(BLEWritten, sensorLocation_Written);

  configService.addCharacteristic(config_humidityGreenLimit);
  config_humidityGreenLimit.setEventHandler(BLEWritten, humidityGreenLimit_Written);

  configService.addCharacteristic(config_humidityAmberLimit);
  config_humidityAmberLimit.setEventHandler(BLEWritten, humidityAmberLimit_Written);

  configService.addCharacteristic(lockConfig);
  lockConfig.setEventHandler(BLEWritten, lockConfig_Written);

  configService.addCharacteristic(unlockConfig);
  unlockConfig.setEventHandler(BLEWritten, unlockConfig_Written);

  configService.addCharacteristic(config_configIsLocked);
  config_configIsLocked.writeValue(configIsLocked);
  
  DEBUG_PRINTF("configService configured!\n");
}

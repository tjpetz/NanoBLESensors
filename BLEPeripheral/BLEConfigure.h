/*
 * BLEConfigure.h
 * 
 * Author: thomas.j.petz@gsk.com
 * 
 * Allow the boards configuration information to be managed via BLE.
 * 
 */

#ifndef __BLEConfigure_h__
#define __BLEConfigure_h__

#include <ArduinoBLE.h>

class AppConfiguration
{
  public:
    AppConfiguration(String sensorName, String sensorLocation, uint16_t humidityGreenLimit, uint16_t humidityAmberLimit);
    
    String sensorName;
    String sensorLocation;
    uint16_t humidityGreenLimit;
    uint16_t humidityAmberLimit;

    void config_configService();
    
  private:
    String configLockPassword ;
    bool configIsLocked;

    BLEService configService;
    // Configurable settings for the application
    BLEStringCharacteristic config_sensorName;
    BLEStringCharacteristic config_sensorLocation;
    BLEUnsignedIntCharacteristic config_humidityGreenLimit;
    BLEUnsignedIntCharacteristic config_humidityAmberLimit;
    // Configuration locking
    BLEStringCharacteristic lockConfig;
    BLEStringCharacteristic unlockConfig;
    BLEBoolCharacteristic config_configIsLocked;

  public:
    // Callbacks, static to avoid function signature problems.
    void sensorName_Written(BLEDevice central, BLECharacteristic characteristic);
    void sensorLocation_Written(BLEDevice central, BLECharacteristic characteristic);
    void humidityGreenLimit_Written(BLEDevice central, BLECharacteristic characteristic);
    void humidityAmberLimit_Written(BLEDevice central, BLECharacteristic characteristic);
    void lockConfig_Written(BLEDevice central, BLECharacteristic characteristic);
    void unlockConfig_Written(BLEDevice central, BLECharacteristic characteristic);
};

#endif

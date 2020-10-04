#ifndef BLECONFIGURE_H
#define BLECONFIGURE_H

#include <ArduinoBLE.h>

/**
 * @brief Provide a BLE service to allow reading and updatingthe
 * configuration information for the application.  Once
 * locked the settings are retained in flash memory.
 */
class BLEConfigure {
public:
  BLEConfigure();

  /** @brief Initialize the service, call before use.
   *  @return false if failed
   */
  bool begin();

  /** @return true if the configuration is locked */
  bool isLocked() const { return isLocked_; }

  /** @return a reference to the encapsulated BLE service */
  BLEService &getService() { return service_; }

  char sensorName[32]; /** name of the sensor in bluetooth */
  char location[64];   /** location of the sensor used for logging */
  uint16_t
      humidityGreenLimit; /** humidity below this level sets the LED green */
  uint16_t humidityAmberLimit; /** humidity below this level and above green set
                                  the LED amber */

protected:
  friend void on_lockConfig_Written(BLEDevice central,
                                    BLECharacteristic characteristic);

  /** @brief write the configuration to flash */
  void updateConfig();

  BLEService service_;
  BLEStringCharacteristic sensorName_Characteristic_;
  BLEDescriptor sensorName_Descriptor_;
  BLEStringCharacteristic location_Characteristic_;
  BLEUnsignedIntCharacteristic humidityGreenLimit_Characteristic_;
  BLEUnsignedIntCharacteristic humidityAmberLimit_Characteristic_;
  BLEBoolCharacteristic configIsLocked_Characteristic_;
  BLEStringCharacteristic lockPassword_Characteristic_;

  char lockPassword_[64];
  bool isLocked_;
  bool isInitialized_;
};

#endif

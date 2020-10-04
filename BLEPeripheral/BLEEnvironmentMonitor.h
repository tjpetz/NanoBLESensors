#include <Arduino.h>
#include <ArduinoBLE.h>

/**
 * @brief BLE service implementing the Environment Service profile.
 */
class BLEEnvironmentMonitor {
public:
  BLEEnvironmentMonitor(const char *locationName);

  /**
   * @brief Call to start the measurement devices.
   * @return true if successful
   */
  bool begin();

  /**
   * @brief measure the environment and update the BLE characteristics.
   */
  void measure();

  /**
   * @brief a reference to the the Environment service.
   * @return a reference to the BLEService
   */
  BLEService &getService() { return environmentService_; }

  uint16_t humidity() { return previousHumidity_; }
  int16_t temperature() { return previousTemperature_; }
  uint32_t pressure() { return previousPressure_; }

private:
  BLEService environmentService_;

  BLEUnsignedIntCharacteristic
      humidityCharacteristic_; // standard 16-bit UUID and remote client may
                               // read.
  BLEIntCharacteristic temperatureCharacteristic_; // standard UUID for temp
                                                   // characteristic in C 0.01
  BLEUnsignedLongCharacteristic
      pressureCharacteristic_; // standard UUID for pressure characteristinc in
                               // Pa 0.1
  BLEStringCharacteristic locationNameCharacteristic_; // location

  char location_[64];           // Name of the location
  uint16_t previousHumidity_;   // last humidity level
  int16_t previousTemperature_; // last temperature
  uint32_t previousPressure_;   // last pressure
};

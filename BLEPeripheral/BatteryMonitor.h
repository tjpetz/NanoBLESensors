#ifndef _BatteryMonitor_h_
#define _BatteryMonitor_h_

#include <ArduinoBLE.h>

/*
 * @brief A simple BLE service to measure the current battery voltage.
 * An analog and digital pin are required to measure the battery
 * and control the measurement.
 */
class BatteryMonitor {
  
  public:
    BatteryMonitor(const int sensorPin, const int controlPin);
    
    /** @brief begin to measure the battery voltage */
    void begin();
    
    /** @brief measure the battery voltage and update the BLE characteristic.
     * @return the current voltage */
    float measureVoltage();

    /** @brief return a reference to the battery service. */
    BLEService& getBatteryService();
  
  private:
    float currentVoltage_;
    const int sensorPin_;
    const int controlPin_;
    BLEService batteryService_;
    BLEByteCharacteristic batteryLevelCharacteristic_;

    float measureBattery() const;
    byte batteryPct(float battVoltage) const;
};

#endif

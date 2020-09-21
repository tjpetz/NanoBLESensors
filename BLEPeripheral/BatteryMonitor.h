/*
 * A simple BLE service to measure the current battery voltage.
 * An analog and digital pin are required to measure the battery
 * and control the measurement.
 */

#ifndef _BatteryMonitor_h_
#define _BatteryMonitor_h_

#include <ArduinoBLE.h>

class BatteryMonitor {
  
  public:
    BatteryMonitor(const int sensorPin, const int controlPin);
    void begin();
    float measureVoltage();
    BLEService getBatteryService() const;
  
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

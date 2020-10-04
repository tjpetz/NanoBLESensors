#include "BatteryMonitor.h"
#define _DEBUG_
#include "Debug.h"

BatteryMonitor::BatteryMonitor(const int sensorPin, const int controlPin)
  : sensorPin_(sensorPin),
    controlPin_(controlPin),
    batteryService_("180F"),
    batteryLevelCharacteristic_("2A19", BLERead | BLENotify)
{
  analogReference(AR_INTERNAL2V4);
  pinMode(controlPin_, OUTPUT);
  
  currentVoltage_ = measureBattery();
}

void BatteryMonitor::begin() {
  // Use the internal reference of 2.4 v.

  batteryService_.addCharacteristic(batteryLevelCharacteristic_);
  batteryLevelCharacteristic_.writeValue(batteryPct(currentVoltage_));
}

BLEService& BatteryMonitor::getService() {
  return batteryService_;
}

float BatteryMonitor::measureVoltage() {

  float battVoltage = measureBattery();
  byte battPct = batteryPct(battVoltage);

  DEBUG_PRINTF("battVoltage = %0.2fv, battPct = %d%%\n", battVoltage, battPct);

  // Update the characteristic if the voltage change is > 1.0%
  if (abs((currentVoltage_ - battVoltage) / currentVoltage_) > 0.01) {
    Serial.println("Updating battery level characteristic.");
    currentVoltage_ = battVoltage;
    batteryLevelCharacteristic_.writeValue(batteryPct(currentVoltage_));    
  }

  return battVoltage;
}

byte BatteryMonitor::batteryPct(float battVoltage) const {
  // Battery Capacity is 3.0v = 0% to 4.2v = 100%
  return ((battVoltage - 3.0) / 1.2) * 100.0;
}

float BatteryMonitor::measureBattery() const {
  // The voltage is measured relative to 2.4 volt reference and is sourced 
  // through a 2::1 voltage divider.

  // Turn on the MOSFET to read the voltage.
  digitalWrite(controlPin_, HIGH);
  float v = map(analogRead(sensorPin_), 0, 1024, 0, 2400) / 1000.0 * 2.0;
  digitalWrite(controlPin_, LOW);
  
  return v;
}

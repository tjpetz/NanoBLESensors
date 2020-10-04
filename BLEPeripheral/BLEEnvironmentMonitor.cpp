#include <Arduino.h>
#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>

#include "BLEEnvironmentMonitor.h"

BLEEnvironmentMonitor::BLEEnvironmentMonitor(const char *location)
    : environmentService_("181A"),
      humidityCharacteristic_("2A6F", BLERead | BLENotify),
      temperatureCharacteristic_("2A6E", BLERead | BLENotify | BLEBroadcast),
      pressureCharacteristic_("2A6D", BLERead | BLENotify),
      locationNameCharacteristic_("2AB5", BLERead | BLENotify,
                                  sizeof(location_)),
      previousHumidity_(0), previousTemperature_(0), previousPressure_(0) {
  strncpy(location_, location, sizeof(location_));
}

bool BLEEnvironmentMonitor::begin() {
  bool retStatus = true;

  retStatus &= HTS.begin();
  retStatus &= BARO.begin();

  environmentService_.addCharacteristic(humidityCharacteristic_);
  environmentService_.addCharacteristic(temperatureCharacteristic_);
  environmentService_.addCharacteristic(pressureCharacteristic_);
  environmentService_.addCharacteristic(locationNameCharacteristic_);

  // Set the initial values
  previousHumidity_ = (uint16_t)(HTS.readHumidity() * 100);
  previousTemperature_ = (int16_t)(HTS.readTemperature() * 100);
  previousPressure_ = (uint32_t)(BARO.readPressure() * 10000);

  humidityCharacteristic_.writeValue(previousHumidity_);
  temperatureCharacteristic_.writeValue(previousTemperature_);
  pressureCharacteristic_.writeValue(previousPressure_);
  locationNameCharacteristic_.writeValue(location_);

  return retStatus;
}

void BLEEnvironmentMonitor::measure() {
  uint16_t currentHumidity = (uint16_t)(HTS.readHumidity() * 100);
  if (currentHumidity != previousHumidity_) {
    humidityCharacteristic_.writeValue(currentHumidity);
    previousHumidity_ = currentHumidity;
  }

  // Update the temperature reading
  int16_t currentTemperature = (int16_t)(HTS.readTemperature() * 100);
  if (currentTemperature != previousTemperature_) {
    temperatureCharacteristic_.writeValue(currentTemperature);
    previousTemperature_ = currentTemperature;
  }

  // Update the pressure reading, reading is in kPa and we need to report in 0.1
  // pa
  uint32_t currentPressure = (uint32_t)(BARO.readPressure() * 10000);
  if (currentPressure != previousPressure_) {
    pressureCharacteristic_.writeValue(currentPressure);
    previousPressure_ = currentPressure;
  }
}
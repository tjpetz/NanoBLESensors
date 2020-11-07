/**
 * EnvironmentSensor.cpp
 */

#include "EnvironmentSensor.h"

const char *EnvironmentSensor::environmentServiceUUID = "181A";
const char *EnvironmentSensor::temperatureCharacteristicUUID = "2A6E";
const char *EnvironmentSensor::humidityCharacteristicUUID = "2A6F";
const char *EnvironmentSensor::pressureCharacteristicUUID = "2A6D";
const char *EnvironmentSensor::locationCharacteristicUUID = "2AB5";

EnvironmentSensor::EnvironmentSensor(BLEDevice sensor) {
  sensor_ = sensor;
  strncpy(name_, sensor.localName().c_str(), sizeof(name_));
}

EnvironmentSensor::~EnvironmentSensor() {
  if (sensor_.connected()) {
    sensor_.disconnect();
  }
}

bool EnvironmentSensor::connect() {
  return sensor_.connect();
}

bool EnvironmentSensor::disconnect() {
  return sensor_.disconnect();
}

bool EnvironmentSensor::readSensor() {

  if (!sensor_.connected() && !connect()) {
    return false;
  }

  if (!sensor_.discoverService(EnvironmentSensor::environmentServiceUUID)) {
    return false;
  }

  // Read the temperature
  if (BLECharacteristic temperatureCharacteristic = sensor_.characteristic(EnvironmentSensor::temperatureCharacteristicUUID)) {
    int16_t temperatureRawValue;
    temperatureCharacteristic.readValue(temperatureRawValue);
    temperature_ = temperatureRawValue / 100.0;
    DEBUG_PRINTF("Temperature = %.2f C\n", temperature_);
  } else {
    DEBUG_PRINTF("Cannot find temperature characteristic\n");
  }

  // Read the humidity
  if (BLECharacteristic humidityCharacteristic = sensor_.characteristic(EnvironmentSensor::humidityCharacteristicUUID)) {
    uint16_t humidityRawValue;
    humidityCharacteristic.readValue(humidityRawValue);
    humidity_ = humidityRawValue / 100.0;
    DEBUG_PRINTF("Humidity = %.2f %%\n", humidity_);
  } else {
    DEBUG_PRINTF("Cannot find humidity characteristic\n");
  }

  // Read the pressure
  if (BLECharacteristic pressureCharacteristic = sensor_.characteristic(EnvironmentSensor::pressureCharacteristicUUID)) {
    uint32_t pressureRawValue;
    pressureCharacteristic.readValue(pressureRawValue);
    pressure_ = pressureRawValue * 0.00001450; // convert to PSI
    DEBUG_PRINTF("Pressure = %.2f PSI\n", pressure_);
  } else {
    DEBUG_PRINTF("Cannot find pressure characteristic\n");
  }

  // Read the location
  if (BLECharacteristic locationNameCharacteristic = sensor_.characteristic(EnvironmentSensor::locationCharacteristicUUID)) {
    char buff[255];
    memset(buff, 0, sizeof(buff)); // zero out the buffer.
    locationNameCharacteristic.readValue(buff, sizeof(buff));
    strncpy(location_, buff, sizeof(location_));
    DEBUG_PRINTF("Location Name = %s\n", location_);
  } else {
    DEBUG_PRINTF("Cannot find locationName characteristic\n");
  }

  return true;
}  


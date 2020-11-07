/**
 * An environment sensor will access a BLE peripheral to retrieve the
 * environment characteristics.
 */

#pragma once

#include <ArduinoBLE.h>

#define _DEBUG_
#include "Debug.h"

class EnvironmentSensor {
  
  public:
    EnvironmentSensor(BLEDevice sensor);
    ~EnvironmentSensor();

    /**
     * @brief connect to the BLE peripheral
     * @return true on success
     */
    bool connect();

    /**
     * @brief disconnect from the BLE peripheral
     * @return true on success
     */
    bool disconnect();

    /**
     * @brief retrieve the sensor measurements
     * @return true on success
     */
    bool readSensor();

    /** @brief the bluetooth address of the sensor */
    String address() const {return sensor_.address();}

    /** @brief the name of the sensor */
    const char* name() const {return name_;}

    /** @brief the location of the sensor */
    const char* location() const {return location_;}

    /** @brief the temperature in C */
    float temperature() const {return temperature_;} 

    /** @brief the humidity in % RH */
    float humidity() const {return humidity_;}

    /** @brief the pressure in PSI */
    float pressure() const {return pressure_;}

    static const char *environmentServiceUUID;          /** UUID of the environment service profile */
    static const char *temperatureCharacteristicUUID;   /** UUID of the temperature characteristic */
    static const char *humidityCharacteristicUUID;      /** UUID of the humidity characteristic */
    static const char *pressureCharacteristicUUID;      /** UUID of the pressure characteristic */
    static const char *locationCharacteristicUUID;      /** UUID of the location characteristic */

  private:
    BLEDevice sensor_;
    char name_[64];
    char location_[64];
    float temperature_;
    float humidity_;
    float pressure_;
};


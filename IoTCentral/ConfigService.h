#ifndef CONFIGSERVICE_H
#define CONFIGSERVICE_H

#include <ArduinoBLE.h>

/**
 * @brief Provide a BLE service to allow reading and updating the
 * configuration information for the application.  Once
 * locked the settings are retained in flash memory.
 */
class ConfigService {
public:
  ConfigService(const char *p_hostName, const char *p_mqttBroker,
                const char *p_topicRoot, unsigned int p_sampleInterval);

  /** @brief Initialize the service, call before use. */
  void begin();

  /** @return true if initialized */
  bool initialized() const;

  /** @return a reference to the ConfigService */
  BLEService &getConfigService();

  char ssid[64];         /** the ssid for WiFi */
  char wifiPassword[64]; /** the wifi Password */
  char hostName[32];     /** host name used for as the local device name */
  char mqttBroker[64];   /** FQDN address of the mqtt server */
  char topicRoot[64];    /** mqtt topic name to use as the base name */
  unsigned int sampleInterval; /** not currently used! */

  bool isInitialized; /** True if initialized from Flash memory */
  bool isLocked;      /** True if the configuration has been locked */

  /** @brief process callbacks for the the service characteristics */
  friend void onLock(BLEDevice central, BLECharacteristic characteristic);

protected:
  BLEService service_;
  BLEStringCharacteristic ssidCharacteristic_;
  BLEDescriptor ssidDescriptor_;
  BLEStringCharacteristic wifiPasswordCharacteristic_;
  BLEDescriptor wifiPasswordDescriptor_;
  BLEStringCharacteristic hostNameCharacteristic_;
  BLEDescriptor hostNameDescriptor_;
  BLEStringCharacteristic mqttBrokerCharacteristic_;
  BLEDescriptor mqttBrokerDescriptor_;
  BLEStringCharacteristic topicRootCharacteristic_;
  BLEDescriptor topicRootDescriptor_;
  BLEUnsignedIntCharacteristic sampleIntervalCharacteristic_;
  BLEUnsignedIntCharacteristic isLockedCharacteristic_;
  BLEDescriptor sampleIntervalDescriptor_;
  BLEStringCharacteristic lockCharacteristic_;

  void debug_print_configuration();
  
private:
  char configurationPassword[64]; /** password to lock and unlock the
                                   configuration */
};

#endif

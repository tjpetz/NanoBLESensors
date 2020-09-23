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
    ConfigService(String p_hostName, String p_mqttBroker, String p_topicRoot, unsigned int p_sampleInterval);

    /** @brief Initialize the service, call before use. */
    void begin();

    /** @return true if initialized */
    bool initialized() const;

    /** @return a reference to the ConfigService */
    BLEService& getConfigService();
    
    String ssid;                    /** the ssid for WiFi */
    String wifiPassword;            /** the wifi Password */
    String hostName;                /** host name used for as the local device name */
    String mqttBroker;              /** FQDN address of the mqtt server */  
    String topicRoot;               /** mqtt topic name to use as the base name */
    unsigned int sampleInterval;    /** not currently used! */
    String configurationPassword;   /** password to lock and unlock the configuration */

    bool isInitialized;             /** True if initialized from Flash memory */
    bool isLocked;                  /** True if the configuration has been locked */

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
    BLEDescriptor sampleIntervalDescriptor_;
    BLEStringCharacteristic lockCharacteristic_;
    BLEUnsignedIntCharacteristic isLockedCharacteristic_;    
};

#endif

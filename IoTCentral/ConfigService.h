/*
 * Provide a BLE service to allow reading and updating the
 * configuration information for the application.
 */

#ifndef __CONFIGSERVICE_H__
#define __CONFIGSERVICE_H__

#include <ArduinoBLE.h>

class ConfigService {
  public:
    ConfigService(String p_hostName, String p_mqttBroker, String p_topicRoot, unsigned int p_sampleInterval);

    void begin();

    bool initialized() const;
    BLEService& getConfigService();
    
    String ssid;
    String wifiPassword;
    String hostName;
    String mqttBroker;
    String topicRoot;
    unsigned int sampleInterval;
    String configurationPassword;

    bool isInitialized;
    bool isLocked;

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

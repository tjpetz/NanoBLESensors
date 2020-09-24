#include "ConfigService.h"

#define _DEBUG_
#include "Debug.h"

#include <FlashAsEEPROM.h>

typedef struct {
  boolean valid;
  char ssid[255];
  char wifiPassword[255];
  char hostName[255];
  char mqttBroker[255];
  char topicRoot[255];
  unsigned int sampleInterval;
  char configurationPassword[255];
} ConfigServiceEEPROM_t;

FlashStorage(flash_configuration, ConfigServiceEEPROM_t);

static ConfigService* singleton;        /** the one and only instance, used in the callbacks */

void onLock(BLEDevice central, BLECharacteristic characteristic);

ConfigService::ConfigService(String p_hostName, String p_mqttBroker, String p_topicRoot, unsigned int p_sampleInterval)
  : service_("44af2abf-f3d9-4429-bbb8-ec770f1e355a"),
    ssidCharacteristic_("0465ac13-a0f3-46a6-990a-5a04b32b3b60", BLERead | BLEWrite, 255),
    ssidDescriptor_("2901", "WiFi SSID"),
    wifiPasswordCharacteristic_("27030384-eac9-4907-8e44-5c16d778aa7a", BLEWrite, 255),
    wifiPasswordDescriptor_("2901", "WiFi Password"),
    hostNameCharacteristic_("ef42bf97-1b9c-4d45-941d-d60dc564dc6f", BLERead | BLEWrite, 255),
    hostNameDescriptor_("2901", "Host Name"),
    mqttBrokerCharacteristic_("0d071785-b22b-49d6-86be-270de52da930", BLERead | BLEWrite, 255),
    mqttBrokerDescriptor_("2901", "MQTT Address"),
    topicRootCharacteristic_("18cda5b0-3b76-4319-9716-acd1a409d3f6", BLERead | BLEWrite, 255),
    topicRootDescriptor_("2901", "MQTT Root Topic"),
    sampleIntervalCharacteristic_("1682229f-bb5c-4f4a-96a9-1027f13d83f9", BLERead | BLEWrite),
    sampleIntervalDescriptor_("2901", "Sample Interval in Seconds"),
    isLockedCharacteristic_("d02db20e-6e1f-4541-bd3d-7715e00b2b82", BLERead | BLENotify),
    lockCharacteristic_("29636a43-d59a-46a1-ad0a-34aa23a0e90c", BLEWrite, 255)
{
  hostName = p_hostName;
  mqttBroker = p_mqttBroker;
  topicRoot = p_topicRoot;
  sampleInterval = p_sampleInterval;

  isInitialized = false;
  configurationPassword = "";
  isLocked = false;

  ConfigServiceEEPROM_t flashConfig;

  flashConfig = flash_configuration.read();
  if (flashConfig.valid) {
    // Load the config settings from flash
    ssid = flashConfig.ssid;
    wifiPassword = flashConfig.wifiPassword;
    hostName = flashConfig.hostName;
    mqttBroker = flashConfig.mqttBroker;
    topicRoot = flashConfig.topicRoot;
    sampleInterval = flashConfig.sampleInterval;
    configurationPassword = flashConfig.configurationPassword;

    isInitialized = true;
    isLocked = false;
  }
}


BLEService& ConfigService::getConfigService() {
  return service_;
}


void ConfigService::begin() {

  singleton = this;

  ssidCharacteristic_.addDescriptor(ssidDescriptor_);
  wifiPasswordCharacteristic_.addDescriptor(wifiPasswordDescriptor_);
  hostNameCharacteristic_.addDescriptor(hostNameDescriptor_);
  mqttBrokerCharacteristic_.addDescriptor(mqttBrokerDescriptor_);
  topicRootCharacteristic_.addDescriptor(topicRootDescriptor_);
  sampleIntervalCharacteristic_.addDescriptor(sampleIntervalDescriptor_);

  // set the initial values
  ssidCharacteristic_.writeValue("please set me");
  hostNameCharacteristic_.writeValue(hostName.c_str());
  mqttBrokerCharacteristic_.writeValue(mqttBroker.c_str());
  topicRootCharacteristic_.writeValue(topicRoot.c_str());
  sampleIntervalCharacteristic_.writeValue(sampleInterval);
  isLockedCharacteristic_.writeValue(isLocked ? 1 : 0);
  
  // Callbacks for lock and unlock
  lockCharacteristic_.setEventHandler(BLEWritten, onLock);
  
  service_.addCharacteristic(ssidCharacteristic_);
  service_.addCharacteristic(wifiPasswordCharacteristic_);
  service_.addCharacteristic(hostNameCharacteristic_);
  service_.addCharacteristic(mqttBrokerCharacteristic_);
  service_.addCharacteristic(topicRootCharacteristic_);
  service_.addCharacteristic(sampleIntervalCharacteristic_);
  service_.addCharacteristic(isLockedCharacteristic_);
  service_.addCharacteristic(lockCharacteristic_);

  BLE.addService(service_);
}

void onLock(BLEDevice central, BLECharacteristic characteristic) {
  if (!singleton->isLocked) {
    DEBUG_PRINTF("Configuration Locked with password = %s\n", singleton->lockCharacteristic_.value().c_str());
    singleton->isInitialized = true;
    singleton->isLocked = true;
    singleton->configurationPassword = singleton->lockCharacteristic_.value();
    singleton->ssid = singleton->ssidCharacteristic_.value();
    singleton->wifiPassword = singleton->wifiPasswordCharacteristic_.value();
    singleton->isLockedCharacteristic_.writeValue(1);
    
    // Save the configuration to EEPROM
    ConfigServiceEEPROM_t flashConfig;
  
    // Set the config object
    flashConfig.valid = true;
    strcpy(flashConfig.ssid, singleton->ssid.c_str());
    strcpy(flashConfig.wifiPassword, singleton->wifiPassword.c_str());
    strcpy(flashConfig.hostName, singleton->hostName.c_str());
    strcpy(flashConfig.mqttBroker, singleton->mqttBroker.c_str());
    strcpy(flashConfig.topicRoot, singleton->topicRoot.c_str());
    flashConfig.sampleInterval = singleton->sampleInterval;
    strcpy(flashConfig.configurationPassword, singleton->configurationPassword.c_str());

    flash_configuration.write(flashConfig);
      
  } else {
    if (singleton->lockCharacteristic_.value() == singleton->configurationPassword) {
      DEBUG_PRINTF("Configuration Unlocked with password = %s\n", singleton->lockCharacteristic_.value().c_str());
      singleton->isLocked = false;
      singleton->isLockedCharacteristic_.writeValue(0);
    } else {
      DEBUG_PRINTF("ERROR - incorrect password, configuation is locked!\n");
    }
  }
}

#include "ConfigService.h"

#define _DEBUG_
#include "Debug.h"

#include <FlashAsEEPROM.h>

typedef struct {
  int valid;
  char ssid[64];
  char wifiPassword[64];
  char hostName[32];
  char mqttBroker[64];
  char topicRoot[64];
  unsigned int sampleInterval;
  char configurationPassword[64];
} ConfigServiceEEPROM_t;

FlashStorage(flash_configuration, ConfigServiceEEPROM_t);

static ConfigService
    *singleton; /** the one and only instance, used in the callbacks */

void onLock(BLEDevice central, BLECharacteristic characteristic);

ConfigService::ConfigService(const char *p_hostName, const char *p_mqttBroker,
                             const char *p_topicRoot,
                             unsigned int p_sampleInterval)
    : service_("44af2abf-f3d9-4429-bbb8-ec770f1e355a"),
      ssidCharacteristic_("0465ac13-a0f3-46a6-990a-5a04b32b3b60",
                          BLERead | BLEWrite, sizeof(ssid)),
      ssidDescriptor_("2901", "WiFi SSID"),
      wifiPasswordCharacteristic_("27030384-eac9-4907-8e44-5c16d778aa7a",
                                  BLEWrite, sizeof(wifiPassword)),
      wifiPasswordDescriptor_("2901", "WiFi Password"),
      hostNameCharacteristic_("ef42bf97-1b9c-4d45-941d-d60dc564dc6f",
                              BLERead | BLEWrite, sizeof(hostName)),
      hostNameDescriptor_("2901", "Host Name"),
      mqttBrokerCharacteristic_("0d071785-b22b-49d6-86be-270de52da930",
                                BLERead | BLEWrite, sizeof(mqttBroker)),
      mqttBrokerDescriptor_("2901", "MQTT Address"),
      topicRootCharacteristic_("18cda5b0-3b76-4319-9716-acd1a409d3f6",
                               BLERead | BLEWrite, sizeof(topicRoot)),
      topicRootDescriptor_("2901", "MQTT Root Topic"),
      sampleIntervalCharacteristic_("1682229f-bb5c-4f4a-96a9-1027f13d83f9",
                                    BLERead | BLEWrite),
      sampleIntervalDescriptor_("2901", "Sample Interval in Seconds"),
      isLockedCharacteristic_("d02db20e-6e1f-4541-bd3d-7715e00b2b82",
                              BLERead | BLENotify),
      lockCharacteristic_("29636a43-d59a-46a1-ad0a-34aa23a0e90c", BLEWrite,
                          sizeof(configurationPassword)) {
  memset(ssid, 0, sizeof(ssid));
  strncpy(hostName, p_hostName, sizeof(hostName));
  strncpy(mqttBroker, p_mqttBroker, sizeof(mqttBroker));
  strncpy(topicRoot, p_topicRoot, sizeof(topicRoot));
  sampleInterval = p_sampleInterval;

  isInitialized = false;
  memset(configurationPassword, 0, sizeof(configurationPassword));
  isLocked = false;

  ConfigServiceEEPROM_t flashConfig;

  flashConfig = flash_configuration.read();
  if (flashConfig.valid == 99) {
    // Load the config settings from flash
    strncpy(ssid, flashConfig.ssid, sizeof(ssid));
    strncpy(wifiPassword, flashConfig.wifiPassword, sizeof(wifiPassword));
    strncpy(hostName, flashConfig.hostName, sizeof(hostName));
    strncpy(mqttBroker, flashConfig.mqttBroker, sizeof(mqttBroker));
    strncpy(topicRoot, flashConfig.topicRoot, sizeof(topicRoot));
    sampleInterval = flashConfig.sampleInterval;
    strncpy(configurationPassword, flashConfig.configurationPassword,
            sizeof(configurationPassword));

    isInitialized = true;
    isLocked = true;
  }
}

BLEService &ConfigService::getConfigService() { return service_; }

void ConfigService::begin() {

  singleton = this;

  ssidCharacteristic_.addDescriptor(ssidDescriptor_);
  wifiPasswordCharacteristic_.addDescriptor(wifiPasswordDescriptor_);
  hostNameCharacteristic_.addDescriptor(hostNameDescriptor_);
  mqttBrokerCharacteristic_.addDescriptor(mqttBrokerDescriptor_);
  topicRootCharacteristic_.addDescriptor(topicRootDescriptor_);
  sampleIntervalCharacteristic_.addDescriptor(sampleIntervalDescriptor_);

  // set the initial values
  ssidCharacteristic_.writeValue(ssid);
  hostNameCharacteristic_.writeValue(hostName);
  mqttBrokerCharacteristic_.writeValue(mqttBroker);
  topicRootCharacteristic_.writeValue(topicRoot);
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
    DEBUG_PRINTF("Configuration Locked with password = %s\n",
                 singleton->lockCharacteristic_.value().c_str());
    singleton->isInitialized = true;
    singleton->isLocked = true;
    strncpy(singleton->configurationPassword,
            singleton->lockCharacteristic_.value().c_str(),
            sizeof(singleton->configurationPassword));
    strncpy(singleton->ssid, singleton->ssidCharacteristic_.value().c_str(),
            sizeof(singleton->ssid));
    strncpy(singleton->wifiPassword,
            singleton->wifiPasswordCharacteristic_.value().c_str(),
            sizeof(singleton->wifiPassword));
    strncpy(singleton->hostName,
            singleton->hostNameCharacteristic_.value().c_str(),
            sizeof(singleton->hostName));
    strncpy(singleton->mqttBroker,
            singleton->mqttBrokerCharacteristic_.value().c_str(),
            sizeof(singleton->mqttBroker));
    strncpy(singleton->topicRoot,
            singleton->topicRootCharacteristic_.value().c_str(),
            sizeof(singleton->topicRoot));
    singleton->sampleInterval = singleton->sampleIntervalCharacteristic_.value();
    singleton->isLockedCharacteristic_.writeValue(1);

    // Save the configuration to EEPROM
    ConfigServiceEEPROM_t flashConfig;

    // Set the config object
    flashConfig.valid = 99;
    strncpy(flashConfig.ssid, singleton->ssid, sizeof(flashConfig.ssid));
    strncpy(flashConfig.wifiPassword, singleton->wifiPassword,
            sizeof(flashConfig.wifiPassword));
    strncpy(flashConfig.hostName, singleton->hostName,
            sizeof(flashConfig.hostName));
    strncpy(flashConfig.mqttBroker, singleton->mqttBroker,
            sizeof(flashConfig.mqttBroker));
    strncpy(flashConfig.topicRoot, singleton->topicRoot,
            sizeof(flashConfig.topicRoot));
    flashConfig.sampleInterval = singleton->sampleInterval;
    strncpy(flashConfig.configurationPassword, singleton->configurationPassword,
            sizeof(flashConfig.configurationPassword));

    DEBUG_PRINTF("valid = %d\n", flashConfig.valid);
    DEBUG_PRINTF("ssid = %s\n", flashConfig.ssid);
    DEBUG_PRINTF("hostName = %s\n", flashConfig.hostName);

    flash_configuration.write(flashConfig);

    ConfigServiceEEPROM_t flashConfig2 = flash_configuration.read();

    DEBUG_PRINTF("After flash and re-read\n");
    DEBUG_PRINTF("valid = %d\n", flashConfig2.valid);
    DEBUG_PRINTF("ssid = %s\n", flashConfig2.ssid);
    DEBUG_PRINTF("hostName = %s\n", flashConfig2.hostName);

    singleton->debug_print_configuration();

  } else {
    if (singleton->lockCharacteristic_.value() ==
        singleton->configurationPassword) {
      DEBUG_PRINTF("Configuration Unlocked with password = %s\n",
                   singleton->lockCharacteristic_.value().c_str());
      singleton->isLocked = false;
      singleton->isLockedCharacteristic_.writeValue(0);
    } else {
      DEBUG_PRINTF("ERROR - incorrect password, configuation is locked!\n");
    }
  }
}

void ConfigService::debug_print_configuration() {
  DEBUG_PRINTF("--- Current Configuration ---\n");
  DEBUG_PRINTF(" SSID = %s\n", ssid);
  DEBUG_PRINTF(" hostname = %s\n", hostName);
  DEBUG_PRINTF(" mqttBroker = %s\n", mqttBroker);
  DEBUG_PRINTF(" topicRoot = %s\n", topicRoot);
  DEBUG_PRINTF(" sampleInterval = %u\n", sampleInterval);
}

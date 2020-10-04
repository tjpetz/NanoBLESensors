#include "BLEConfigure.h"
#include "Debug.h"
#include "Watchdog.h"

static BLEConfigure *singleton; /** the one and only instance */

// the system configuration settings need to be added to this struct
typedef struct flash_mem {
  char sensor_name[128];
  char location[128];
  uint16_t humidityGreenLimit;
  uint16_t humidityAmberLimit;
  char configLockPassword[64];
  char reserve[4096 - 2 * 128 - 4 -
               64]; // fill out the remainder of the page so nothing else is
                    // allocated in this page
} flash_config_t;

// Default configuration information, this is stored in flash memory
const flash_config_t flashConfig __attribute__((aligned(0x1000))) = {
    "NanoBLESense", // sensorName
    "Location",     // location
    4500,           // green humidity limit
    6500,           // amber humidity limit
    ""              // lock password is empty by default
};

void on_lockConfig_Written(BLEDevice central, BLECharacteristic characteristic);
void writeFlashPage(uint32_t *to, uint32_t *from, int len);
void hexDumpMemory(uint8_t *memStart, unsigned int nbrOfBytes);

BLEConfigure::BLEConfigure()
    : service_("7f76c1b2-c592-4ace-8089-47bf14d07ced"),
      sensorName_Characteristic_("641aff8f-217e-4ddb-aece-3053b128c27d",
                                 BLERead | BLEWrite, sizeof(sensorName)),
      sensorName_Descriptor_("2901", "Sensor Name"),
      location_Characteristic_("52b8d6c4-cd20-4f51-bf71-ea0de788ebb4",
                               BLERead | BLEWrite, sizeof(location)),
      humidityGreenLimit_Characteristic_("ebf92cf8-2744-4df2-be70-e856fcaf01a7",
                                         BLERead | BLEWrite),
      humidityAmberLimit_Characteristic_("90441958-8975-4f62-aa13-08bdb86acd16",
                                         BLERead | BLEWrite),
      configIsLocked_Characteristic_("32b6a19f-ddac-45db-a1b5-8a7dbe9a76fc",
                                     BLERead | BLENotify),
      lockPassword_Characteristic_("3ffb9262-18a2-4acf-918c-2f8932577c48",
                                   BLEWrite, sizeof(lockPassword_)) {

  // Initialize from flash
  strncpy(sensorName, flashConfig.sensor_name, sizeof(sensorName));
  strncpy(location, flashConfig.location, sizeof(location));
  humidityGreenLimit = flashConfig.humidityGreenLimit;
  humidityAmberLimit = flashConfig.humidityAmberLimit;
  strncpy(lockPassword_, flashConfig.configLockPassword, sizeof(lockPassword_));

  isLocked_ = (lockPassword_[0] != '\0');
}

bool BLEConfigure::begin() {
  DEBUG_PRINTF("Configuring the configService\n");

  bool retStatus = true;

  singleton = this;

  sensorName_Characteristic_.addDescriptor(sensorName_Descriptor_);
  service_.addCharacteristic(sensorName_Characteristic_);
  retStatus &= sensorName_Characteristic_.writeValue(sensorName);

  service_.addCharacteristic(location_Characteristic_);
  retStatus &= location_Characteristic_.writeValue(location);

  service_.addCharacteristic(humidityGreenLimit_Characteristic_);
  retStatus &=
      humidityGreenLimit_Characteristic_.writeValue(humidityGreenLimit);

  service_.addCharacteristic(humidityAmberLimit_Characteristic_);
  retStatus &=
      humidityAmberLimit_Characteristic_.writeValue(humidityAmberLimit);

  service_.addCharacteristic(lockPassword_Characteristic_);
  lockPassword_Characteristic_.setEventHandler(BLEWritten,
                                               on_lockConfig_Written);

  service_.addCharacteristic(configIsLocked_Characteristic_);
  configIsLocked_Characteristic_.writeValue(isLocked());

  DEBUG_PRINTF("configService configured!\n");

  return retStatus;
}

void on_lockConfig_Written(BLEDevice central,
                           BLECharacteristic characteristic) {
  if (!singleton->isLocked_) {
    // save the values, update the flash memory, and lock the configuration
    strncpy(singleton->sensorName,
            singleton->sensorName_Characteristic_.value().c_str(),
            sizeof(singleton->sensorName));
    strncpy(singleton->location,
            singleton->location_Characteristic_.value().c_str(),
            sizeof(singleton->location));
    strncpy(singleton->lockPassword_,
            singleton->lockPassword_Characteristic_.value().c_str(),
            sizeof(singleton->lockPassword_));
    singleton->humidityGreenLimit =
        singleton->humidityGreenLimit_Characteristic_.value();
    singleton->humidityAmberLimit =
        singleton->humidityAmberLimit_Characteristic_.value();
    singleton->isLocked_ = true;
    singleton->configIsLocked_Characteristic_.writeValue(true);
    singleton->updateConfig();
    DEBUG_PRINTF("Configuration Updated!\n");
    DEBUG_PRINTF("  sensorName = %s\n", singleton->sensorName);
    DEBUG_PRINTF("  location = %s\n", singleton->location);
    DEBUG_PRINTF("  humidityGreenLimit = %du\n", singleton->humidityGreenLimit);
    DEBUG_PRINTF("  humidityAmberLimit = %du\n", singleton->humidityAmberLimit);
    DEBUG_PRINTF("  lockPassword = %s\n", singleton->lockPassword_);
  } else {
    if (strcmp(singleton->lockPassword_,
               singleton->lockPassword_Characteristic_.value().c_str()) == 0) {
      singleton->isLocked_ = false;
      singleton->configIsLocked_Characteristic_.writeValue(false);
      DEBUG_PRINTF("Configuration unlocked with password = %s\n",
                   singleton->lockPassword_);
    } else {
      DEBUG_PRINTF(
          "Invalid password attempting to unlock the configuration: %s\n",
          singleton->lockPassword_Characteristic_.value().c_str());
    }
  }
}

// Update the flash configuration
void BLEConfigure::updateConfig() {

  // To update the flash we make a copy of the values we want to write in
  // and then erase and write out the new values.
  DEBUG_PRINTF("Updating the flash config...\n");

  flash_config_t newFlash;
  memset(&newFlash, 0, sizeof(newFlash)); // zero the memory
  strncpy(newFlash.sensor_name, sensorName, sizeof(newFlash.sensor_name));
  strncpy(newFlash.location, location, sizeof(newFlash.location));
  newFlash.humidityGreenLimit = humidityGreenLimit;
  newFlash.humidityAmberLimit = humidityAmberLimit;

  DEBUG_PRINTF("newFlash.sensor_name = %s\n", newFlash.sensor_name);
  DEBUG_PRINTF("newFlash.location = %s\n", newFlash.location);
  DEBUG_PRINTF("newFlash.humidityGreenLimit = %d\n",
               newFlash.humidityGreenLimit);
  DEBUG_PRINTF("newFlash.humidityAmberLimit = %d\n",
               newFlash.humidityAmberLimit);

  resetWDT();
  writeFlashPage((uint32_t *)&flashConfig, (uint32_t *)&newFlash,
                 sizeof(newFlash));
  resetWDT();

  DEBUG_PRINTF("... flash updated!\n");
}

void writeFlashPage(uint32_t *to, uint32_t *from, int len) {

  uint32_t *tgt, *src;
  tgt = to;
  src = from;

  DEBUG_PRINTF("Before updating flash...");
  hexDumpMemory((uint8_t *)to, (unsigned int)len);
  hexDumpMemory((uint8_t *)from, (unsigned int)len);

  // Erase the page
  if (*(uint32_t *)NRF_NVMC->READY == 1) {
    DEBUG_PRINTF("Erasing...");
    *(uint32_t *)NRF_NVMC->CONFIG = 0x02;
    *(uint32_t *)NRF_NVMC->ERASEPAGE = (uint32_t)(tgt);
    while (*(uint32_t *)NRF_NVMC->READY == 0)
      delay(85);
    *(uint32_t *)NRF_NVMC->CONFIG = 0x00;
    DEBUG_PRINTF("...Erased\n");

    if (*(uint32_t *)NRF_NVMC->READY == 1) {
      DEBUG_PRINTF("Writing to = 0x%08lx, from = 0x%08lx\n", (uint32_t)tgt,
                   (uint32_t)src);
      // write the values
      *(uint32_t *)NRF_NVMC->CONFIG = 0x01;
      for (int i = 0; i < len; i += 4) {
        *tgt++ = *src++;
        while (*(uint32_t *)NRF_NVMC->READY == 0)
          delayMicroseconds(41);
      }
      *(uint32_t *)NRF_NVMC->CONFIG = 0x00;
    } else {
      DEBUG_PRINTF("... NOT ready to write to flash! ...\n");
    }
    DEBUG_PRINTF("... Memory Flashed ...\n");
  } else {
    DEBUG_PRINTF("... Flash Not Ready to Erase! ...\n");
  }

  hexDumpMemory((uint8_t *)to, (unsigned int)len);
}

void hexDumpMemory(uint8_t *memStart, unsigned int nbrOfBytes) {
  /* hex dump memory to the serial interface starting at memStart for nbrOfBytes
   */

  uint8_t *ptr;
  int bytesPerLine = 15;
  int bytesOnLine = 0;

  ptr = memStart;

  Serial.print("Memory dump of: ");
  Serial.println((unsigned long)(memStart), HEX);

  for (unsigned int i = 0; i < nbrOfBytes; i++) {
    if (bytesOnLine == 0) {
      Serial.print((unsigned long)(ptr + i), HEX);
      Serial.print(": ");
    }

    if (*(ptr + i) < 0x10) // print a leading 0
      Serial.print("0");
    Serial.print(*(ptr + i), HEX);
    Serial.print(" ");

    if (bytesOnLine == bytesPerLine) {
      Serial.println(" ");
      bytesOnLine = 0;
    } else {
      bytesOnLine += 1;
    }
  }
  Serial.println("");
}

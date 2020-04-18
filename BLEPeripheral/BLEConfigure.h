/*
 * BLEConfigure.h
 * 
 * Author: thomas.j.petz@gsk.com
 * 
 * Allow the boards configuration information to be managed via BLE.
 * 
 */

#ifndef __BLEConfigure_h__
#define __BLEConfigure_h__

#include <ArduinoBLE.h>

extern BLEService configService;

void config_configService();

// Callbacks, static to avoid function signature problems.
void sensorName_Written(BLEDevice central, BLECharacteristic characteristic);
void sensorLocation_Written(BLEDevice central, BLECharacteristic characteristic);
void humidityGreenLimit_Written(BLEDevice central, BLECharacteristic characteristic);
void humidityAmberLimit_Written(BLEDevice central, BLECharacteristic characteristic);
void lockConfig_Written(BLEDevice central, BLECharacteristic characteristic);
void unlockConfig_Written(BLEDevice central, BLECharacteristic characteristic);

// the system configuration settings need to be added to this struct

typedef struct flash_mem {
  char sensor_name[128];
  char location[128];
  uint16_t  humidityGreenLimit;
  uint16_t  humidityAmberLimit;
  char configLockPassword[64];
  char  reserve[4096 - 2*128 - 4 - 64];        // fill out the remainder of the page so nothing else is allocated in this page
} flash_config_t;


// nFR52 NVMC registers
#define NVMC_BASE       (0x4001E000U)
#define NVMC_READY      (NVMC_BASE + 0x400U)
#define NVMC_READYNEXT  (NVMC_BASE + 0x408U)
#define NVMC_CONFIG     (NVMC_BASE + 0x504U)
#define NVMC_ERASEPAGE  (NVMC_BASE + 0x508U)
#define NVMC_ERASEALL   (NVMC_BASE + 0x50CU)
#define NVMC_ERASEUICR  (NVMC_BASE + 0x514U)
#define NVMC_ERASEPAGEPARTIAL   (NVMC_BASE + 0X518U)
#define NVMC_ERASEPAGEPARTIALCFG  (NVMC_BASE + 0X51CU)
#define NVMC_ICACHECNF  (NVMC_BASE + 0x540U)
#define NVMC_IHIT       (NVMC_BASE + 0x548U)
#define NVMC_IMISS      (NMVC_BASE + 0x54cU)

#endif

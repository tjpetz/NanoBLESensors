#include <Arduino.h>
#include "Watchdog.h"

void initializeWDT(uint32_t timeout) {

  // Configure the Watch Dog Timer
  NRF_WDT->CONFIG = 0x01;
  NRF_WDT->CRV = timeout * 32768 + 1;
  NRF_WDT->RREN = 0x01;
  NRF_WDT->TASKS_START = 1;
}

void resetWDT() {

  // Reset the WDT
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}
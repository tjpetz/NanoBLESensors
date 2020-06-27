/**
 * NRF52840 Watchdog timer
 */

#ifndef _Watchdog_h_
#define _Watchdog_h_

extern void initializeWDT(uint32_t timeout);
extern void resetWDT();

#endif
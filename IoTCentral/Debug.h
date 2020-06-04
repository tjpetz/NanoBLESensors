/*
 * Debug.h
 * 
 * A set of very simple DEBUG macros
 * 
 */

#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef _DEBUG_
#define CDC_ENABLED
#define MAX_DEBUG_BUFF    256
#define DEBUG_PRINTF(...) {char _buff[MAX_DEBUG_BUFF]; snprintf(_buff, MAX_DEBUG_BUFF, __VA_ARGS__); Serial.print(_buff);}
#else
#define DEBUG_PRINTF(buff, fmt, ...)
#endif

#endif

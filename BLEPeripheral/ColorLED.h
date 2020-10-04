#ifndef __COLORLED_H__
#define __COLORLED_H__

#include <Arduino.h>

typedef enum
{
  BLACK = 0x000000,
  RED = 0xff0000,
  YELLOW = 0xffff00,
  GREEN = 0x00ff00,
  CYAN = 0x00ffff,
  BLUE = 0x0000ff,
  PURPLE = 0xff00ff,
  WHITE = 0xffffff
} LEDColors;

class RGBled
{
public:
  RGBled();
  void setColor(const uint8_t r, const uint8_t g, const uint8_t b);
  void setColor(const uint32_t rgb);
  void turnOff();
  void turnOn();

private:
  uint8_t _r;
  uint8_t _g;
  uint8_t _b;
};

#endif

#include "ColorLED.h"

RGBled::RGBled()
{
  _r = 0;
  _g = 0;
  _b = 0;

  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  turnOff();
}

void RGBled::setColor(const uint8_t r, const uint8_t g, const uint8_t b)
{
  _r = r;
  _g = g;
  _b = b;

  analogWrite(LEDR, 255 - _r);
  analogWrite(LEDG, 255 - _g);
  analogWrite(LEDB, 255 - _b);
}

void RGBled::setColor(const uint32_t rgb)
{
  // break out the components from the unsigned long
  _r = (0xFF0000 & rgb) >> 16;
  _g = (0x00FF00 & rgb) >> 8;
  _b = (0x0000FF & rgb);

  analogWrite(LEDR, 255 - _r);
  analogWrite(LEDG, 255 - _g);
  analogWrite(LEDB, 255 - _b);
}

void RGBled::turnOff()
{
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
}

void RGBled::turnOn()
{
  analogWrite(LEDR, 255 - _r);
  analogWrite(LEDG, 255 - _g);
  analogWrite(LEDB, 255 - _b);
}

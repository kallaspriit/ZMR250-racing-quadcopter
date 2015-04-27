#include "Display.h"

Display::Display(int dcPin, int csPin, int rstPin, int mosiPin, int sclkPin) :
  tft(NULL),
  dcPin(dcPin),
  csPin(csPin),
  rstPin(rstPin),
  mosiPin(mosiPin),
  sclkPin(sclkPin)
{
  width = ST7735::width;
  height = ST7735::height;
  
  tft = new ST7735(csPin, dcPin, mosiPin, sclkPin, rstPin);
}

void Display::init() {
  tft->initR();
  tft->writecommand(ST7735_DISPON);
  
  clear();
}

void Display::clear(Color color) {
  tft->fillScreen(color);
}

void Display::drawString(int x, int y, String str, Color color, int size) {
  char buf[str.length() + 1];
  
  str.toCharArray(buf, str.length() + 1);
  
  tft->drawString(x, y, buf, color, size);
}

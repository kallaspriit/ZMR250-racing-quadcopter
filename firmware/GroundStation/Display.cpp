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

void Display::fillRect(int x, int y, int width, int height, Color color) {
  tft->fillRect(x, y, width, height, color);
}

void Display::drawString(int x, int y, String str, Color color, Color bg, int size) {
  char buf[str.length() + 1];
  
  str.toCharArray(buf, str.length() + 1);
  
  if (bg != TRANSPARENT) {
    BoundingBox bb = getStringBoundingBox(x, y, str.length(), size);
    
    fillRect(bb.x, bb.y, bb.width, bb.height, bg);
  }
  
  tft->drawString(x, y, buf, color, size);
}

void Display::drawStringCentered(int x, int y, String string, Display::Color color, Color bg, int size) {
  int stringWidth = calculateStringWidth(string.length(), size);
  int stringHeight = calculateStringHeight(size);
  
  drawString(
    x - stringWidth / 2,
    y - stringHeight / 2,
    string,
    color,
    bg,
    size
  );
}

void Display::drawStringLine(int x, int y, String str, Color color, Color bg, int size) {
  int messageHeight = calculateStringHeight(size);
  
  // paint over entire line
  fillRect(0, y, width, messageHeight, bg);
  drawString(x, y, str, color, TRANSPARENT, size);
}

void Display::drawStringLineCentered(int x, int y, String str, Color color, Color bg, int size) {
  int messageHeight = calculateStringHeight(size);
  
  // paint over entire line
  fillRect(0, y - messageHeight / 2, width, messageHeight, bg);
  drawStringCentered(x, y, str, color, TRANSPARENT, size);
}

Display::BoundingBox Display::getStringBoundingBox(int x, int y, int stringLength, int size) {
  return BoundingBox(
    x,
    y,
    calculateStringWidth(stringLength, size),
    calculateStringHeight(size)
  );
}

Display::BoundingBox Display::getStringBoundingBox(int stringLength, int size) {
  return getStringBoundingBox(0, 0, stringLength, size);
}

int Display::calculateStringWidth(int stringLength, int size) {
  return stringLength * 6 * size;
}

int Display::calculateStringHeight(int size) {
  return 7 * size;
}

Display::Color Display::rgb(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

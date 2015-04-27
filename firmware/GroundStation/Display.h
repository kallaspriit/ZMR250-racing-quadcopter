#ifndef DISPLAY_H
#define DISPLAY_H

#include "ST7735.h"

class Display {

public:
  typedef uint16_t Color;
  
  static const Color WHITE = 0xFFFF;
  static const Color BLACK = 0x0000;
  
  int width;
  int height;

  Display(int dcPin, int csPin, int rstPin, int mosiPin = 11, int sclkPin = 13);
  
  void init();
  void clear(Color color = WHITE);
  void drawString(int x, int y, String str, Color color = BLACK, int size = 1);
  
private:
  ST7735 *tft;
  
  int dcPin;
  int csPin;
  int rstPin;
  int mosiPin = 11;
  int sclkPin;
  
};

#endif // DISPLAY_H

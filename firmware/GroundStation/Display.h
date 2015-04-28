#ifndef DISPLAY_H
#define DISPLAY_H

#include "ST7735.h"

class Display {

public:
  typedef uint16_t Color;
  
  struct BoundingBox {
    BoundingBox(int x, int y, int width, int height) : x(x), y(y), width(width), height(height) {}
    
    int x;
    int y;
    int width;
    int height;
  };
  
  static const Color WHITE = 0xFFFF;
  static const Color BLACK = 0x0000;
  static const Color RED = 0x001F;
  static const Color GREEN = 0x07E0;
  static const Color BLUE = 0xF800;
  static const Color YELLOW = 0xFFE0;
  static const Color GRAY_15 = 0x0861;
  static const Color GRAY_50 = 0x3186;
  static const Color GRAY_128 = 0x8410;
  static const Color GRAY_225 = 0xE71C;
  static const Color TRANSPARENT = 0x10000;
  
  int width;
  int height;

  Display(int dcPin, int csPin, int rstPin, int mosiPin = 11, int sclkPin = 13);
  
  void init();
  void clear(Color color = BLACK);
  
  void fillRect(int x, int y, int width, int height, Color color);
  
  void drawString(int x, int y, String str, Color color = WHITE, Color bg = TRANSPARENT, int size = 1);
  void drawStringCentered(int x, int y, String string, Color color = WHITE, Color bg = TRANSPARENT, int size = 1);
  
  void drawStringLine(int x, int y, String str, Color color = WHITE, Color bg = BLACK, int size = 1);
  void drawStringLineCentered(int x, int y, String string, Color color = WHITE, Color bg = BLACK, int size = 1);
  
  BoundingBox getStringBoundingBox(int x, int y, int stringLength, int size = 1);
  BoundingBox getStringBoundingBox(int stringLength, int size = 1);
  
  int calculateStringWidth(int stringLength, int size = 1);
  int calculateStringHeight(int size = 1);
  
  static Color rgb(uint8_t r, uint8_t g, uint8_t b);
  
private:
  ST7735 *tft;
  
  int dcPin;
  int csPin;
  int rstPin;
  int mosiPin = 11;
  int sclkPin;
  
};

#endif // DISPLAY_H

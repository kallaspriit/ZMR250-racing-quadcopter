#ifndef UI_H
#define UI_H

#include "Arduino.h"
#include "Display.h"

class UI {
  public:
    UI(Display *display);
    
    void clear();
    
    void showLoading(String message);
    
    void drawStringCentered(int x, int y, String string, Display::Color color = 0xFFFF, int size = 1);
    
    int calculateStringWidth(int stringLength, int size = 1);
    int calculateStringHeight(int size = 1);
    
    void testDrawStringCenter(int size);
    
  private:
    Display *display;
};

#endif

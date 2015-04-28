#ifndef UI_H
#define UI_H

#include "Arduino.h"
#include "Display.h"

class UI {
  public:
    UI(Display *display);
    
    void clear();
    
    void showLoading(String message);
    void renderFooter(String monitorState);
    void renderHeader(boolean isBluetoothConnected, float localBatteryVoltage);
    
  private:
    Display *display;
};

#endif

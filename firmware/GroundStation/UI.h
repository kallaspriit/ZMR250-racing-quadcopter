#ifndef UI_H
#define UI_H

#include "Arduino.h"
#include "Display.h"

class UI {
  public:
    UI(Display *display);
    
    void clear();
    
    void showLoading(String message);
    void renderHeader(boolean isBluetoothConnected, float localBatteryVoltage, boolean isChargingBattery);
    void renderFooter(String monitorState);
    
  private:
    Display *display;
};

#endif

#ifndef UI_H
#define UI_H

#include "Arduino.h"
#include "Display.h"

class Device;

class UI {
  public:
    enum Size {
      SMALL = 1,
      LARGE = 2
    };
  
    UI(Display *display);
    
    void clear();
    
    void renderHeader(boolean isBluetoothConnected, float localBatteryVoltage, boolean isChargingBattery);
    void renderFooter(String monitorState);
    
    void renderLoadingView(String message, int size = 2);
    void renderDetailsView(Device *device);
    
  private:
    Display *display;
};

#endif

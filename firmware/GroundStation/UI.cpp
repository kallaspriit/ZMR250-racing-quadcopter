#include "UI.h"

UI::UI(Display *display) : display(display) {
  clear();
};

void UI::clear() {
  display->clear();
}

void UI::showLoading(String message) {
  display->drawStringLineCentered(
    display->width / 2,
    display->height / 2,
    message,
    Display::WHITE,
    Display::BLACK,
    2
  );
}

void UI::renderHeader(boolean isBluetoothConnected, float localBatteryVoltage, boolean isChargingBattery) {
  int size = 1;
  String bluetoothStatusText = "BT";
  int textWidth = display->calculateStringWidth(bluetoothStatusText.length(), size);
  int textHeight = display->calculateStringHeight(size);
  
  // draw header background
  display->fillRect(0, 0, display->width, 15, Display::GRAY_50);
  
  float lowBatteryLevel = 3.8f;
  float criticalBatteryLevel = 3.4f;
  Display::Color batteryColor;
  
  if (localBatteryVoltage <= criticalBatteryLevel) {
    batteryColor = Display::RED;
  } else if (localBatteryVoltage <= lowBatteryLevel) {
    batteryColor = Display::YELLOW;
  } else {
    batteryColor = Display::GREEN;
  }
  
  // draw local battery voltage in top left corner
  display->drawString(
    3,
    4,
    String(localBatteryVoltage) + "V" + (isChargingBattery ? "++" : ""),
    batteryColor,
    Display::BLACK,
    size
  );
  
  // draw bluetooth state in top right corner
  display->drawString(
    display->width - textWidth - 3,
    4,
    bluetoothStatusText,
    isBluetoothConnected ? Display::GREEN : Display::RED,
    Display::BLACK,
    size
  );
}

void UI::renderFooter(String monitorState) {
  int size = 1;
  int lineHeight = display->calculateStringHeight(size);
  
  // draw header background
  display->fillRect(0, display->height - 15, display->width, 15, Display::GRAY_50);
  
  display->drawStringCentered(
    display->width / 2,
    display->height - lineHeight - 1,
    monitorState,
    Display::WHITE,
    Display::BLACK,
    size
  );
}

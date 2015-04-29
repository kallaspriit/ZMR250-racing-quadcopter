#include "UI.h"
#include "Device.h"

UI::UI(Display *display) : display(display) {
  clear();
};

void UI::clear() {
  display->clear();
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

void UI::renderLoadingView(String message, int size) {
  display->drawStringLineCentered(
    display->width / 2,
    display->height / 2,
    message,
    Display::WHITE,
    Display::BLACK,
    size
  );
}

void UI::renderDetailsView(Device *device) {
  String pitchText = "";
  pitchText += "Pitch: ";
  pitchText += device->pitch;
  pitchText += " deg";
  
  String yawText = "";
  yawText += "Yaw: ";
  yawText += device->yaw;
  yawText += " deg";
  
  String rollText = "";
  rollText += "Roll: ";
  rollText += device->roll;
  rollText += " deg";
  
  String voltageText = "";
  voltageText += "Voltage: ";
  voltageText += device->voltage;
  voltageText += "V";
  
  String currentText = "";
  currentText += "Current: ";
  currentText += device->current > 0.0f ? (String(device->current) + "A") : "n/a";
  
  int lineHeight = 10;
  int posY = 20 - lineHeight;
  
  display->drawStringLine(4, posY += lineHeight, pitchText);
  display->drawStringLine(4, posY += lineHeight, yawText);
  display->drawStringLine(4, posY += lineHeight, rollText);
  display->drawStringLine(4, posY += lineHeight, voltageText);
  display->drawStringLine(4, posY += lineHeight, currentText);
}

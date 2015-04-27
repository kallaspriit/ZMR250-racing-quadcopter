#include "UI.h"

UI::UI(Display *display) : display(display) {

};

void UI::clear() {
  display->clear(Display::BLACK);
}

void UI::showLoading(String message) {
  clear();
  
  drawStringCentered(display->width / 2, display->height / 2, message, Display::WHITE, 2);
}

void UI::drawStringCentered(int x, int y, String string, Display::Color color, int size) {
  int stringWidth = calculateStringWidth(string.length(), size);
  int stringHeight = calculateStringHeight(size);
  
  display->drawString(
    x - stringWidth / 2,
    y - stringHeight / 2,
    string,
    color,
    size
  );
}

int UI::calculateStringWidth(int stringLength, int size) {
  return stringLength * 6 * size;
}

int UI::calculateStringHeight(int size) {
  return 7 * size;
}

void UI::testDrawStringCenter(int size) {
  clear();
  
  char buf[100];
  
  for (int i = 0; i < 20; i++) {
    buf[i] = 'X';
    buf[i + 1] = '\0';
    
    String str(buf);
    
    drawStringCentered(display->width / 2, i * 8 * size, str, Display::WHITE, size);
  }
}

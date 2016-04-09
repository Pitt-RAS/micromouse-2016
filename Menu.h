#ifndef MENU_H
#define MENU_H

#include <Arduino.h>
#include <LedDisplay.h>

#include "data.h"

class Menu {
 private:
  LedDisplay display_;
  bool initialized_ = false;

 public:
  Menu();
  void begin();

  void showInt(int value, int d);
  void showString(const char* s, int chars = -1, bool left_align = true);

  // gets a integer between min and max with at most d digits
  int getInt(int min, int max, int initial, int d);

  // picks a string out of an array and returns the index selected
  size_t getString(const char* strings[], size_t strings_len, size_t chars, size_t initial = 0,
                   bool left_align = true);

  bool buttonOkPressed();
  bool buttonBackPressed();

  void soundBuzzer(int frequency);
  void checkBattery();
  void waitForHand();

  void storeDefaultDirection(Compass8 dir);
  Compass8 loadDefaultDirection();
};

extern Menu menu;

#endif

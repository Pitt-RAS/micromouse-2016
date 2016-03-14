#ifndef MENU_H
#define MENU_H

#include <Arduino.h>
#include <LedDisplay.h>

class Menu {
 private:
  LedDisplay display_;
  static bool initialized_;

  Menu();

  void showInt(int value, int d);
  void showString(char* s, bool left_align = true);

 public:
  static void initialize();

  // gets a integer between min and max with at most d digits
  int getInt(int min, int max, int initial, int d);

  // picks a string out of an array and returns the index selected
  size_t getString(char* strings[], size_t strings_len, size_t initial = 0,
                   bool left_align = false);

  bool buttonOkPressed();
  bool buttonBackPressed();
};

extern Menu menu;

#endif

#include "Menu.h"

#include <Arduino.h>

#include "conf.h"
#include "motors.h"
#include "PIDController.h"
#include "sensors_encoders.h"

Menu menu;

Menu::Menu()
    : display_(DISPLAY_DATA_PIN, DISPLAY_RS_PIN, DISPLAY_CLOCK_PIN,
               DISPLAY_ENABLE_PIN, DISPLAY_RESET_PIN, DISPLAY_SIZE) {
}

void Menu::showInt(int value, int d) {
  char buf[d + 1];
  int chars = snprintf(buf, d + 1, "%i", value);
  display_.setCursor(DISPLAY_SIZE - d);
  for (int i = 0; i < d - chars; i++) {
      display_.write(' ');
  }
  for (int i = 0; i < chars; i++) {
    display_.write(buf[i]);
  }
}

void Menu::showString(char* s, int chars, bool left_align) {
  int len = strlen(s);

  if (chars < 0) {
    chars = len;
  }

  if (left_align) {
    display_.setCursor(0);
  } else {
    display_.setCursor(max(0, DISPLAY_SIZE - chars));
  }

  if (!left_align) {
    for (int i = 0; i < min(chars, DISPLAY_SIZE) - len; i++) {
      display_.write(' ');
    }
  }
  for (int i = 0; i < min(len, DISPLAY_SIZE); i++) {
    display_.write(s[i]);
  }
  if (left_align) {
    for (int i = 0; i < min(chars, DISPLAY_SIZE) - len; i++) {
      display_.write(' ');
    }
  }
}

void Menu::begin() {
  if (!initialized_) {
    display_.setBrightness(DISPLAY_BRIGHTNESS);
    display_.begin();
    initialized_ = true;
  }
}

int Menu::getInt(int min, int max, int initial, int d) {
  float distance_between_options = MENU_STEP_ANGLE * DEG_TO_RAD * WHEEL_RADIUS;
  int result = initial;
  enc_right_write(0);
  showInt(result, d);

  motor_l.Set(0, 0);

  bool okPressed = false;
  bool backPressed = false;
  while (!okPressed && !backPressed) {
    float distance_from_center = enc_right_extrapolate();

    if (distance_from_center > distance_between_options / 2) {
      if (result < max && result + 1 < pow(10, d)) {
        enc_right_write(-distance_between_options / 2);
        result++;
        showInt(result, d);
      }
    } else if (distance_from_center < -distance_between_options / 2) {
      if (result > min && result - 1 > -pow(10, d - 1)) {
        enc_right_write(distance_between_options / 2);
        result--;
        showInt(result, d);
      }
    }

    if (abs(enc_right_extrapolate()) < MENU_DEAD_ZONE) {
        motor_r.Set(0, enc_right_velocity());
    } else if (enc_right_extrapolate() < 0) {
        motor_r.Set(MENU_RESTORING_FORCE, enc_right_velocity());
    } else {
        motor_r.Set(-MENU_RESTORING_FORCE, enc_right_velocity());
    }

    okPressed = buttonOkPressed();
    backPressed = buttonBackPressed();
  }

  if (okPressed) {
    return result;
  } else {
    return initial;
  }
}

size_t Menu::getString(char* strings[], size_t strings_len, size_t chars, size_t initial, bool left_align) {
  float distance_between_options = MENU_STEP_ANGLE * DEG_TO_RAD * WHEEL_RADIUS;
  size_t result = initial;
  enc_right_write(0);
  showString(strings[result], left_align);

  motor_l.Set(0, 0);

  bool okPressed = false;
  bool backPressed = false;
  while (!okPressed && !backPressed) {
    float distance_from_center = enc_right_extrapolate();

    if (distance_from_center > distance_between_options / 2) {
      if (result < strings_len - 1) {
        enc_right_write(-distance_between_options / 2);
        result++;
        showString(strings[result], chars, left_align);
      }
    } else if (distance_from_center < -distance_between_options / 2) {
      if (result > 0) {
        enc_right_write(distance_between_options / 2);
        result--;
        showString(strings[result], chars, left_align);
      }
    }

    if (abs(enc_right_extrapolate()) < MENU_DEAD_ZONE) {
        motor_r.Set(0, enc_right_velocity());
    } else if (enc_right_extrapolate() < 0) {
        motor_r.Set(MENU_RESTORING_FORCE, enc_right_velocity());
    } else {
        motor_r.Set(-MENU_RESTORING_FORCE, enc_right_velocity());
    }

    okPressed = buttonOkPressed();
    backPressed = buttonBackPressed();
  }

  if (okPressed) {
    return result;
  } else {
    return initial;
  }
}

bool Menu::buttonOkPressed() {
  return (digitalRead(BUTTON_OK_PIN) == LOW);
}

bool Menu::buttonBackPressed() {
  return (digitalRead(BUTTON_BACK_PIN) == LOW);
}

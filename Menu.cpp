#include "Menu.h"

#include <Arduino.h>

#include "conf.h"
#include "motors.h"
#include "PIDController.h"
#include "sensors_encoders.h"

bool Menu::initialized_ = false;

Menu::Menu()
    : display_(DISPLAY_DATA_PIN, DISPLAY_RS_PIN, DISPLAY_CLOCK_PIN,
               DISPLAY_ENABLE_PIN, DISPLAY_RESET_PIN, DISPLAY_SIZE) {
  display_.setBrightness(DISPLAY_BRIGHTNESS);
  display_.begin();
}

void Menu::showInt(int value, int d) {
  char buf[d + 1];
  int chars = snprintf(buf, d + 1, "%i", value);
  display_.setCursor(DISPLAY_SIZE - chars);
  for (int i = 0; i < chars; i++) {
    display_.write(buf[i]);
  }
}

void Menu::showString(char* s, bool left_align) {
  int len = strlen(s);

  if (left_align) {
    display_.setCursor(0);
  } else {
    display_.setCursor(max(0, DISPLAY_SIZE - len));
  }

  for (int i = 0; i < min(len, DISPLAY_SIZE); i++) {
    display_.write(s[i]);
  }
}

void Menu::initialize() {
  if (!initialized_) {
    menu = Menu();
    initialized_ = true;
  }
}

int Menu::getInt(int min, int max, int initial, int d) {
  float distance_between_options = MENU_STEP_ANGLE * DEG_TO_RAD * WHEEL_RADIUS;
  int result = initial;
  PIDController pid (MENU_KP, MENU_KI, MENU_KD);
  enc_right_write(0);
  showInt(result, d);

  while (!(buttonOkPressed() || buttonBackPressed())) {
    float distance_from_center = enc_right_extrapolate();

    if (distance_from_center > distance_between_options / 2) {
      if (result < max && result + 1 < pow(10, d)) {
        enc_right_write(-distance_between_options / 2);
        result++;
        showInt(result, d);

        // skip over the derivative spike from changing the setpoint
        pid.Calculate(enc_right_extrapolate());
      }
    } else if (distance_from_center < -distance_between_options / 2) {
      if (result > min && result - 1 > -pow(10, d - 1)) {
        enc_right_write(distance_between_options / 2);
        result--;
        showInt(result, d);

        // skip over the derivative spike from changing the setpoint
        pid.Calculate(enc_right_extrapolate());
      }
    }

    float error = pid.Calculate(enc_right_extrapolate());
    motor_r.Set(error, enc_right_velocity());
  }

  return result;
}

size_t Menu::getString(char* strings[], size_t strings_len, size_t initial, bool left_align) {
  float distance_between_options = MENU_STEP_ANGLE * DEG_TO_RAD * WHEEL_RADIUS;
  size_t result = initial;
  PIDController pid (MENU_KP, MENU_KI, MENU_KD);
  enc_right_write(0);
  showString(strings[result], left_align);

  while (!(buttonOkPressed() || buttonBackPressed())) {
    float distance_from_center = enc_right_extrapolate();

    if (distance_from_center > distance_between_options / 2) {
      if (result < strings_len - 1) {
        enc_right_write(-distance_between_options / 2);
        result++;
        showString(strings[result], left_align);

        // skip over the derivative spike from changing the setpoint
        pid.Calculate(enc_right_extrapolate());
      }
    } else if (distance_from_center < -distance_between_options / 2) {
      if (result > 0) {
        enc_right_write(distance_between_options / 2);
        result--;
        showString(strings[result], left_align);

        // skip over the derivative spike from changing the setpoint
        pid.Calculate(enc_right_extrapolate());
      }
    }

    float error = pid.Calculate(enc_right_extrapolate());
    motor_r.Set(error, enc_right_velocity());
  }

  return result;
}

bool Menu::buttonOkPressed() {
  return (digitalRead(BUTTON_OK_PIN) == LOW);
}

bool Menu::buttonBackPressed() {
  return (digitalRead(BUTTON_BACK_PIN) == LOW);
}

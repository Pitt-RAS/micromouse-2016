#include "Menu.h"

#include <Arduino.h>

#include "conf.h"
#include "motors.h"
#include "PIDController.h"
#include "RangeSensorContainer.h"
#include "sensors_encoders.h"
#include "sensors_orientation.h"

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
  for (int i = 0; i < min(len, min(chars, DISPLAY_SIZE)); i++) {
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
  enc_left_back_write(0);
  showInt(result, d);

  motor_lf.Set(0, 0);
  motor_rf.Set(0, 0);
  motor_rb.Set(0, 0);

  bool okPressed = false;
  bool backPressed = false;
  while (!okPressed && !backPressed) {
    float distance_from_center = enc_left_back_extrapolate();

    if (distance_from_center > distance_between_options / 2) {
      if (result < max && result + 1 < pow(10, d)) {
        enc_left_back_write(-distance_between_options / 2);
        result++;
        showInt(result, d);
      }
    } else if (distance_from_center < -distance_between_options / 2) {
      if (result > min && result - 1 > -pow(10, d - 1)) {
        enc_left_back_write(distance_between_options / 2);
        result--;
        showInt(result, d);
      }
    }

    if (abs(enc_left_back_extrapolate()) < MENU_DEAD_ZONE) {
        motor_lb.Set(0, enc_left_back_velocity());
    } else if (enc_left_back_extrapolate() < 0) {
        motor_lb.Set(MENU_RESTORING_FORCE, enc_left_back_velocity());
    } else {
        motor_lb.Set(-MENU_RESTORING_FORCE, enc_left_back_velocity());
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
  enc_left_back_write(0);
  showString(strings[result], chars, left_align);

  motor_lf.Set(0, 0);
  motor_rf.Set(0, 0);
  motor_rb.Set(0, 0);

  bool okPressed = false;
  bool backPressed = false;
  while (!okPressed && !backPressed) {
    float distance_from_center = enc_left_back_extrapolate();

    if (distance_from_center > distance_between_options / 2) {
      if (result < strings_len - 1) {
        enc_left_back_write(-distance_between_options / 2);
        result++;
        showString(strings[result], chars, left_align);
      }
    } else if (distance_from_center < -distance_between_options / 2) {
      if (result > 0) {
        enc_left_back_write(distance_between_options / 2);
        result--;
        showString(strings[result], chars, left_align);
      }
    }

    if (abs(enc_left_back_extrapolate()) < MENU_DEAD_ZONE) {
        motor_lb.Set(0, enc_left_back_velocity());
    } else if (enc_left_back_extrapolate() < 0) {
        motor_lb.Set(MENU_RESTORING_FORCE, enc_left_back_velocity());
    } else {
        motor_lb.Set(-MENU_RESTORING_FORCE, enc_left_back_velocity());
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

void Menu::soundBuzzer(int frequency)
{
  if (frequency == 0) {
    analogWrite(BUZZER_PIN, 0);
    return;
  }

  analogWriteFrequency(BUZZER_PIN, frequency);
  analogWrite(BUZZER_PIN, PWM_SPEED_STEPS / 2);
}

void Menu::checkBattery()
{
  char buf[5];
  int whole;
  int decimal;

  float voltage = (8.225 / 6.330) * (26 / 10) * (3.3 / 1023) * analogRead(BATTERY_PIN);

  if (voltage < BATTERY_VOLTAGE_WARNING)
    soundBuzzer(2000);

  whole = (int) voltage;
  decimal = (int) ((voltage - whole) * 100);

  if (whole > 9 || decimal > 99) {
    whole = 0;
    decimal = 0;
  }

  sprintf(buf, "%0d.%02d", whole, decimal);
  menu.showString(buf);
}

void Menu::waitForHand()
{
  Orientation* orientation = Orientation::getInstance();
  float initial_heading = orientation->getHeading();
  uint32_t time = millis();
  while (millis() - time < HAND_SWIPE_START_TIME) {
    float delta_heading = abs(orientation->getHeading() - initial_heading);
    if (delta_heading > HAND_SWIPE_HEADING_TOLERANCE) {
      time = millis();
      initial_heading = orientation->getHeading();
    }
  }

  do {
    RangeSensors.frontRightSensor.updateRange();
    RangeSensors.diagRightSensor.updateRange();
  } while (RangeSensors.frontRightSensor.getRange() > HAND_SWIPE_FORWARD_RANGE
            || RangeSensors.diagRightSensor.getRange() > HAND_SWIPE_DIAG_RANGE);

  do {
    RangeSensors.frontRightSensor.updateRange();
    RangeSensors.diagRightSensor.updateRange();
  } while (RangeSensors.frontRightSensor.getRange() < HAND_SWIPE_FORWARD_RANGE
            || RangeSensors.diagRightSensor.getRange() < HAND_SWIPE_DIAG_RANGE);
}

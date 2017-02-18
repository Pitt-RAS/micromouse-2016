#include "UserInterface.h"

#include <Arduino.h>

// Dependencies within Micromouse
#include "Orientation.h"
#include "PIDController.h"
#include "PlayMelodies.h"
#include "RangeSensorContainer.h"
#include "conf.h"
#include "motors.h"
#include "sensors_encoders.h"
#include "UserInterface.h"

UserInterface gUserInterface;

char gWorkaroundBuffer[9] = "--------";

UserInterface::UserInterface()
    : display_(DISPLAY_DATA_PIN, DISPLAY_RS_PIN, DISPLAY_CLOCK_PIN,
               DISPLAY_ENABLE_PIN, DISPLAY_RESET_PIN, DISPLAY_SIZE)
{
  // workaround for memory bug in older versions of LedDisplay library
  display_.setString(gWorkaroundBuffer);
}

void UserInterface::showInt(int value, int d) {
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

void UserInterface::showString(const char* s, int chars, bool left_align) {
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

void UserInterface::begin() {
  if (!initialized_) {
    display_.setBrightness(DISPLAY_BRIGHTNESS);
    display_.begin();
    initialized_ = true;
  }
}

int UserInterface::getInt(int min, int max, int initial, int d) {
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
    } else {
        motor_lb.Set(-MENU_RESTORING_FORCE * enc_left_back_extrapolate(),
                     enc_left_back_velocity());
    }

    okPressed = buttonOkPressed();
    backPressed = buttonBackPressed();
  }

  motor_lb.Set(0, 0);

  if (okPressed) {
    delay(100);
    while (buttonOkPressed()) {
      // Wait for button release
    }
    delay(100);
    return result;
  } else {
    delay(100);
    while (buttonBackPressed()) {
      // Wait for button release
    }
    delay(100);
    return initial;
  }
}

size_t UserInterface::getString(const char* strings[], size_t strings_len, size_t chars, size_t initial, bool left_align) {
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
      if (result > 0) {
        enc_left_back_write(-distance_between_options / 2);
        result--;
        showString(strings[result], chars, left_align);
      }
    } else if (distance_from_center < -distance_between_options / 2) {
      if (result < strings_len - 1) {
        enc_left_back_write(distance_between_options / 2);
        result++;
        showString(strings[result], chars, left_align);
      }
    }

    if (abs(enc_left_back_extrapolate()) < MENU_DEAD_ZONE) {
        motor_lb.Set(0, enc_left_back_velocity());
    } else {
        motor_lb.Set(-MENU_RESTORING_FORCE * enc_left_back_extrapolate(),
                     enc_left_back_velocity());
    }

    okPressed = buttonOkPressed();
    backPressed = buttonBackPressed();
  }

  motor_lb.Set(0, 0);

  if (okPressed) {
    delay(100);
    while (buttonOkPressed()) {
      // Wait for button release
    }
    delay(100);
    return result;
  } else {
    delay(100);
    while (buttonBackPressed()) {
      // Wait for button release
    }
    delay(100);
    return initial;
  }
}

bool UserInterface::buttonOkPressed() {
  return (digitalRead(BUTTON_OK_PIN) == LOW);
}

bool UserInterface::buttonBackPressed() {
  return (digitalRead(BUTTON_BACK_PIN) == LOW);
}

void UserInterface::soundBuzzer(int frequency)
{
  static bool on = false;

  if (frequency == 0) {
    analogWrite(BUZZER_PIN, 0);
    on = false;
    return;
  }

  if (!on) {
    analogWriteFrequency(BUZZER_PIN, frequency);
    analogWrite(BUZZER_PIN, PWM_SPEED_STEPS / 2);
    on = true;
  }
}

void UserInterface::checkBattery()
{
  char buf[5];
  int whole;
  int decimal;

  float voltage = (8.225 / 6.330) * (26 / 10) * (3.3 / 1023) * analogRead(BATTERY_PIN);

  if (voltage < BATTERY_VOLTAGE_WARNING) {
    tone(BUZZER_PIN, 2000);
    delay(1000);
    analogWriteFrequency(MOTOR_LF_PWM_PIN, 46875);
    analogWriteFrequency(MOTOR_RF_PWM_PIN, 46875);
    analogWriteFrequency(MOTOR_LB_PWM_PIN, 46875);
    analogWriteFrequency(MOTOR_RB_PWM_PIN, 46875);
  }

  whole = (int) voltage;
  decimal = (int) ((voltage - whole) * 100);

  if (whole > 9 || decimal > 99) {
    whole = 0;
    decimal = 0;
  }

  sprintf(buf, "%0d.%02d", whole, decimal);
  gUserInterface.showString(buf);
}

void UserInterface::waitForHand()
{
  Orientation* orientation = Orientation::getInstance();
  float heading = orientation->getHeading();
  bool readyToStart = false;
  int state = 0;

  while (!readyToStart) {

    checkBattery();
    RangeSensors.frontRightSensor.updateRange();
    RangeSensors.diagRightSensor.updateRange();

    float delta_heading = abs(heading - orientation->getHeading());

    if ( state == 0 ) { // Waiting for hand
      // Robot has moved significantly
      if (delta_heading > HAND_SWIPE_HEADING_TOLERANCE)
        state = 2;
      else if ( RangeSensors.frontRightSensor.getRange() < HAND_SWIPE_FORWARD_RANGE // We see a hand
                && RangeSensors.diagRightSensor.getRange() < HAND_SWIPE_DIAG_RANGE ) {
                  state = 1;
                  handSeenMelody();
      }

    }
    else if ( state == 1 ) { // We saw a hand, now wait for it to go away
      // Robot has moved significantly
      if (delta_heading > HAND_SWIPE_HEADING_TOLERANCE) {
        state = 2;
      }
      else if ( RangeSensors.frontRightSensor.getRange() > HAND_SWIPE_FORWARD_RANGE
                && RangeSensors.diagRightSensor.getRange() > HAND_SWIPE_DIAG_RANGE )
              readyToStart = true;
    }
    else if ( state == 2 ) { // Robot moved
      delay(100);
      state = 0; // Back to waiting for hand
    }
    heading = orientation->getHeading();

    delay(100);
  }
}

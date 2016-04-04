#include <Arduino.h>

#include <LedDisplay.h>
#include "conf.h"
#include "data.h"
#include "driver.h"
#include "Logger.h"
#include "Navigator.h"
#include "Menu.h"
#include "motion.h"
#include "RangeSensorContainer.h"
#include "motors.h"
#include "sensors_encoders.h"
#include "sensors_orientation.h"
#include "EncoderMod.h"
#include "IdealSweptTurns.h"
#include <I2Cdev.h>
#include <MPU9150.h>

void setup()
{
  // Set higher pwm frequency for smoother motor control.
  analogWriteFrequency(MOTOR_LF_PWM_PIN, 46875);
  analogWriteFrequency(MOTOR_RF_PWM_PIN, 46875);
  analogWriteFrequency(MOTOR_LB_PWM_PIN, 46875);
  analogWriteFrequency(MOTOR_RB_PWM_PIN, 46875);

  // PWM resolution is 0-1023.
  analogWriteResolution(10);

  pinMode(EMITTER_DIAG_LEFT_PIN, OUTPUT);
  pinMode(EMITTER_DIAG_RIGHT_PIN, OUTPUT);
  pinMode(EMITTER_FRONT_LEFT_PIN, OUTPUT);
  pinMode(EMITTER_FRONT_RIGHT_PIN, OUTPUT);

  digitalWrite(EMITTER_DIAG_LEFT_PIN, LOW);
  digitalWrite(EMITTER_DIAG_RIGHT_PIN, LOW);
  digitalWrite(EMITTER_FRONT_LEFT_PIN, LOW);
  digitalWrite(EMITTER_FRONT_RIGHT_PIN, LOW);

  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);

  pinMode(13, OUTPUT);
  digitalWrite(13, 0);

  Serial.begin(BAUD);

  menu.begin();
}

char* primary_options[] = {
  "RUN",
  "OPT"
};

char* secondary_options[] = {
  "CLR",
  "BACK"
};

void loop()
{
  switch (menu.getString(primary_options, 2, 4)) {
    case 0: { // RUN
      Navigator<ContinuousRobotDriverRefactor> navigator;
      Orientation* orientation = Orientation::getInstance();

      menu.waitForHand();
      delay(1000);

      enc_left_front_write(0);
      enc_right_front_write(0);
      enc_left_back_write(0);
      enc_right_back_write(0);
      orientation->resetHeading();

      navigator.runDevelopmentCode();
      break;
    }
    case 1: { // OPT
      delay(500);
      switch (menu.getString(secondary_options, 2, 4)) {
        case 0: { // CLR
          RobotDriver driver;
          driver.clearState();
          break;
        }
        case 1: // BACK
        default: {
          break;
        }
      }
      break;
    }
    default: {
      break;
    }
  }
  delay(1000);
}

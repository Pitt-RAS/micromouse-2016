#include <Arduino.h>

// External libraries
#include <I2CdevPittMicromouse.h>
#include <LedDisplay.h>
#include <MPU9150PittMicromouse.h>
#include <EncoderPittMicromouse.h>

// Dependencies within Micromouse
#include "IdealSweptTurns.h"
#include "Logger.h"
#include "Menu.h"
#include "Navigator.h"
#include "Orientation.h"
#include "PersistantStorage.h"
#include "PlayMelodies.h"
#include "RangeSensorContainer.h"
#include "conf.h"
#include "data.h"
#include "driver.h"
#include "motion.h"
#include "motors.h"
#include "parser.h"
#include "sensors_encoders.h"
#include "utility.h"

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

  Menu menu;
  for (;;) menu.main();
}

void loop() {}

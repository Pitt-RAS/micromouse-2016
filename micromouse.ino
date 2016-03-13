#include <Arduino.h>
#include <LedDisplay.h>
#include "conf.h"
#include "data.h"
#include "driver.h"
#include "Navigator.h"
#include "Menu.h"
#include "motion.h"
#include "RangeSensorContainer.h"
#include "motors.h"
#include "sensors_encoders.h"
#include "EncoderMod.h"
#include "SweptTurnTable.h"

#define BAUD 9600

void setup()
{
  // Set higher pwm frequency for smoother motor control.
  analogWriteFrequency(MOTOR_LF_PWM_PIN, 46875);
  analogWriteFrequency(MOTOR_RF_PWM_PIN, 46875);
  analogWriteFrequency(MOTOR_LB_PWM_PIN, 46875);
  analogWriteFrequency(MOTOR_RB_PWM_PIN, 46875);

  // PWM resolution is 0-1023.
  analogWriteResolution(10);

  pinMode(EMITTER1_PIN, OUTPUT);
  pinMode(EMITTER2_PIN, OUTPUT);
  pinMode(EMITTER3_PIN, OUTPUT);
  pinMode(EMITTER4_PIN, OUTPUT);

  digitalWrite(EMITTER1_PIN, LOW);
  digitalWrite(EMITTER2_PIN, LOW);
  digitalWrite(EMITTER3_PIN, LOW);
  digitalWrite(EMITTER4_PIN, LOW);

  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  menu.begin();
}

void loop()
{
  Navigator<RobotDriver> navigator;

  // Wait for button press.
  while (digitalRead(BUTTON1_PIN) == HIGH);
  delay(1000);
  
  enc_left_write(0);
  enc_right_write(0);

  navigator.runDevelopmentCode();
}

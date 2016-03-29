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
#include "sensors_orientation.h"
#include "EncoderMod.h"
#include "IdealSweptTurns.h"
#include <I2Cdev.h>
#include <MPU9150.h>

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
  menu.checkBattery();

}

void loop()
{
  Navigator<RobotDriver> navigator;
  Orientation* orientation = Orientation::getInstance();
  

  // Wait for button press.
  while (digitalRead(BUTTON1_PIN) == HIGH)
    menu.checkBattery();
  delay(1000);
  
  enc_left_front_write(0);
  enc_right_front_write(0);
  enc_left_back_write(0);
  enc_right_back_write(0);
  orientation->resetHeading();

  motion_forward(30.0,0.0);
  motion_hold(1000);
  
  //motion_forward(180.0*1.5,0.2);
  //motion_corner(kLeftTurn90, 0.2);
  //motion_forward(180.0*4+30,0.0);
  //motion_rotate(360);
  //motion_hold(10);
  
  navigator.runDevelopmentCode();
  motion_hold(10);
}

#include <Arduino.h>
#include "conf.h"
#include "data.h"
#include "driver.h"
#include "motion.h"
#include "RangeSensorContainer.h"
#include "motors.h"
#include "sensors_encoders.h"
#include "EncoderMod.h"

#define BAUD 9600

void setup()
{
  // Set higher pwm frequency for smoother motor control.
  analogWriteFrequency(MOTOR_AP_PIN, 46875);
  analogWriteFrequency(MOTOR_BP_PIN, 46875);

  // PWM resolution is 0-1023.
  analogWriteResolution(10);

  pinMode(BUTTON1_PIN, INPUT);
}

void loop()
{
  // Wait for button press.
  while (digitalRead(BUTTON1_PIN) == LOW);
  delay(1000);
  
  enc_left_write(0);
  enc_right_write(0);

  //SerialDriver serial_driver;
  //serial_driver.setSleepTime(200);
  //Driver &driver = serial_driver;

  RobotDriver robot_driver;
  Driver &driver = robot_driver;

  driver.move(kNorth, 3);
  driver.move(kEast, 1);
  driver.move(kWest, 1);
  driver.move(kSouth, 1);
  driver.move(kEast, 2);
  driver.move(kNorth, 1);
}

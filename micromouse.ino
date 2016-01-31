#include "EncoderMod.h"
#include "conf.h"
#include "motors.h"
#include "motion.h"
#include "sensors_encoders.h"
#include "RangeSensorContainer.h"

#define BAUD 9600

int incomingByte = 0;
void setup()
{
  analogWriteFrequency(MOTOR_AP_PIN, 46875);  // Set higher pwm frequency for smoother motor control
  analogWriteFrequency(MOTOR_BP_PIN, 46875);  // Set higher pwm frequency for smoother motor control
  analogWriteResolution(10);  // PWM resolution is 0-1023

  // initialize all classes
  Serial2.begin(BAUD);
  
  pinMode(BUTTON1_PIN, INPUT);
  while (digitalRead(BUTTON1_PIN) == LOW);
  delay(1000);
  
  enc_left_write(0);
  enc_right_write(0);

  motion_forward(600, 0);
  motion_hold(100);

}

void loop()
{
}

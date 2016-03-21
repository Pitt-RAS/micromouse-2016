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
#include "IdealSweptTurns.h"

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

  while (digitalRead(BUTTON1_PIN) == HIGH);
}

void loop()
{
  int dl, dr, fl, fr;
  int i;

  int n = 1;

  while (true) {
    for (i = 0; i < 1000; i++) {
      RangeSensors.updateReadings(n);

      dl = RangeSensors.diagLeftSensor.getRange();
      dr = RangeSensors.diagRightSensor.getRange();
      fl = RangeSensors.frontLeftSensor.getRange();
      fr = RangeSensors.frontRightSensor.getRange();

      if (digitalRead(BUTTON2_PIN) == LOW) {
        delay(50);

        while (digitalRead(BUTTON2_PIN) == LOW);
        delay(50);

        n++;

        if (n > 4)
          n = 1;

        break;
      }
    }

    Serial.printf("dl = %3d  dr = %3d  fl = %3d  fr = %3d\n", dl, dr, fl, fr);
  }
}

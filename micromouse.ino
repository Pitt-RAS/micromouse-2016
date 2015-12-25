#include "EncoderMod.h"
#include "conf.h"
#include "motors.h"
#include "motion.h"
#include "sensors_encoders.h"
#include "sensors_range.h"

#define BAUD 9600

extern RangeSensor RangeSensor;

int incomingByte = 0;
void setup()
{
  analogWriteFrequency(MOTOR_AP, 46875);  // Set higher pwm frequency for smoother motor control
  analogWriteFrequency(MOTOR_BP, 46875);  // Set higher pwm frequency for smoother motor control
  analogWriteResolution(10);  // PWM resolution is 0-1023

  // initialize all classes
  motors_init();


  Serial2.begin(BAUD);

  pinMode(BUTTON1, INPUT);
  while (digitalRead(BUTTON1) == LOW);
  delay(1000);
  
  enc_left_write(0);
  enc_right_write(0);

}

void loop() {


  Serial2.print("diag_left=");
  Serial2.print(RangeSensor.GetRange(RANGE_DIAG_LEFT),3);
  Serial2.print("  front=");
  Serial2.print(RangeSensor.GetRange(RANGE_FRONT),3);
  Serial2.print("  diag_right=");
  Serial2.print(RangeSensor.GetRange(RANGE_DIAG_RIGHT),3);
  Serial2.print("  right=");
  Serial2.print(RangeSensor.GetRange(RANGE_RIGHT),3);
  Serial2.print("  left=");
  Serial2.println(RangeSensor.GetRange(RANGE_LEFT),3);
  delay(50);
  







  //  //Serial2.println(enc_left_velocity());
  //  if (Serial2.available() > 0) {
  //    // read the incoming byte:
  //    incomingByte = Serial2.parseInt();
  //
  //
  //    if (incomingByte != 0) {
  //      // say what you got:
  //      Serial2.print("Current Speed =");
  //      Serial2.println(incomingByte, DEC);
  //      float motorSpeed = (float)incomingByte / 1023;
  //      motor_set(&motor_a, motorSpeed);
  //    }
  //  }











}


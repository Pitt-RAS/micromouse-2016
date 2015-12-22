#include <EncoderMod.h>

#include "conf.h"
#include "motors.h"
#include "motion.h"
#include "sensors_encoders.h"
#include "sensors_range.h"

#define BAUD 9600


int incomingByte = 0;
void setup()
{
  analogWriteFrequency(MOTOR_AP, 46875);  // Set higher pwm frequency for smoother motor control
  analogWriteFrequency(MOTOR_BP, 46875);  // Set higher pwm frequency for smoother motor control
  analogWriteResolution(10);  // PWM resolution is 0-1023
  
  motors_init();

  
  Serial2.begin(BAUD);
  
  pinMode(BUTTON1, INPUT);
  while (digitalRead(BUTTON1) == LOW);
  delay(1000);
  enc_left_write(0);
  enc_right_write(0);

}

void loop() {


  RangeSensor::read(RANGE_FRONT);
  delay(100);
  
//  Serial2.print("\t");
//  Serial2.print(RangeSensor::read(RANGE_FRONT_LEFT));
//  Serial2.print("\t");
//  Serial2.print(RangeSensor::read(RANGE_FRONT_RIGHT));
//  Serial2.print("\t");
//  Serial2.print(RangeSensor::read(RANGE_LEFT));
//  Serial2.print("\t");
//  Serial2.print(RangeSensor::read(RANGE_RIGHT));
//  Serial2.println();
  
//  motion_forward(700,0);
//  motion_rotate(90);






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




//    if ((elapsedTimeNow > 1000) && (elapsedTimeNow < 1100)){
//      velocityTimer = 0;
//      enc_left_write(0);
//    }
//    else if ((elapsedTimeNow > 10000) && (elapsedTimeNow < 10100)) {
//      if (distance == 0) {
//      int elapsedVelocityTime = velocityTimer;
//      distance = enc_left_read();
//      Serial2.println(distance / 1000 / (velocityTimer / 1000000), 5);
//      }
//    }
//    else if (elapsedTimeNow > 10100) {
//      motor_set(&motor_a, 0);
//    }



    
  //  Serial2.print(enc_left_read());
  //  Serial2.print(" ");
  //  Serial2.print(enc_left_extrapolate());
  //  Serial2.print(" ");
  //  Serial2.println(enc_left_velocity());
  //  Serial2.print(" ");
  //  Serial2.print(enc_right_read());
  //  Serial2.print(" ");
  //  Serial2.println(enc_right_extrapolate());
  //  delay(10);










}


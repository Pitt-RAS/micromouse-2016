#include "Arduino.h"
#include "stdio.h"

#include "conf.h"
#include "motors.h"
#include "sensors_encoders.h"
#include "MotionCalc.h"
#include "PIDController.h"
#include "RangeSensorContainer.h"
#include "RangeSensor.h"
#include "motion.h"
#include "IdealSweptTurns.h"

static float max_accel_straight = MAX_ACCEL_STRAIGHT;
static float max_decel_straight = MAX_DECEL_STRAIGHT;

// This can probably match the straight accels until they become very high.  testing required
static float max_accel_rotate = MAX_ACCEL_ROTATE;
static float max_decel_rotate = MAX_DECEL_ROTATE;

// should be calculated based on turn radius and max velocity so that the friction force required is not more than the force available
static float max_accel_corner = MAX_ACCEL_CORNER;
static float max_decel_corner = MAX_DECEL_CORNER;

static float max_vel_straight = MAX_VEL_STRAIGHT;
static float max_vel_rotate = MAX_VEL_ROTATE;
static float max_vel_corner = MAX_VEL_CORNER;

// instantiate the 90 degree turn table
//IdealSweptTurns turn_90_table(0.8, 90.0, 0.001);

void motion_forward(float distance, float exit_speed) {
  float errorRight, errorLeft, rotationOffset;
  float idealDistance, idealVelocity;
  elapsedMicros moveTime;

  float current_speed = (enc_left_velocity() + enc_right_velocity()) / 2;
  MotionCalc motionCalc (distance, max_vel_straight, current_speed, exit_speed, max_accel_straight,
                         max_decel_straight);

  PIDController left_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController right_PID (KP_POSITION, KI_POSITION, KD_POSITION);

  PIDController rotation_PID (KP_ROTATION, KI_ROTATION, KD_ROTATION);

  // zero clock before move
  moveTime = 0;

  // execute motion
  while (idealDistance != distance) {
    //Run sensor protocol here.  Sensor protocol should use encoder_left/right_write() to adjust for encoder error
    idealDistance = motionCalc.idealDistance(moveTime);
    idealVelocity = motionCalc.idealVelocity(moveTime);

    // Add error from rangefinder data.  Positive error is when it is too close to the left wall, requiring a positive angle to fix it.
    //RangeSensors.updateReadings();
    //rotationOffset = rotation_PID.Calculate(RangeSensors.errorFromCenter());
    rotationOffset = 0;

    errorLeft = enc_left_extrapolate() - idealDistance - rotationOffset;
    errorRight = enc_right_extrapolate() - idealDistance + rotationOffset;

    // Run PID to determine the offset that should be added/subtracted to the left/right wheels to fix the error.  Remember to remove or at the very least increase constraints on the I term
    // the offsets that are less than an encoder tick need to be added/subtracted from errorLeft and errorRight instead of encoderWrite being used.  Maybe add a third variable to the error calculation for these and other offsets

    motor_l.Set(motionCalc.idealAccel(moveTime) + left_PID.Calculate(errorLeft), idealVelocity);
    motor_r.Set(motionCalc.idealAccel(moveTime) + right_PID.Calculate(errorRight), idealVelocity);
  }

  enc_left_write(0);
  enc_right_write(0);
}

void motion_collect(float distance, float exit_speed){
    float errorRight, errorLeft;
  float idealDistance, idealVelocity;
  elapsedMicros moveTime;

  float current_speed = (enc_left_velocity() + enc_right_velocity()) / 2;
  MotionCalc motionCalc (distance, max_vel_straight, current_speed, exit_speed, max_accel_straight,
                         max_decel_straight);

  PIDController left_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController right_PID (KP_POSITION, KI_POSITION, KD_POSITION);

  PIDController rotation_PID (KP_ROTATION, KI_ROTATION, KD_ROTATION);

  //RangeSensor front_left_sensor(RANGE_FRONT_LEFT_PIN, 0, 0);
  //RangeSensor front_right_sensor(RANGE_FRONT_RIGHT_PIN, 0, 0);

  float off_reading_L, off_reading_R, off_reading_DL, off_reading_DR;
  float on_reading_L, on_reading_R, on_reading_DL, on_reading_DR;
  float value_L, value_R, value_DL, value_DR;
  int delayTime = 45;
  int num_range_value = distance/5;
  int num_readings = 1;
  int i;
  int last_reading_distance = -1;
  int int_distance;

  int reading_output_L[num_range_value];
  int reading_output_R[num_range_value];
  int reading_output_DL[num_range_value];
  int reading_output_DR[num_range_value];

  int reading_counter = 0;

  // zero clock before move
  moveTime = 0;

  // execute motion
  while (idealDistance != distance) {
    //Run sensor protocol here.  Sensor protocol should use encoder_left/right_write() to adjust for encoder error
    idealDistance = motionCalc.idealDistance(moveTime);
    idealVelocity = motionCalc.idealVelocity(moveTime);

    int_distance = (int) ((enc_left_extrapolate() + enc_right_extrapolate()) / 2);

    // takes a reading every 5mm
    if (int_distance % 5 == 0 && int_distance > last_reading_distance) {
      last_reading_distance = int_distance;

      on_reading_L = 0;
      off_reading_L = 0;
      on_reading_R = 0;
      off_reading_R = 0;
      on_reading_DL = 0;
      off_reading_DL = 0;
      on_reading_DR = 0;
      off_reading_DR = 0;


      for(i = 0; i<num_readings; i++)
      {
        // Front left

        digitalWrite(EMITTER3_PIN, HIGH);
        delayMicroseconds(delayTime);

        on_reading_L += analogRead(RANGE3_PIN);

        digitalWrite(EMITTER3_PIN, LOW);

        delayMicroseconds(delayTime);

        off_reading_L += analogRead(RANGE3_PIN);

        // Front right

        digitalWrite(EMITTER4_PIN, HIGH);
        delayMicroseconds(delayTime);

        on_reading_R += analogRead(RANGE4_PIN);

        digitalWrite(EMITTER4_PIN, LOW);

        delayMicroseconds(delayTime);

        off_reading_R += analogRead(RANGE4_PIN);

        delayMicroseconds(delayTime);

        // Diag left

        digitalWrite(EMITTER1_PIN, HIGH);
        delayMicroseconds(delayTime);

        on_reading_DL += analogRead(RANGE1_PIN);

        digitalWrite(EMITTER1_PIN, LOW);

        delayMicroseconds(delayTime);

        off_reading_DL += analogRead(RANGE1_PIN);

        delayMicroseconds(delayTime);

        // Diag right

        digitalWrite(EMITTER2_PIN, HIGH);
        delayMicroseconds(delayTime);

        on_reading_DR += analogRead(RANGE2_PIN);

        digitalWrite(EMITTER2_PIN, LOW);

        delayMicroseconds(delayTime);

        off_reading_DR += analogRead(RANGE2_PIN);

        delayMicroseconds(delayTime);
      }

      on_reading_L /= num_readings;
      off_reading_L /= num_readings;
      on_reading_R /= num_readings;
      off_reading_R /= num_readings;
      on_reading_DL /= num_readings;
      off_reading_DL /= num_readings;
      on_reading_DR /= num_readings;
      off_reading_DR /= num_readings;

      value_L = on_reading_L - off_reading_L;
      value_R = on_reading_R - off_reading_R;
      value_DL = on_reading_DL - off_reading_DL;
      value_DR = on_reading_DR - off_reading_DR;

      reading_output_L[reading_counter] = value_L;
      reading_output_R[reading_counter] = value_R;
      reading_output_DL[reading_counter] = value_DL;
      reading_output_DR[reading_counter] = value_DR;

      reading_counter++;
    }

    errorLeft = enc_left_extrapolate() - idealDistance;
    errorRight = enc_right_extrapolate() - idealDistance;

    // Run PID to determine the offset that should be added/subtracted to the left/right wheels to fix the error.  Remember to remove or at the very least increase constraints on the I term
    // the offsets that are less than an encoder tick need to be added/subtracted from errorLeft and errorRight instead of encoderWrite being used.  Maybe add a third variable to the error calculation for these and other offsets

    motor_l.Set(motionCalc.idealAccel(moveTime) + left_PID.Calculate(errorLeft), idealVelocity);
    motor_r.Set(motionCalc.idealAccel(moveTime) + right_PID.Calculate(errorRight), idealVelocity);
  }

  enc_left_write(0);
  enc_right_write(0);

  motion_hold(100);

  Serial.println("=== Front Left ===");
  for(i = 0; i<num_range_value; i++){
    Serial.print(reading_output_L[i]);
    Serial.print(" ");
    Serial.print((int) distance - 5 * i);
    Serial.println();
  }
  Serial.println("=== Front Right ===");
  for(i = 0; i<num_range_value; i++){
    Serial.print(reading_output_R[i]);
    Serial.print(" ");
    Serial.print((int) distance - 5 * i);
    Serial.println();
  }
  Serial.println("=== Diag Left ===");
  for(i = 0; i<num_range_value; i++){
    Serial.print(reading_output_DL[i]);
    Serial.print(" ");
    Serial.print((int) distance - 5 * i);
    Serial.println();
  }
  Serial.println("=== Diag Right ===");
  for(i = 0; i<num_range_value; i++){
    Serial.print(reading_output_DR[i]);
    Serial.print(" ");
    Serial.print((int) distance - 5 * i);
    Serial.println();
  }
}

// clockwise angle is positive, angle is in degrees
void motion_rotate(float angle) {
  float distancePerDegree = 3.14159265359 * MM_BETWEEN_WHEELS / 360;
  float idealLinearDistance, idealLinearVelocity;
  float errorLeft, errorRight;
  float linearDistance = distancePerDegree * angle;
  elapsedMicros moveTime;

  float current_speed = (enc_left_velocity() - enc_right_velocity()) / 2;
  MotionCalc motionCalc (linearDistance, max_vel_rotate, current_speed, 0, max_accel_rotate,
                         max_decel_rotate);

  PIDController left_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController right_PID (KP_POSITION, KI_POSITION, KD_POSITION);

  // zero encoders and clock before move
  moveTime = 0;

  // the right will always be the negative of the left in order to rotate on a point.
  while (idealLinearDistance != linearDistance) {
    //Run sensor protocol here.  Sensor protocol should use encoder_left/right_write() to adjust for encoder error
    idealLinearDistance = motionCalc.idealDistance(moveTime);
    idealLinearVelocity = motionCalc.idealVelocity(moveTime);

    errorLeft = enc_left_extrapolate() - idealLinearDistance;
    errorRight = enc_right_extrapolate() + idealLinearDistance;

    motor_l.Set(motionCalc.idealAccel(moveTime) + left_PID.Calculate(errorLeft),
                idealLinearVelocity);
    motor_r.Set(-motionCalc.idealAccel(moveTime) + right_PID.Calculate(errorRight),
                idealLinearVelocity);

    //run PID loop here.  new PID loop will add or subtract from a predetermined PWM value that was calculated with the motor curve and current ideal speed

  }

  enc_left_write(0);
  enc_right_write(0);
}

void motion_corner(float angle, float speed) {
  float sign = 1;
  if (angle < 0) {
    angle = -angle;
    sign = -1;
  }

// SOME SORT OF SWITCH IS NEEDED TO DETERMINE WHICH LOOKUP TABLE TO USE
//  switch (angle) {
//    case 45:
//      
//      break;
//    case 90:
//      
//      break;
//    case 135:
//      
//      break;
//    case 180;
//      
//      break;
//  }
  float errorRight, errorLeft;
  float idealDistance;
  float rotation_offset;
  float time_scaling = speed / SWEPT_TURN_90_FORWARD_SPEED * 1000;
  int move_time_scaled = 0;
  float distancePerDegree = 3.14159265359 * MM_BETWEEN_WHEELS / 360;
//  float total_time = turn_90_table.getTotalTime();

  elapsedMicros move_time;

  PIDController left_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController right_PID (KP_POSITION, KI_POSITION, KD_POSITION);

  // zero clock before move
  move_time = 0;
  

  // execute motion
//  while (move_time_scaled < total_time) {
//    //Run sensor protocol here.  Sensor protocol should use encoder_left/right_write() to adjust for encoder error
//    move_time_scaled = move_time * time_scaling;
//    
//    idealDistance = move_time_scaled * speed / 1000;
//
////    rotation_offset = turn_90_table.getOffsetAtMicros(move_time_scaled);
//    
//
//    errorLeft = enc_left_extrapolate() - idealDistance - rotation_offset;
//    errorRight = enc_right_extrapolate() - idealDistance + rotation_offset;
//
////    motor_l.Set(distancePerDegree
////        * time_scaling
////        * turn_90_table.getAngularAcceleration(move_time_scaled / 1000) / 1000
////        + left_PID.Calculate(errorLeft),
////        enc_left_velocity());
////    motor_r.Set(- distancePerDegree
////        * time_scaling
////        * turn_90_table.getAngularAcceleration(move_time_scaled / 1000) / 1000
////        + right_PID.Calculate(errorRight),
////        enc_right_velocity());
//   
//       motor_l.Set(left_PID.Calculate(errorLeft),
//        enc_left_velocity());
//       motor_r.Set(right_PID.Calculate(errorRight),
//        enc_right_velocity());
//   
//   }

  enc_left_write(0);
  enc_right_write(0);
}



























void motion_hold(unsigned int time) {
  float errorRight, errorLeft;
  float rightOutput, leftOutput;
  elapsedMicros currentTime;

  PIDController left_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController right_PID (KP_POSITION, KI_POSITION, KD_POSITION);

  currentTime = 0;

  while (currentTime / 1000 < time) {
    errorLeft = enc_left_extrapolate();
    errorRight = enc_right_extrapolate();

    leftOutput = left_PID.Calculate(errorLeft);
    rightOutput = right_PID.Calculate(errorRight);

    motor_l.Set(leftOutput, 0);
    motor_r.Set(rightOutput, 0);
  }

  motor_l.Set(0, 0);
  motor_r.Set(0, 0);
}

void motion_hold_range(int setpoint, unsigned int time) {
  float errorRight, errorLeft;
  float rightOutput, leftOutput;
  elapsedMicros currentTime;

  digitalWrite(13, HIGH);

  PIDController left_PID (KP_HOLD_RANGE, KI_HOLD_RANGE, KD_HOLD_RANGE);
  PIDController right_PID (KP_HOLD_RANGE, KI_HOLD_RANGE, KD_HOLD_RANGE);

  currentTime = 0;

  while (currentTime / 1000 < time) {

    RangeSensors.leftSensor.updateRange();
    RangeSensors.rightSensor.updateRange();
    
    errorLeft = RangeSensors.leftSensor.getRange();
    errorRight = RangeSensors.rightSensor.getRange();

    leftOutput = left_PID.Calculate(errorLeft);
    rightOutput = right_PID.Calculate(errorRight);

    motor_l.Set(leftOutput, 0);
    motor_r.Set(rightOutput, 0);
  }

  motor_l.Set(0, 0);
  motor_r.Set(0, 0);
}

// functions to set max velocity variables
void motion_set_maxAccel_straight(float temp_max_accel_straight) {
  max_accel_straight = temp_max_accel_straight;
}
void motion_set_maxDecel_straight(float temp_max_decel_straight) {
  max_decel_straight = temp_max_decel_straight;
}

void motion_set_maxAccel_rotate(float temp_max_accel_rotate) {
  max_accel_rotate = temp_max_accel_rotate;
}
void motion_set_maxDecel_rotate(float temp_max_decel_rotate) {
  max_decel_rotate = temp_max_decel_rotate;
}

void motion_set_maxAccel_corner(float temp_max_accel_corner) {
  max_accel_corner = temp_max_accel_corner;
}
void motion_set_maxDecel_corner(float temp_max_decel_corner) {
  max_decel_corner = temp_max_decel_corner;
}

void motion_set_maxVel_straight(float temp_max_vel_straight) {
  max_vel_straight = temp_max_vel_straight;
}
void motion_set_maxVel_rotate(float temp_max_vel_rotate) {
  max_vel_rotate = temp_max_vel_rotate;
}
void motion_set_maxVel_corner(float temp_max_vel_corner) {
  max_vel_corner = temp_max_vel_corner;
}

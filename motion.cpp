#include "Arduino.h"
#include "stdio.h"

#include "conf.h"
#include "motors.h"
#include "sensors_encoders.h"
#include "MotionCalc.h"
#include "PIDController.h"
#include "RangeSensorContainer.h"

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
    RangeSensors.updateReadings();
    rotationOffset = rotation_PID.Calculate(RangeSensors.errorFromCenter());

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

void motion_corner(float angle, float radius, float exit_speed) {
  float errorRight, errorLeft;
  float leftFraction, rightFraction;
  float idealDistance, idealVelocity;
  float distance;

  if (radius < 0) {
    radius *= -1;
  }
  else if (radius == 0) {
    motion_rotate(angle);
    return;
  }

  if (exit_speed < 0) {
    exit_speed *= -1;
  }

  distance = angle * 3.14159265359 * radius / 180;
  if (distance < 0) {
    distance *= -1;
  }

  elapsedMicros moveTime;

  if (angle <= 0) {
    leftFraction = (radius + MM_BETWEEN_WHEELS / 2) / radius;
    rightFraction = (radius - MM_BETWEEN_WHEELS / 2) / radius;
  }
  else {
    leftFraction = (radius - MM_BETWEEN_WHEELS / 2) / radius;
    rightFraction = (radius + MM_BETWEEN_WHEELS / 2) / radius;
  }

  float current_speed = (enc_left_velocity() + enc_right_velocity()) / 2;
  MotionCalc motionCalc (distance, max_vel_corner, current_speed, exit_speed, max_accel_corner,
                         max_decel_corner);

  PIDController left_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController right_PID (KP_POSITION, KI_POSITION, KD_POSITION);

  // zero clock before move
  moveTime = 0;

  // execute motion
  while (idealDistance != distance) {
    //Run sensor protocol here.  Sensor protocol should use encoder_left/right_write() to adjust for encoder error
    idealDistance = motionCalc.idealDistance(moveTime);
    idealVelocity = motionCalc.idealVelocity(moveTime);

    errorLeft = enc_left_extrapolate() - idealDistance * rightFraction;
    errorRight = enc_right_extrapolate() - idealDistance * leftFraction;

    motor_l.Set(motionCalc.idealAccel(moveTime) * leftFraction + left_PID.Calculate(errorLeft),
                idealVelocity * leftFraction);
    motor_r.Set(motionCalc.idealAccel(moveTime) * rightFraction + right_PID.Calculate(errorRight),
                idealVelocity * rightFraction);
  }

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

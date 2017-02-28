#include <Arduino.h>
#include "../device/Motor.h"
#include "../device/Orientation.h"
#include "../device/RangeSensor.h"
#include "../device/RangeSensorContainer.h"
#include "../device/sensors_encoders.h"
#include "../user_interaction/UserInterface.h"
#include "../user_interaction/FreakOut.h"
#include "../user_interaction/Logger.h"
#include "../user_interaction/Menu.h"
#include "../conf.h"
#include "MotionCalc.h"
#include "PIDController.h"
#include "SweptTurnProfile.h"
#include "motion.h"

static float max_accel_straight = MAX_ACCEL_STRAIGHT;
static float max_decel_straight = MAX_DECEL_STRAIGHT;

// This can probably match the straight accels until they become very high.  testing required
static float max_accel_rotate = MAX_ACCEL_ROTATE;
static float max_decel_rotate = MAX_DECEL_ROTATE;

// should be calculated based on turn radius and max velocity so that the friction force required is not more than the force available
static float max_accel_corner = MAX_ACCEL_CORNER;
static float max_decel_corner = MAX_DECEL_CORNER;

static float max_vel_straight = MAX_VEL_STRAIGHT;
static float max_vel_diag = MAX_VEL_DIAG;
static float max_vel_rotate = MAX_VEL_ROTATE;

static Orientation* orientation = NULL;

// instantiate the turn profiles
const SweptTurnProfile turn_45_table(SWEPT_TURN_45_FORWARD_SPEED,
                                     SWEPT_TURN_45_ANGLE);
const SweptTurnProfile turn_90_table(SWEPT_TURN_90_FORWARD_SPEED,
                                     SWEPT_TURN_90_ANGLE);
const SweptTurnProfile turn_135_table(SWEPT_TURN_135_FORWARD_SPEED,
                                      SWEPT_TURN_135_ANGLE);
const SweptTurnProfile turn_180_table(SWEPT_TURN_180_FORWARD_SPEED,
                                      SWEPT_TURN_180_ANGLE);

void motion_forward(float distance, float current_speed, float exit_speed) {
  // HACK
  //distance *= 1.01;
  float currentFrontRight, currentBackRight, currentFrontLeft, currentBackLeft;
  float setpointFrontRight, setpointBackRight, setpointFrontLeft, setpointBackLeft;
  float correctionFrontRight, correctionBackRight, correctionFrontLeft, correctionBackLeft;
  float rangeOffset;
  float gyroOffset = 0;
  float distancePerDegree = 3.14159265359 * MM_BETWEEN_WHEELS / 360;
  float idealDistance, idealVelocity;
  elapsedMicros moveTime;

  float currentExtrapolation = (enc_left_front_extrapolate() + enc_right_front_extrapolate()
                                 + enc_left_back_extrapolate() + enc_right_back_extrapolate())/4;

  float drift = currentExtrapolation;

  enc_left_front_write(0);
  enc_right_front_write(0);
  enc_left_back_write(0);
  enc_right_back_write(0);

  //instantiate with distance - the amount of drift between motion commands
  MotionCalc motionCalc (distance-drift, max_vel_straight, current_speed, exit_speed, max_accel_straight,
                         max_decel_straight);

  if (orientation == NULL) {
    orientation = Orientation::getInstance();
  }

  PIDController left_back_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController left_front_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController right_back_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController right_front_PID (KP_POSITION, KI_POSITION, KD_POSITION);

  PIDController range_PID (KP_RANGE, KI_RANGE, KD_RANGE);
  PIDController gyro_PID (KP_GYRO_FWD, KI_GYRO_FWD, KD_GYRO_FWD);

  // zero clock before move
  moveTime = 0;

  RangeSensors.updateReadings();

  //float lastRangeError = 0;
  bool passedMiddle = false;
  orientation->handler_update_ = false;

  // execute motion
  while (moveTime < motionCalc.getTotalTime()) {
    orientation->update();
    //Run sensor protocol here.  Sensor protocol should use encoder_left/right_write() to adjust for encoder error
    idealDistance = motionCalc.idealDistance(moveTime);
    idealVelocity = motionCalc.idealVelocity(moveTime);

    float leftReading = enc_left_front_extrapolate();
    float rightReading = enc_right_front_extrapolate();
    float position = (leftReading + rightReading) / 2 / MM_PER_BLOCK;

    // Add error from rangefinder data. Positive error is when it is too close
    // to the left wall, requiring a positive angle to fix it.
    RangeSensors.updateReadings();
    rangeOffset = range_PID.Calculate(RangeSensors.errorFromCenter(), 0);
    gyroOffset += gyro_PID.Calculate(orientation->getHeading()*distancePerDegree, rangeOffset);

    if (abs(orientation->getHeading()) > 60) {
      freakOut("BAD1");
    }

    currentFrontLeft = enc_left_front_extrapolate();
    currentBackLeft = enc_left_back_extrapolate();
    currentFrontRight = enc_right_front_extrapolate();
    currentBackRight = enc_right_back_extrapolate();

    setpointFrontLeft = idealDistance + gyroOffset;
    setpointBackLeft = idealDistance + gyroOffset;
    setpointFrontRight = idealDistance - gyroOffset;
    setpointBackRight = idealDistance - gyroOffset;

    correctionFrontLeft = left_front_PID.Calculate(currentFrontLeft,
                                                   setpointFrontLeft);
    correctionBackLeft = left_back_PID.Calculate(currentBackLeft,
                                                   setpointBackLeft);
    correctionFrontRight = right_front_PID.Calculate(currentFrontRight,
                                                   setpointFrontRight);
    correctionBackRight = right_back_PID.Calculate(currentBackRight,
                                                   setpointBackRight);

    // Save isWall state for use by high-level code.
    if (!passedMiddle && position > distance / MM_PER_BLOCK - 0.5) {
	    RangeSensors.saveIsWall();
	    passedMiddle = true;
    }

    // Only use range sensor correction in the middle half of a cell
//    float nearestWhole = (int) (position + 0.5);
//    float difference = position - nearestWhole;
//
//    if (-0.25 < difference && difference < 0.25)
//      rangeOffset = lastRangeError;
//    else
//      lastRangeError = rangeOffset;

    // Run PID to determine the offset that should be added/subtracted to the left/right wheels to fix the error.  Remember to remove or at the very least increase constraints on the I term
    // the offsets that are less than an encoder tick need to be added/subtracted from errorFrontLeft and errorFrontRight instead of encoderWrite being used.  Maybe add a third variable to the error calculation for these and other offsets

    motor_lf.Set(motionCalc.idealAccel(moveTime) + correctionFrontLeft,
                 idealVelocity);
    motor_rf.Set(motionCalc.idealAccel(moveTime) + correctionFrontRight,
                 idealVelocity);
    motor_rb.Set(motionCalc.idealAccel(moveTime) + correctionBackRight,
                 idealVelocity);
    motor_lb.Set(motionCalc.idealAccel(moveTime) + correctionBackLeft,
                 idealVelocity);

    logger.logMotionType('f');
    logger.nextCycle();
  }

  uint8_t old_SREG = SREG;
  noInterrupts();
  orientation->update();
  orientation->handler_update_ = true;
  SREG = old_SREG;

  enc_left_front_write(0);
  enc_right_front_write(0);
  enc_left_back_write(0);
  enc_right_back_write(0);

  float current_left_velocity = (enc_left_front_velocity()
                                  + enc_left_back_velocity()) / 2;
  float current_right_velocity = (enc_right_front_velocity()
                                   + enc_right_back_velocity()) / 2;
  motor_lf.Set(0, current_left_velocity);
  motor_rf.Set(0, current_right_velocity);
  motor_rb.Set(0, current_right_velocity);
  motor_lb.Set(0, current_left_velocity);
}

void motion_forward_diag(float distance, float current_speed, float exit_speed) {
  float currentFrontRight, currentBackRight, currentFrontLeft, currentBackLeft;
  float setpointFrontRight, setpointBackRight, setpointFrontLeft, setpointBackLeft;
  float correctionFrontRight, correctionBackRight, correctionFrontLeft, correctionBackLeft;
  float rangeOffset;
  float gyroOffset = 0;
  float distancePerDegree = 3.14159265359 * MM_BETWEEN_WHEELS / 360;
  float idealDistance, idealVelocity;
  elapsedMicros moveTime;

  float currentExtrapolation = (enc_left_front_extrapolate() + enc_right_front_extrapolate()
                                 + enc_left_back_extrapolate() + enc_right_back_extrapolate())/4;

  float drift = currentExtrapolation;

  //instantiate with distance - the amount of drift between motion commands
  MotionCalc motionCalc (distance-drift, max_vel_diag, current_speed, exit_speed, max_accel_straight,
                         max_decel_straight);

  if (orientation == NULL) {
    orientation = Orientation::getInstance();
  }

  PIDController left_back_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController left_front_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController right_back_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController right_front_PID (KP_POSITION, KI_POSITION, KD_POSITION);

  PIDController range_PID (KP_RANGE, KI_RANGE, KD_RANGE);
  PIDController gyro_PID (0.0015, 0.000, 0.00);

  // zero clock before move
  moveTime = 0;

  RangeSensors.updateReadings();

  orientation->handler_update_ = false;

  // execute motion
  while (moveTime < motionCalc.getTotalTime()) {
    orientation->update();
    //Run sensor protocol here.  Sensor protocol should use encoder_left/right_write() to adjust for encoder error
    idealDistance = motionCalc.idealDistance(moveTime);
    idealVelocity = motionCalc.idealVelocity(moveTime);

    // Add error from rangefinder data. Positive error is when it is too close
    // to the left wall, requiring a positive angle to fix it.
    RangeSensors.updateReadings();
    //rangeOffset = range_PID.Calculate(RangeSensors.errorFromCenter(), 0);
    if (RangeSensors.frontRightSensor.getRange() < 150) {
      if (RangeSensors.frontLeftSensor.getRange() < 150) {
        rangeOffset = 0;
      } else {
        rangeOffset = RangeSensors.frontRightSensor.getRange() - 150;
      }
    } else if (RangeSensors.frontLeftSensor.getRange() < 150) {
      rangeOffset = 150 - RangeSensors.frontLeftSensor.getRange();
    } else {
      rangeOffset = 0;
    }
    rangeOffset *= KP_DIAG_RANGE;
    gyroOffset += gyro_PID.Calculate(orientation->getHeading()*distancePerDegree, rangeOffset);

    if (abs(orientation->getHeading()) > 60) {
      freakOut("BAD2");
      //menu.showString("FACK", 4);
      //delay(3000);
      //char buf[5];
      //sprintf(buf, "%04f", orientation->getHeading());
      //freakOut(buf);
    }

    currentFrontLeft = enc_left_front_extrapolate();
    currentBackLeft = enc_left_back_extrapolate();
    currentFrontRight = enc_right_front_extrapolate();
    currentBackRight = enc_right_back_extrapolate();

    setpointFrontLeft = idealDistance + gyroOffset;
    setpointBackLeft = idealDistance + gyroOffset;
    setpointFrontRight = idealDistance - gyroOffset;
    setpointBackRight = idealDistance - gyroOffset;

    correctionFrontLeft = left_front_PID.Calculate(currentFrontLeft,
                                                   setpointFrontLeft);
    correctionBackLeft = left_back_PID.Calculate(currentBackLeft,
                                                   setpointBackLeft);
    correctionFrontRight = right_front_PID.Calculate(currentFrontRight,
                                                   setpointFrontRight);
    correctionBackRight = right_back_PID.Calculate(currentBackRight,
                                                   setpointBackRight);

    // Run PID to determine the offset that should be added/subtracted to the left/right wheels to fix the error.  Remember to remove or at the very least increase constraints on the I term
    // the offsets that are less than an encoder tick need to be added/subtracted from errorFrontLeft and errorFrontRight instead of encoderWrite being used.  Maybe add a third variable to the error calculation for these and other offsets

    motor_lf.Set(motionCalc.idealAccel(moveTime) + correctionFrontLeft,
                 idealVelocity);
    motor_rf.Set(motionCalc.idealAccel(moveTime) + correctionFrontRight,
                 idealVelocity);
    motor_rb.Set(motionCalc.idealAccel(moveTime) + correctionBackRight,
                 idealVelocity);
    motor_lb.Set(motionCalc.idealAccel(moveTime) + correctionBackLeft,
                 idealVelocity);

    logger.logMotionType('d');
    logger.nextCycle();
  }

  uint8_t old_SREG = SREG;
  noInterrupts();
  orientation->update();
  orientation->handler_update_ = true;
  SREG = old_SREG;

  enc_left_front_write(0);
  enc_right_front_write(0);
  enc_left_back_write(0);
  enc_right_back_write(0);

  float current_left_velocity = (enc_left_front_velocity()
                                  + enc_left_back_velocity()) / 2;
  float current_right_velocity = (enc_right_front_velocity()
                                   + enc_right_back_velocity()) / 2;
  motor_lf.Set(0, current_left_velocity);
  motor_rf.Set(0, current_right_velocity);
  motor_rb.Set(0, current_right_velocity);
  motor_lb.Set(0, current_left_velocity);
}

void motion_collect(float distance, float current_speed, float exit_speed){
  float currentFrontRight, currentFrontLeft, currentBackRight, currentBackLeft;
  float setpointFrontRight, setpointFrontLeft, setpointBackRight, setpointBackLeft;
  float correctionFrontRight, correctionFrontLeft, correctionBackRight, correctionBackLeft;
  float rotationOffset;
  float idealDistance, idealVelocity;
  float distancePerDegree = 3.14159265359 * MM_BETWEEN_WHEELS / 360;
  elapsedMicros moveTime;

  MotionCalc motionCalc (distance, max_vel_straight, current_speed, exit_speed, max_accel_straight,
                         max_decel_straight);

  PIDController left_back_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController left_front_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController right_back_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController right_front_PID (KP_POSITION, KI_POSITION, KD_POSITION);

  PIDController rotation_PID (KP_ROTATION, KI_ROTATION, KD_ROTATION);

  if (orientation == NULL) {
    orientation = Orientation::getInstance();
  }
  rotationOffset = 0;

  int num_range_value = distance/MOTION_COLLECT_MM_PER_READING;
  int num_readings = 1;
  int i;
  int last_reading_distance = -1;
  int int_distance;

  int reading_output_L[num_range_value];
  int reading_output_R[num_range_value];
  int reading_output_DL[num_range_value];
  int reading_output_DR[num_range_value];
  memset(reading_output_L, 0, num_range_value * sizeof(int));
  memset(reading_output_R, 0, num_range_value * sizeof(int));
  memset(reading_output_DL, 0, num_range_value * sizeof(int));
  memset(reading_output_DR, 0, num_range_value * sizeof(int));

  int reading_counter = 0;

  // zero clock before move
  moveTime = 0;

  // execute motion
  while (idealDistance != distance) {
    //Run sensor protocol here.  Sensor protocol should use encoder_left/right_write() to adjust for encoder error
    orientation->update();
    idealDistance = motionCalc.idealDistance(moveTime);
    idealVelocity = motionCalc.idealVelocity(moveTime);

    int_distance = (int) ((enc_left_front_extrapolate() + enc_right_front_extrapolate() + enc_left_back_extrapolate() + enc_right_back_extrapolate())/4);

    // takes a reading every 5mm
    if (int_distance % MOTION_COLLECT_MM_PER_READING == 0 && int_distance > last_reading_distance) {
      last_reading_distance = int_distance;

      for (i = 0; i < num_readings; i++)
      {
        // Front left
        RangeSensors.frontLeftSensor.updateRange();
        reading_output_L[reading_counter] += RangeSensors.frontLeftSensor.getRawReading();

        // Front right
        RangeSensors.frontRightSensor.updateRange();
        reading_output_R[reading_counter] += RangeSensors.frontRightSensor.getRawReading();

        // Diag left
        RangeSensors.diagLeftSensor.updateRange();
        reading_output_DL[reading_counter] += RangeSensors.diagLeftSensor.getRawReading();

        // Diag right
        RangeSensors.diagRightSensor.updateRange();
        reading_output_DR[reading_counter] += RangeSensors.diagRightSensor.getRawReading();
      }

      reading_output_L[reading_counter] /= num_readings;
      reading_output_R[reading_counter] /= num_readings;
      reading_output_DL[reading_counter] /= num_readings;
      reading_output_DR[reading_counter] /= num_readings;

      reading_counter++;

      logger.logMotionType('c');
      logger.nextCycle();
    }

    rotationOffset += rotation_PID.Calculate(orientation->getHeading() * distancePerDegree, 0);

    currentFrontLeft = enc_left_front_extrapolate();
    currentBackLeft = enc_left_back_extrapolate();
    currentFrontRight = enc_right_front_extrapolate();
    currentBackRight = enc_right_back_extrapolate();

    setpointFrontLeft = idealDistance + rotationOffset;
    setpointBackLeft = idealDistance + rotationOffset;
    setpointFrontRight = idealDistance - rotationOffset;
    setpointBackRight = idealDistance - rotationOffset;

    correctionFrontLeft = left_front_PID.Calculate(currentFrontLeft,
                                                   setpointFrontLeft);
    correctionBackLeft = left_back_PID.Calculate(currentBackLeft,
                                                   setpointBackLeft);
    correctionFrontRight = right_front_PID.Calculate(currentFrontRight,
                                                   setpointFrontRight);
    correctionBackRight = right_back_PID.Calculate(currentBackRight,
                                                   setpointBackRight);

    // Run PID to determine the offset that should be added/subtracted to the left/right wheels to fix the error.  Remember to remove or at the very least increase constraints on the I term
    // the offsets that are less than an encoder tick need to be added/subtracted from errorFrontLeft and errorFrontRight instead of encoderWrite being used.  Maybe add a third variable to the error calculation for these and other offsets

    motor_lf.Set(motionCalc.idealAccel(moveTime) + correctionFrontLeft,
                 idealVelocity);
    motor_rf.Set(motionCalc.idealAccel(moveTime) + correctionFrontRight,
                 idealVelocity);
    motor_rb.Set(motionCalc.idealAccel(moveTime) + correctionBackRight,
                 idealVelocity);
    motor_lb.Set(motionCalc.idealAccel(moveTime) + correctionBackLeft,
                 idealVelocity);
  }

  enc_left_front_write(0);
  enc_right_front_write(0);
  enc_left_back_write(0);
  enc_right_back_write(0);

  motion_hold(100);
  while (!gUserInterface.buttonOkPressed()) {}

  Serial.println("=== Front Left ===");
  for(i = 0; i<num_range_value; i++){
    Serial.print(reading_output_L[i]);
    Serial.print(" ");
    Serial.print((int) distance - MOTION_COLLECT_MM_PER_READING * i);
    Serial.println();
  }
  Serial.println("=== Front Right ===");
  for(i = 0; i<num_range_value; i++){
    Serial.print(reading_output_R[i]);
    Serial.print(" ");
    Serial.print((int) distance - MOTION_COLLECT_MM_PER_READING * i);
    Serial.println();
  }
  Serial.println("=== Diag Left ===");
  for(i = 0; i<num_range_value; i++){
    Serial.print(reading_output_DL[i]);
    Serial.print(" ");
    Serial.print((int) distance - MOTION_COLLECT_MM_PER_READING * i);
    Serial.println();
  }
  Serial.println("=== Diag Right ===");
  for(i = 0; i<num_range_value; i++){
    Serial.print(reading_output_DR[i]);
    Serial.print(" ");
    Serial.print((int) distance - MOTION_COLLECT_MM_PER_READING * i);
    Serial.println();
  }
}

// clockwise angle is positive, angle is in degrees
void motion_rotate(float angle) {
  float distancePerDegree = 3.14159265359 * MM_BETWEEN_WHEELS_ROTATE / 360;
  float idealLinearDistance, idealLinearVelocity;
  float currentFrontRight, currentBackRight, currentFrontLeft, currentBackLeft;
  float setpointFrontRight, setpointBackRight, setpointFrontLeft, setpointBackLeft;
  float correctionFrontRight, correctionBackRight, correctionFrontLeft, correctionBackLeft;
  float gyro_correction;
  float linearDistance = distancePerDegree * angle;
  elapsedMicros moveTime;

  float current_speed = ((enc_left_front_velocity() + enc_left_back_velocity()) - (enc_right_front_velocity() + enc_right_back_velocity()))/4;
  
  float drift = (enc_left_front_extrapolate() + enc_left_front_extrapolate()
                 - enc_right_front_extrapolate() - enc_right_back_extrapolate())/2;
  drift = 0;

  //instantiate with distance - the amount of drift between motion commands 
  MotionCalc motionCalc (linearDistance - drift, max_vel_rotate, current_speed, 0, max_accel_rotate,
                         max_decel_rotate);

  if (orientation == NULL) {
    orientation = Orientation::getInstance();
  }

  PIDController left_front_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController right_front_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController left_back_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController right_back_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController gyro_PID (KP_GYRO, KI_GYRO, KD_GYRO);

  // zero encoders and clock before move
  moveTime = 0;

  // the right will always be the negative of the left in order to rotate on a point.
  orientation->handler_update_ = false;
  while (idealLinearDistance != linearDistance - drift) {
    orientation->update();
    if (orientation->getHeading() > 180)
      break;

    //Run sensor protocol here.  Sensor protocol should use encoder_left/right_write() to adjust for encoder error
    idealLinearDistance = motionCalc.idealDistance(moveTime);
    idealLinearVelocity = motionCalc.idealVelocity(moveTime);

    gyro_correction = gyro_PID.Calculate(
        orientation->getHeading() * distancePerDegree,
        idealLinearDistance);

    if (abs(orientation->getHeading() - idealLinearDistance / distancePerDegree) > 60) {
      freakOut("BAD3");
    }

    currentFrontLeft = enc_left_front_extrapolate();
    currentBackLeft = enc_left_back_extrapolate();
    currentFrontRight = enc_right_front_extrapolate();
    currentBackRight = enc_right_back_extrapolate();

    setpointFrontLeft = idealLinearDistance + gyro_correction;
    setpointBackLeft = idealLinearDistance + gyro_correction;
    setpointFrontRight = -idealLinearDistance - gyro_correction;
    setpointBackRight = -idealLinearDistance - gyro_correction;

    correctionFrontLeft = left_front_PID.Calculate(currentFrontLeft,
                                                   setpointFrontLeft);
    correctionBackLeft = left_back_PID.Calculate(currentBackLeft,
                                                 setpointBackLeft);
    correctionFrontRight = right_front_PID.Calculate(currentFrontRight,
                                                     setpointFrontRight);
    correctionBackRight = right_back_PID.Calculate(currentBackRight,
                                                   setpointBackRight);

    motor_lf.Set(motionCalc.idealAccel(moveTime) + correctionFrontLeft,
                 idealLinearVelocity);
    motor_rf.Set(-motionCalc.idealAccel(moveTime) + correctionFrontRight,
                 -idealLinearVelocity);
    motor_rb.Set(-motionCalc.idealAccel(moveTime) + correctionBackRight,
                 -idealLinearVelocity);
    motor_lb.Set(motionCalc.idealAccel(moveTime) + correctionBackLeft,
                 idealLinearVelocity);
    //run PID loop here.  new PID loop will add or subtract from a predetermined PWM value that was calculated with the motor curve and current ideal speed

    logger.logMotionType('p');
    logger.nextCycle();
  }
  //menu.showInt(orientation->getHeading(),4);
  uint8_t old_SREG = SREG;
  noInterrupts();
  orientation->update();
  orientation->incrementHeading(-angle);
  orientation->handler_update_ = true;
  SREG = old_SREG;

  enc_left_front_write(0);
  enc_right_front_write(0);
  enc_left_back_write(0);
  enc_right_back_write(0);

  float current_left_velocity = (enc_left_front_velocity()
                                  + enc_left_back_velocity()) / 2;
  float current_right_velocity = (enc_right_front_velocity()
                                   + enc_right_back_velocity()) / 2;
  motor_lf.Set(0, current_left_velocity);
  motor_rf.Set(0, current_right_velocity);
  motor_rb.Set(0, current_right_velocity);
  motor_lb.Set(0, current_left_velocity);
}

void motion_gyro_rotate(float angle) {
  float distancePerDegree = 3.14159265359 * MM_BETWEEN_WHEELS / 360;
  float idealLinearDistance, idealLinearVelocity;
  float rotation_correction;
  float linearDistance = distancePerDegree * angle;
  elapsedMicros moveTime;

  float current_speed = (enc_left_front_velocity() + enc_left_back_velocity()
                         - enc_right_front_velocity() - enc_right_back_velocity()) / 4;
  MotionCalc motionCalc (linearDistance, max_vel_rotate, current_speed, 0, max_accel_rotate,
                         max_decel_rotate);

  if (orientation == NULL) {
    orientation = Orientation::getInstance();
  }

  PIDController rotation_PID (KP_ROTATION, KI_ROTATION, KD_ROTATION);

  // zero encoders and clock before move
  moveTime = 0;

  // the right will always be the negative of the left in order to rotate on a point.
  idealLinearDistance = 0;
  while (idealLinearDistance != linearDistance) {
    orientation->update();

    //Run sensor protocol here.  Sensor protocol should use encoder_left/right_write() to adjust for encoder error
    idealLinearDistance = motionCalc.idealDistance(moveTime);
    idealLinearVelocity = motionCalc.idealVelocity(moveTime);

    rotation_correction = rotation_PID.Calculate(
        orientation->getHeading() * distancePerDegree,
        idealLinearDistance);
    Serial.println(rotation_correction);

    if (abs(orientation->getHeading() - idealLinearDistance / distancePerDegree) > 60) {
      freakOut("BAD4");
    }

    // run PID loop here.  new PID loop will add or subtract from a predetermined
    //   PWM value that was calculated with the motor curve and current ideal speed
    motor_lf.Set(motionCalc.idealAccel(moveTime) + rotation_correction,
                 idealLinearVelocity);
    motor_rf.Set(-motionCalc.idealAccel(moveTime) - rotation_correction,
                 -idealLinearVelocity);
    motor_rb.Set(-motionCalc.idealAccel(moveTime) - rotation_correction,
                 -idealLinearVelocity);
    motor_lb.Set(motionCalc.idealAccel(moveTime) + rotation_correction,
                 idealLinearVelocity);

    logger.logMotionType('g');
    logger.nextCycle();
  }

  enc_left_front_write(0);
  enc_right_front_write(0);
  enc_left_back_write(0);
  enc_right_back_write(0);
  orientation->incrementHeading(-angle);

  float current_left_velocity = (enc_left_front_velocity()
                                  + enc_left_back_velocity()) / 2;
  float current_right_velocity = (enc_right_front_velocity()
                                   + enc_right_back_velocity()) / 2;
  motor_lf.Set(0, current_left_velocity);
  motor_rf.Set(0, current_right_velocity);
  motor_rb.Set(0, current_right_velocity);
  motor_lb.Set(0, current_left_velocity);
}

void motion_corner(SweptTurnType turn_type, float speed, float size_scaling) {
  int sign = 1;
  float currentFrontRight, currentFrontLeft, currentBackRight, currentBackLeft;
  float setpointFrontRight, setpointFrontLeft, setpointBackRight, setpointBackLeft;
  float idealDistance;
  float rotation_offset;
  float gyro_correction;
  float time_scaling = 1;
  int move_time_scaled = 0;
  float distancePerDegree = 3.14159265359 * MM_BETWEEN_WHEELS / 360;
  float total_time;
  const SweptTurnProfile* turn_table = NULL;

  if (orientation == NULL) {
    orientation = Orientation::getInstance();
  }

  //float drift = (enc_left_front_extrapolate() + enc_right_front_extrapolate()
  //               + enc_left_back_extrapolate() + enc_right_back_extrapolate())/4;

  switch (turn_type) {
    case kLeftTurn45:
      turn_table = &turn_45_table;
      time_scaling = speed / SWEPT_TURN_45_FORWARD_SPEED;
      sign = -1;
      break;
    case kLeftTurn90:
      turn_table = &turn_90_table;
      time_scaling = speed / SWEPT_TURN_90_FORWARD_SPEED;
      sign = -1;
      break;
    case kLeftTurn135:
      turn_table = &turn_135_table;
      time_scaling = speed / SWEPT_TURN_135_FORWARD_SPEED;
      sign = -1;
      break;
    case kLeftTurn180:
      turn_table = &turn_180_table;
      time_scaling = speed / SWEPT_TURN_180_FORWARD_SPEED;
      sign = -1;
      break;
    case kRightTurn45:
      turn_table = &turn_45_table;
      time_scaling = speed / SWEPT_TURN_45_FORWARD_SPEED;
      sign = 1;
      break;
    case kRightTurn90:
      turn_table = &turn_90_table;
      time_scaling = speed / SWEPT_TURN_90_FORWARD_SPEED;
      sign = 1;
      break;
    case kRightTurn135:
      turn_table = &turn_135_table;
      time_scaling = speed / SWEPT_TURN_135_FORWARD_SPEED;
      sign = 1;
      break;
    case kRightTurn180:
      turn_table = &turn_180_table;
      time_scaling = speed / SWEPT_TURN_180_FORWARD_SPEED;
      sign = 1;
      break;
    default:
      freakOut("OOPS");
      break;
  }

  total_time = turn_table->getTotalTime();

  elapsedMicros move_time;

  PIDController left_front_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController right_front_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController left_back_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController right_back_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController gyro_PID (KP_GYRO, KI_GYRO, KD_GYRO);

  // zero clock before move
  move_time = 0;

  orientation->handler_update_ = false;

  // execute motion
  while (move_time_scaled < total_time * 1000000) {
    //Run sensor protocol here.  Sensor protocol should use encoder_left/right_write() to adjust for encoder error
    orientation->update();

    move_time_scaled = move_time * time_scaling / size_scaling;
    
    idealDistance = move_time * speed / 1000;

    rotation_offset = MM_BETWEEN_WHEELS / 2
        * sign * turn_table->getAngle(move_time_scaled / 1000000.0);

    gyro_correction = gyro_PID.Calculate(
        orientation->getHeading() * distancePerDegree,
        rotation_offset);

    if (abs(orientation->getHeading() - rotation_offset / distancePerDegree) > 60) {
      freakOut("BAD5");
    }

    currentFrontLeft = enc_left_front_extrapolate();
    currentBackLeft = enc_left_back_extrapolate();
    currentFrontRight = enc_right_front_extrapolate();
    currentBackRight = enc_right_back_extrapolate();

    setpointFrontLeft = idealDistance + rotation_offset + gyro_correction;
    setpointBackLeft = idealDistance + rotation_offset + gyro_correction;
    setpointFrontRight = idealDistance - rotation_offset - gyro_correction;
    setpointBackRight = idealDistance - rotation_offset - gyro_correction;

//    motor_l.Set(distancePerDegree
//        * time_scaling
//        * turn_90_table.getAngularAcceleration(move_time_scaled / 1000) / 1000
//        + left_PID.Calculate(errorFrontLeft),
//        enc_left_velocity());
//    motor_r.Set(- distancePerDegree
//        * time_scaling
//        * turn_90_table.getAngularAcceleration(move_time_scaled / 1000) / 1000
//        + right_PID.Calculate(errorFrontRight),
//        enc_right_velocity());
   
    motor_lf.Set(left_front_PID.Calculate(currentFrontLeft, setpointFrontLeft),
                 enc_left_front_velocity());
    motor_rf.Set(right_front_PID.Calculate(currentFrontRight, setpointFrontRight),
                 enc_right_front_velocity());
    motor_rb.Set(right_back_PID.Calculate(currentBackRight, setpointBackRight),
                 enc_right_back_velocity());
    motor_lb.Set(left_back_PID.Calculate(currentBackLeft, setpointBackLeft),
                 enc_left_back_velocity());

    logger.logMotionType('s');
    logger.nextCycle();
  }

  uint8_t old_SREG = SREG;
  noInterrupts();
  orientation->update();
  orientation->incrementHeading(-sign * turn_table->getTotalAngle());
  orientation->handler_update_ = true;
  SREG = old_SREG;

  enc_left_front_write(0);
  enc_right_front_write(0);
  enc_left_back_write(0);
  enc_right_back_write(0);

  float current_left_velocity = (enc_left_front_velocity()
                                  + enc_left_back_velocity()) / 2;
  float current_right_velocity = (enc_right_front_velocity()
                                   + enc_right_back_velocity()) / 2;
  motor_lf.Set(0, current_left_velocity);
  motor_rf.Set(0, current_right_velocity);
  motor_rb.Set(0, current_right_velocity);
  motor_lb.Set(0, current_left_velocity);
}

void motion_hold(unsigned int time) {
  float errorFrontRight, errorFrontLeft, errorBackRight, errorBackLeft;
  float rightFrontOutput, leftFrontOutput, rightBackOutput, leftBackOutput;
  elapsedMicros currentTime;

  PIDController left_front_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController right_front_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController left_back_PID (KP_POSITION, KI_POSITION, KD_POSITION);
  PIDController right_back_PID (KP_POSITION, KI_POSITION, KD_POSITION);

  currentTime = 0;

  if (orientation == NULL) {
    orientation = Orientation::getInstance();
  }

  orientation->handler_update_ = false;

  while (currentTime / 1000 < time) {
    orientation->update();
    errorFrontLeft = enc_left_front_extrapolate();
    errorFrontRight = enc_right_front_extrapolate();
    errorBackLeft = enc_left_back_extrapolate();
    errorBackRight = enc_right_back_extrapolate();

    leftFrontOutput = left_front_PID.Calculate(errorFrontLeft, 0);
    rightFrontOutput = right_front_PID.Calculate(errorFrontRight, 0);
    leftBackOutput = left_back_PID.Calculate(errorBackLeft, 0);
    rightBackOutput = right_back_PID.Calculate(errorBackRight, 0);

    motor_lf.Set(leftFrontOutput, 0);
    motor_rf.Set(rightFrontOutput, 0);
    motor_rb.Set(rightBackOutput,0);
    motor_lb.Set(leftBackOutput,0);

    logger.logMotionType('h');
    logger.nextCycle();
  }

  uint8_t old_SREG = SREG;
  noInterrupts();
  orientation->update();
  orientation->handler_update_ = true;
  SREG = old_SREG;

  motor_lf.Set(0, 0);
  motor_rf.Set(0, 0);
  motor_rb.Set(0, 0);
  motor_lb.Set(0, 0);
}

void motion_hold_range(int setpoint, unsigned int time) {
  float rightFrontOutput, leftFrontOutput, rightBackOutput, leftBackOutput;
  elapsedMicros currentTime;

  PIDController left_front_PID (KP_HOLD_RANGE, KI_HOLD_RANGE, KD_HOLD_RANGE);
  PIDController right_front_PID (KP_HOLD_RANGE, KI_HOLD_RANGE, KD_HOLD_RANGE);
  PIDController left_back_PID (KP_HOLD_RANGE, KI_HOLD_RANGE, KD_HOLD_RANGE);
  PIDController right_back_PID (KP_HOLD_RANGE, KI_HOLD_RANGE, KD_HOLD_RANGE);

  currentTime = 0;

  if (orientation == NULL) {
    orientation = Orientation::getInstance();
  }

  orientation->handler_update_ = false;

  while (currentTime / 1000 < time) {
    orientation->update();

    RangeSensors.frontLeftSensor.updateRange();
    RangeSensors.frontRightSensor.updateRange();

    leftFrontOutput = left_front_PID.Calculate(
        -RangeSensors.frontLeftSensor.getRange(),
        -setpoint);
    leftBackOutput = left_back_PID.Calculate(
        -RangeSensors.frontLeftSensor.getRange(),
        -setpoint);
    rightFrontOutput = right_front_PID.Calculate(
        -RangeSensors.frontRightSensor.getRange(),
        -setpoint);
    rightBackOutput = right_back_PID.Calculate(
        -RangeSensors.frontRightSensor.getRange(),
        -setpoint);

    if (leftFrontOutput > 10) leftFrontOutput = 10;
    if (leftFrontOutput < -10) leftFrontOutput = -10;
    if (rightFrontOutput > 10) rightFrontOutput = 10;
    if (rightFrontOutput < -10) rightFrontOutput = -10;
    if (leftBackOutput > 10) leftBackOutput = 10;
    if (leftBackOutput < -10) leftBackOutput = -10;
    if (rightBackOutput > 10) rightBackOutput = 10;
    if (rightBackOutput < -10) rightBackOutput = -10;

    motor_lf.Set(leftFrontOutput, 0);
    motor_rf.Set(rightFrontOutput, 0);
    motor_rb.Set(rightBackOutput, 0);
    motor_lb.Set(leftBackOutput, 0);

    logger.logMotionType('r');
    logger.nextCycle();
  }

  enc_left_front_write(0);
  enc_right_front_write(0);
  enc_left_back_write(0);
  enc_right_back_write(0);

  uint8_t old_SREG = SREG;
  noInterrupts();
  orientation->update();
  orientation->handler_update_ = true;
  SREG = old_SREG;

  motor_lf.Set(0, 0);
  motor_rf.Set(0, 0);
  motor_rb.Set(0, 0);
  motor_lb.Set(0, 0);
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
void motion_set_maxVel_diag(float temp_max_vel_diag) {
  max_vel_diag = temp_max_vel_diag;
}
void motion_set_maxVel_rotate(float temp_max_vel_rotate) {
  max_vel_rotate = temp_max_vel_rotate;
}

float motion_get_maxAccel_straight() {
  return max_accel_straight;
}
float motion_get_maxDecel_straight() {
  return max_decel_straight;
}

float motion_get_maxAccel_rotate() {
  return max_accel_rotate;
}
float motion_get_maxDecel_rotate() {
  return max_decel_rotate;
}

float motion_get_maxAccel_corner() {
  return max_accel_corner;
}
float motion_get_maxDecel_corner() {
  return max_decel_corner;
}

float motion_get_maxVel_straight() {
  return max_vel_straight;
}
float motion_get_maxVel_diag() {
  return max_vel_diag;
}
float motion_get_maxVel_rotate() {
  return max_vel_rotate;
}

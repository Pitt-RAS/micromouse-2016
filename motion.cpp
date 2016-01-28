#include "conf.h"
#include "motors.h"
#include "Arduino.h"
#include "sensors_encoders.h"
#include "string"
#include "stdio.h"
#include "RangeSensorContainer.h"

float max_accel_straight = MAX_ACCEL_STRAIGHT;
float max_decel_straight = MAX_DECEL_STRAIGHT;

// This can probably match the straight accels until they become very high.  testing required
float max_accel_rotate = MAX_ACCEL_ROTATE;
float max_decel_rotate = MAX_DECEL_ROTATE;

// should be calculated based on turn radius and max velocity so that the friction force required is not more than the force available
float max_accel_corner = MAX_ACCEL_CORNER;
float max_decel_corner = MAX_DECEL_CORNER;

float max_vel_straight = MAX_VEL_STRAIGHT;
float max_vel_rotate = MAX_VEL_ROTATE;
float max_vel_corner = MAX_VEL_CORNER;



class motionCalc {
  private:
    float max_accel, max_decel;
    float vStart, vEnd, vMax;
    float dStart, dEnd, dTot;
    float aStart, aEnd;
    int tStart, tConst, tEnd;

  public:
    motionCalc (float, float, float, float, float);
    float idealDistance (int elapsedTime) {
      if (elapsedTime < tStart) {
        return (elapsedTime / 1000 * (vStart + .5 * aStart * elapsedTime / 1000000));
      }
      else if (elapsedTime < (tStart + tConst)) {
        return (dStart * 1000 + vMax * (elapsedTime - tStart) / 1000);
      }
      else if (elapsedTime < (tStart + tConst + tEnd)) {
        float t = ((float)elapsedTime - tStart - tConst) / 1000000;
        return (((dTot - dEnd) + (vMax * t + .5 * aEnd * t * t)) * 1000);
      }
      else {
        return (dTot * 1000);
      }
    }
    float idealVelocity (int elapsedTime) {
      if (elapsedTime <= tStart) {
        return (vStart + aStart * elapsedTime / 1000000);
      }
      else if (elapsedTime < (tStart + tConst)) {
        return (vMax);
      }
      else if (elapsedTime < (tStart + tConst + tEnd)) {
        return (vMax + aEnd * (elapsedTime - tStart - tConst) / 1000000);
      }
      else {
        return (vEnd);
      }
    }
    float idealAccel (int elapsedTime) {
      if (elapsedTime <= tStart) {
        return (aStart);
      }
      else if (elapsedTime < (tStart + tConst)) {
        return (0);
      }
      else if (elapsedTime < (tStart + tConst + tEnd)) {
        return (aEnd);
      }
      else {
        return (0);
      }
    }
};

motionCalc::motionCalc (float temp_dTot, float temp_vMax, float temp_vEnd, float temp_max_accel, float temp_max_decel) {

  dTot = temp_dTot;
  vMax = temp_vMax;
  vEnd = temp_vEnd;
  max_accel = temp_max_accel;
  max_decel = temp_max_decel;

  // make sure that accel and decel have the correct sign
  if (max_accel < 0) {
    max_accel *= -1;
  }
  if (max_decel > 0) {
    max_decel *= -1;
  }

  // turn dTot into meters
  dTot /= 1000;

  // get current speed by averaging current instantaneous speed of left and right wheels
  vStart = (enc_left_velocity() + enc_right_velocity()) / 2;

  // set constants from global.  Do this a different way later to reference conf.h
  if (((dTot < 0) && (vMax > 0)) || ((dTot > 0) && (vMax < 0))) {
    vMax = -vMax;
  }
  if (vStart <= vMax) {
    aStart = max_accel;
  }
  else {
    aStart = max_decel;
  }

  if (vEnd <= vMax) {
    aEnd = max_decel;
  }
  else {
    aEnd = max_accel;
  }

  // do initial calculations
  // set distances assuming there is room to reach max speed
  dStart = (vMax * vMax - vStart * vStart) / (2 * aStart);
  dEnd = (vEnd * vEnd - vMax * vMax) / (2 * aEnd);
  tConst = 1000000 * (dTot - dStart - dEnd) / vMax;

  // set distances if there is not space to reach max speed
  if (tConst < 0) {
    dStart = (vStart * vStart - vEnd * vEnd + 2 * aEnd * dTot) / (2 * aEnd - 2 * aStart);
    dEnd = dTot - dStart;
    vMax = sqrt(vStart * vStart + 2 * aStart * dStart);
    tConst = 0;
  }

  // calculate tStart and tEnd based on dStart and dEnd
  tStart = (1000000 * (-vStart + sqrt(vStart * vStart + 2 * aStart * dStart)) / aStart);
  tEnd =  (1000000 * (-vEnd + sqrt(vEnd * vEnd + 2 * -aEnd * dEnd)) / -aEnd);
}





// input current speed and desired force
class idealMotorOutputCalculator {
  private:
    float motorOutput;
    float requiredCurrent, currentBEMF;
    float velocityPerVBEMF, forcePerAmp;

  public:
    idealMotorOutputCalculator();
    float Calculate (float desiredForce, float currentVelocity) {
      requiredCurrent = desiredForce / forcePerAmp;
      currentBEMF = currentVelocity / velocityPerVBEMF;
      return ((requiredCurrent * RATED_INTERNAL_RESISTANCE + currentBEMF) / BATTERY_VOLTAGE);
    }
} left_motor_output_calculator, right_motor_output_calculator;

idealMotorOutputCalculator::idealMotorOutputCalculator() {
  // calcluate these terms initially in case float math isn't done precompile
  velocityPerVBEMF = VELOCITY_PER_VBEMF;
  forcePerAmp = FORCE_PER_AMP;
}


class PID_Controller {
  private:
    float i_term = 0;
    elapsedMicros elapsed_time;
    float last_error = 0;
	  float kp, ki, kd;
  public:
    float i_upper_bound = 2;
    float i_lower_bound = -2;
    PID_Controller(float, float, float);
    float Calculate(float error);
} ;

PID_Controller::PID_Controller(float tempKP, float tempKI, float tempKD) {
	kp = tempKP;
	ki = tempKI;
	kd = tempKD;
}


float PID_Controller::Calculate(float error) {
  i_term += ki * error * elapsed_time;
  i_term = constrain(i_term, i_lower_bound, i_upper_bound);

  float output = -kp * error - i_term + kd * (error - last_error) / elapsed_time;
  last_error = error;

  elapsed_time = 0;

  return output;
}


void motion_forward(float distance, float exit_speed) {
  float errorRight, errorLeft, errorCenter, rotationOffset;
  float rightOutput, leftOutput;
  float idealDistance, idealVelocity;
  float forcePerMotor;
  elapsedMicros moveTime;

  motionCalc motionCalc (distance, max_vel_straight, exit_speed, max_accel_straight, max_decel_straight);

  PID_Controller* left_PID = new PID_Controller(KP_POSITION,KI_POSITION,KD_POSITION);
  PID_Controller* right_PID = new PID_Controller(KP_POSITION,KI_POSITION,KD_POSITION);

  PID_Controller* rotation_PID = new PID_Controller(KP_ROTATION,KI_ROTATION,KD_ROTATION);

  // zero clock before move
  moveTime = 0;   

  // execute motion
  while (idealDistance != distance) {
    //Run sensor protocol here.  Sensor protocol should use encoder_left/right_write() to adjust for encoder error
    idealDistance = motionCalc.idealDistance(moveTime);
    idealVelocity = motionCalc.idealVelocity(moveTime);

    // Add error from rangefinder data.  Positive error is when it is too close to the left wall, requiring a positive angle to fix it.  
    RangeSensors.updateReadings();
    rotationOffset = rotation_PID->Calculate(RangeSensors.errorFromTarget(.5));

    errorLeft = enc_left_extrapolate() - idealDistance - rotationOffset;
    errorRight = enc_right_extrapolate() - idealDistance + rotationOffset;
    

    // Run PID to determine the offset that should be added/subtracted to the left/right wheels to fix the error.  Remember to remove or at the very least increase constraints on the I term
    // the offsets that are less than an encoder tick need to be added/subtracted from errorLeft and errorRight instead of encoderWrite being used.  Maybe add a third variable to the error calculation for these and other offsets


    
    
    // use instantaneous velocity of each encoder to calculate what the ideal PWM would be
    if (idealVelocity > 0) {
      forcePerMotor = (ROBOT_MASS * motionCalc.idealAccel(moveTime) + FRICTION_FORCE) / NUMBER_OF_MOTORS; // this is acceleration motors should have at a current time
    }
    else {
      forcePerMotor = (ROBOT_MASS * motionCalc.idealAccel(moveTime) - FRICTION_FORCE) / NUMBER_OF_MOTORS; // this is acceleration motors should have at a current time
    }

    leftOutput = left_motor_output_calculator.Calculate(forcePerMotor, idealVelocity);
    rightOutput = right_motor_output_calculator.Calculate(forcePerMotor, idealVelocity);

    //run PID loop here.  new PID loop will add or subtract from a predetermined PWM value that was calculated with the motor curve and current ideal speed
    // add PID error correction to ideal value
    leftOutput += left_PID->Calculate(errorLeft);
    rightOutput += right_PID->Calculate(errorRight);

    //Serial2.println(errorLeft);

    // set motors to run at specified rate
    motor_set(&motor_a, leftOutput);
    motor_set(&motor_b, rightOutput);
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
  float forcePerMotor;
  float rightOutput, leftOutput;
  elapsedMicros moveTime;
  
  motionCalc motionCalc (linearDistance, max_vel_rotate, 0, max_accel_rotate, max_decel_rotate);


  PID_Controller* left_PID = new PID_Controller(KP_POSITION,KI_POSITION,KD_POSITION);
  PID_Controller* right_PID = new PID_Controller(KP_POSITION,KI_POSITION,KD_POSITION);


  // zero encoders and clock before move
  moveTime = 0;

  // the right will always be the negative of the left in order to rotate on a point.
  while (idealLinearDistance != linearDistance) {
    //Run sensor protocol here.  Sensor protocol should use encoder_left/right_write() to adjust for encoder error
    idealLinearDistance = motionCalc.idealDistance(moveTime);
    idealLinearVelocity = motionCalc.idealVelocity(moveTime);
    
    errorLeft = enc_left_extrapolate() - idealLinearDistance;
    errorRight = enc_right_extrapolate() + idealLinearDistance;

    if (idealLinearVelocity > 0) {
      forcePerMotor = (ROBOT_MASS * motionCalc.idealAccel(moveTime) + FRICTION_FORCE) / NUMBER_OF_MOTORS; // this is acceleration motors should have at a current time
    }
    else {
      forcePerMotor = (ROBOT_MASS * motionCalc.idealAccel(moveTime) - FRICTION_FORCE) / NUMBER_OF_MOTORS; // this is acceleration motors should have at a current time
    }

    leftOutput = left_motor_output_calculator.Calculate(forcePerMotor, idealLinearVelocity);
    rightOutput = right_motor_output_calculator.Calculate(-forcePerMotor, idealLinearVelocity);

    leftOutput += left_PID->Calculate(errorLeft);
    rightOutput += right_PID->Calculate(errorRight);

    // set motors to run at specified rate
    motor_set(&motor_a, leftOutput);
    motor_set(&motor_b, rightOutput);
    
    
    //run PID loop here.  new PID loop will add or subtract from a predetermined PWM value that was calculated with the motor curve and current ideal speed

  }
  motor_set(&motor_a, 0);
  motor_set(&motor_b, 0);
  enc_left_write(0);
  enc_right_write(0);

}

void motion_corner(float angle, float radius, float exit_speed) {
  float errorRight, errorLeft;
  float rightOutput, leftOutput;
  float leftFraction, rightFraction;
  float idealDistance, idealVelocity;
  float forceLeftMotor, forceRightMotor;
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
  
  motionCalc motionCalc (distance, max_vel_corner, exit_speed, max_accel_corner, max_decel_corner);

  PID_Controller* left_PID = new PID_Controller(KP_POSITION,KI_POSITION,KD_POSITION);
  PID_Controller* right_PID = new PID_Controller(KP_POSITION,KI_POSITION,KD_POSITION);

  // zero clock before move
  moveTime = 0;   

  // execute motion
  while (idealDistance != distance) {
    //Run sensor protocol here.  Sensor protocol should use encoder_left/right_write() to adjust for encoder error
    idealDistance = motionCalc.idealDistance(moveTime);
    idealVelocity = motionCalc.idealVelocity(moveTime);

    errorLeft = enc_left_extrapolate() - idealDistance * rightFraction;
    errorRight = enc_right_extrapolate() - idealDistance * leftFraction;
    
    // use instantaneous velocity of each encoder to calculate what the ideal PWM would be
    if (idealVelocity * leftFraction > 0) {
      forceLeftMotor = (ROBOT_MASS * motionCalc.idealAccel(moveTime) * leftFraction + FRICTION_FORCE) / NUMBER_OF_MOTORS; // this is acceleration motors should have at a current time
    }
    else {
      forceLeftMotor = (ROBOT_MASS * motionCalc.idealAccel(moveTime) * leftFraction - FRICTION_FORCE) / NUMBER_OF_MOTORS; // this is acceleration motors should have at a current time
    }
    if (idealVelocity * rightFraction > 0) {
      forceRightMotor = (ROBOT_MASS * motionCalc.idealAccel(moveTime) * rightFraction + FRICTION_FORCE) / NUMBER_OF_MOTORS; // this is acceleration motors should have at a current time
    }
    else {
      forceRightMotor = (ROBOT_MASS * motionCalc.idealAccel(moveTime) * rightFraction - FRICTION_FORCE) / NUMBER_OF_MOTORS; // this is acceleration motors should have at a current time
    }

    leftOutput = left_motor_output_calculator.Calculate(forceLeftMotor, idealVelocity * leftFraction);
    rightOutput = right_motor_output_calculator.Calculate(forceRightMotor, idealVelocity * rightFraction);

    //run PID loop here.  new PID loop will add or subtract from a predetermined PWM value that was calculated with the motor curve and current ideal speed
    // add PID error correction to ideal value
    leftOutput += left_PID->Calculate(errorLeft);
    rightOutput += right_PID->Calculate(errorRight);

    // set motors to run at specified rate
    motor_set(&motor_a, leftOutput);
    motor_set(&motor_b, rightOutput);
  }
  enc_left_write(0);
  enc_right_write(0);
}

void motion_hold(int time)
{
  float errorRight, errorLeft;
  float rightOutput, leftOutput;
  elapsedMicros currentTime;

  PID_Controller* left_PID = new PID_Controller(KP_POSITION,KI_POSITION,KD_POSITION);
  PID_Controller* right_PID = new PID_Controller(KP_POSITION,KI_POSITION,KD_POSITION);

  currentTime = 0;   

  while (currentTime / 1000 < time) {
    
    errorLeft = enc_left_extrapolate();
    errorRight = enc_right_extrapolate();

    leftOutput = left_PID->Calculate(errorLeft);
    rightOutput = right_PID->Calculate(errorRight);

    motor_set(&motor_a, leftOutput);
    motor_set(&motor_b, rightOutput);
  }
  motor_set(&motor_a, 0);
  motor_set(&motor_b, 0);
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

#include "conf.h"
#include "motors.h"
#include <Arduino.h>
#include "sensors_encoders.h"
#include <string>
#include <stdio.h>


class motionCalc {
  private:
    float vStart, vEnd, vMax;
    float dStart, dEnd, dTot;
    float aStart, aEnd;
    int tStart, tConst, tEnd;

  public:
    motionCalc (float, float, float);
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

motionCalc::motionCalc (float a, float b, float c) {

  dTot = a;
  vMax = b;
  vEnd = c;

  // turn dTot into meters
  dTot /= 1000;

  // get current speed by averaging current instantaneous speed of left and right wheels
  vStart = (enc_left_velocity() + enc_right_velocity()) / 2;

  // set constants from global.  Do this a different way later to reference conf.h
  if (((dTot < 0) && (vMax > 0)) || ((dTot > 0) && (vMax < 0))) {
    vMax = -vMax;
  }
  if (vStart <= vMax) {
    aStart = MAX_ACCEL;
  }
  else {
    aStart = MAX_DECEL;
  }

  if (vEnd <= vMax) {
    aEnd = MAX_DECEL;
  }
  else {
    aEnd = MAX_ACCEL;
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






class PIDCorrectionCalculator {
  private:
    float i_term = 0;
    elapsedMicros elapsed_time;
    float last_error = 0;

  public:
    float Calculate(float error);
} ;

float PIDCorrectionCalculator::Calculate(float error) {
  i_term += KI * error * elapsed_time;
  i_term = constrain(i_term, -1, 1);

  float output = -KP * error - i_term + KD * (error - last_error) / elapsed_time;
  last_error = error;

  elapsed_time = 0;

  return output;
}


void motion_forward(float distance, float exit_speed) {
  float errorRight, errorLeft;
  float rightOutput, leftOutput;
  float idealDistance, idealVelocity;
  float forcePerMotor;
  elapsedMicros moveTime;

  motionCalc motionCalc (distance, MAX_VELOCITY_STRAIGHT, exit_speed);

  PIDCorrectionCalculator* left_PID_calculator = new PIDCorrectionCalculator();
  PIDCorrectionCalculator* right_PID_calculator = new PIDCorrectionCalculator();

  // zero clock before move
  moveTime = 0;   

  // execute motion
  while (idealDistance != distance) {
    //Run sensor protocol here.  Sensor protocol should use encoder_left/right_write() to adjust for encoder error
    idealDistance = motionCalc.idealDistance(moveTime);
    idealVelocity = motionCalc.idealVelocity(moveTime);

    errorLeft = enc_left_extrapolate() - idealDistance;
    errorRight = enc_right_extrapolate() - idealDistance;
    
    
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
    leftOutput += left_PID_calculator->Calculate(errorLeft);
    rightOutput += right_PID_calculator->Calculate(errorRight);

    //Serial2.println(errorLeft);

    // set motors to run at specified rate
    motor_set(&motor_a, leftOutput);
    motor_set(&motor_b, rightOutput);
  }
  enc_left_write(0);
  enc_right_write(0);


    //String stuff
    //Serial2.printf("%s", outputString.c_str());
  
  
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
  
  motionCalc motionCalc (linearDistance, MAX_VELOCITY_ROTATE, 0);

  PIDCorrectionCalculator* left_PID_calculator = new PIDCorrectionCalculator();
  PIDCorrectionCalculator* right_PID_calculator = new PIDCorrectionCalculator();

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

    leftOutput += left_PID_calculator->Calculate(errorLeft);
    rightOutput += right_PID_calculator->Calculate(errorRight);

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
  
  motionCalc motionCalc (distance, MAX_VELOCITY_CORNER, exit_speed);

  PIDCorrectionCalculator* left_PID_calculator = new PIDCorrectionCalculator();
  PIDCorrectionCalculator* right_PID_calculator = new PIDCorrectionCalculator();

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
    leftOutput += left_PID_calculator->Calculate(errorLeft);
    rightOutput += right_PID_calculator->Calculate(errorRight);

    // set motors to run at specified rate
    motor_set(&motor_a, leftOutput);
    motor_set(&motor_b, rightOutput);
  }
  enc_left_write(0);
  enc_right_write(0);
  
  motor_set(&motor_a, 0);
  motor_set(&motor_b, 0);
  
}

void motion_hold(int time)
{
  float errorRight, errorLeft;
  float rightOutput, leftOutput;
  float forcePerMotor;
  elapsedMicros currentTime;

  PIDCorrectionCalculator* left_PID_calculator = new PIDCorrectionCalculator();
  PIDCorrectionCalculator* right_PID_calculator = new PIDCorrectionCalculator();

  currentTime = 0;   

  while (currentTime / 1000 < time) {
    
    errorLeft = enc_left_extrapolate();
    errorRight = enc_right_extrapolate();

    leftOutput = left_PID_calculator->Calculate(errorLeft);
    rightOutput = right_PID_calculator->Calculate(errorRight);

    motor_set(&motor_a, leftOutput);
    motor_set(&motor_b, rightOutput);
  }
  motor_set(&motor_a, 0);
  motor_set(&motor_b, 0);
}

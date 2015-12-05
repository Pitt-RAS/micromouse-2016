#include "conf.h"
#include "motors.h"
#include <Arduino.h>
#include "sensors_encoders.h"


class motionCalc { 
  private:  
    float vStart, vEnd, vMax;
    float dStart, dEnd, dTot;
    float aStart, aEnd;
    int tSpeed, tConst, tSlow;
    
  public:
    motionCalc (float,float,float);
    float idealDistance (int elapsedTime) {
      if (elapsedTime < tSpeed) {
        return ( elapsedTime / 1000000 * (vStart + .5 * aStart * elapsedTime / 1000000));  
      }
      else if (elapsedTime < (tSpeed + tConst)) {
        return (dStart + vMax * (elapsedTime - tSpeed) / 1000000);
      }
      else if (elapsedTime < (tSpeed + tConst + tSlow)) {
        return ((dTot - dEnd) + (elapsedTime - tSpeed - tConst) / 1000000 * (vMax + .5 * aEnd * (elapsedTime - tSpeed - tConst) / 1000000));
      }
      else {
        return (dTot);
      }
    }
    float idealVelocity (int elapsedTime) {
      if (elapsedTime <= tSpeed) {
        return (vStart + aStart * elapsedTime / 1000000);  
      }
      else if (elapsedTime < (tSpeed + tConst)) {
        return (vMax);
      }
      else if (elapsedTime < (tSpeed + tConst + tSlow)) {
        return (vMax + aEnd * (elapsedTime - tSpeed - tConst) / 1000000);
      }
      else {
        return (vEnd);
      }
    }
    float idealAccel (int elapsedTime) {
      if (elapsedTime <= tSpeed) {
        return (aStart);  
      }
      else if (elapsedTime < (tSpeed + tConst)) {
        return (0);
      }
      else if (elapsedTime < (tSpeed + tConst + tSlow)) {
        return (aEnd);
      }
      else {
        return (0);
      }
    }
};

motionCalc::motionCalc (float dTot, float vMax, float vEnd) {

  // get current speed by averaging current instantaneous speed of left and right wheels
  vStart = (enc_left_velocity() + enc_right_velocity()) / 2;
  
  // set constants from global.  Do this a different way later to reference conf.h
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
  if (((dTot < 0) && (vMax > 0)) || ((dTot > 0) && (vMax < 0))) {
    vMax = -vMax;
  }

  // do initial calculations
  
  // set distances assuming there is room to reach max speed
  dStart = (vMax * vMax - vStart * vStart) / (2 * aStart);
  dEnd = (vEnd * vEnd - vMax * vMax) / (2 * aStart);
  tConst = 1000000 * (dTot - dStart - dEnd) / vMax;
  
  // set distances if there is not space to reach max speed
  if (tConst < 0) {
    dStart = vEnd * vEnd - vStart * vStart - 2 * dTot * aEnd;
    dEnd = dTot - dStart;
    tConst = 0;
  }

  // calculate tSpeed and tSlow based on dStart and dEnd
  tSpeed = (1000000 * (-vStart + sqrt(vStart*vStart + 2 * aStart * dStart)) / aStart);
  tSlow =  (1000000 * (-vEnd + sqrt(vEnd*vEnd + 2 * -aEnd * dEnd)) / -aEnd);
}


// input current speed and 
class idealMotorOutputCalculator {
  private: 
    float currentSpeed, desiredForce;
    float motorOutput;
    float aTerm, bTerm;

  public:
    idealMotorOutputCalculator();
      float Calculate (float desiredForce, float currentSpeed) {
        return (((currentSpeed + desiredForce * aTerm) * bTerm) / BATTERY_VOLTAGE);
      }
} left_motor_output_calculator, right_motor_output_calculator;

idealMotorOutputCalculator::idealMotorOutputCalculator() {
  // calcluate these terms initially in case float math isn't done precompile
  aTerm = RATED_FREERUN_VELOCITY / RATED_STALL_FORCE;
  bTerm = RATED_VOLTAGE / RATED_FREERUN_VELOCITY;
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
  float idealDistance;
  float forcePerMotor;
  elapsedMicros moveTime;

  motionCalc straightMove (distance, MAX_VELOCITY_STRAIGHT, exit_speed);

  PIDCorrectionCalculator* left_PID_calculator = new PIDCorrectionCalculator();
  PIDCorrectionCalculator* right_PID_calculator = new PIDCorrectionCalculator();
  
  // zero encoders and clock before move
  enc_left_write(0);
  enc_right_write(0);
  moveTime = 0;

  // execute motion
  while (idealDistance != distance) {
    //Run sensor protocol here.  Sensor protocol should use encoder_left/right_write() to adjust for encoder error
    idealDistance = straightMove.idealDistance(moveTime);
    errorLeft = idealDistance - enc_left_read();
    errorRight = idealDistance - enc_right_read();

    // use instantaneous velocity of each encoder to calculate what the ideal PWM would be
    forcePerMotor = (ROBOT_MASS * straightMove.idealAccel(moveTime) + FRICTION_FORCE) / 2; // this is acceleration motors should have at a current time
    
    leftOutput = left_motor_output_calculator.Calculate(forcePerMotor, enc_left_velocity());
    rightOutput = right_motor_output_calculator.Calculate(forcePerMotor, enc_right_velocity());

    //run PID loop here.  new PID loop will add or subtract from a predetermined PWM value that was calculated with the motor curve and current ideal speed
    // add PID error correction to ideal value
    leftOutput += left_PID_calculator->Calculate(errorLeft);
    rightOutput += right_PID_calculator->Calculate(errorRight);

    // set motors to run at specified rate
    //motor_set(&motor_a, leftOutput);
    //motor_set(&motor_b, rightOutput);
  }

 
}

// clockwise angle is positive, angle is in degrees
void motion_rotate(float angle) {
  float distancePerDegree = 3.14159265359 * MM_BETWEEN_WHEELS / 360;
  float idealLinearDistance, errorLeft, errorRight;
  float linearDistance = distancePerDegree * angle;
  motionCalc rotate (linearDistance, MAX_VELOCITY_ROTATE, 0);
  elapsedMicros moveTime;

  // zero encoders and clock before move
  enc_left_write(0);
  enc_right_write(0);
  moveTime = 0;

  // the right will always be the negative of the left in order to rotate on a point.
  while (idealLinearDistance != linearDistance) {
    //Run sensor protocol here.  Sensor protocol should use encoder_left/right_write() to adjust for encoder error
    idealLinearDistance = rotate.idealDistance(moveTime);
    errorLeft = idealLinearDistance - enc_left_read();
    errorRight = -idealLinearDistance - enc_right_read();
    
    // use instantaneous velocity of each encoder to calculate what the ideal PWM would be
    //idealAccel = rotate.idealAccel(moveTime); // this is acceleration motors should have at a current time
    

    //run PID loop here.  new PID loop will add or subtract from a predetermined PWM value that was calculated with the motor curve and current ideal speed
    
  }
  
  
}

void motion_corner(float angle) {

}

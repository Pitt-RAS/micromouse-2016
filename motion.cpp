#include "conf.h"
#include "motors.h"
#include <Arduino.h>
#include "sensors_encoders.h"

/*working variables*/
unsigned long lastTime;
double Input1, Output1, Setpoint;
double Input2, Output2;
double ITerm1, lastInput1;
double ITerm2, lastInput2;
double kp, ki, kd;
int SampleTime = 1000; // in microseconds
double outMin, outMax;



#define DIRECT 0
#define REVERSE 1
int controllerDirection = DIRECT;

IntervalTimer myTimer;


static float max_speed;
static float max_accel;

int initialized = 0;

static void initialize();






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

motionCalc::motionCalc (float a, float b, float c) {

  dTot = a;
  vMax = b;
  vEnd = c;

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






void Compute()
{
  static double inc = 0;
  static bool done_accelerating = false;
  if (!done_accelerating && inc < 3.8) {
      Setpoint += inc;
      inc += 0.0019;
  } else if (inc > 0) {
    done_accelerating = true;
    Setpoint += inc;
    inc -= 0.0019;
  }
      
      Input1 = (double) enc_left_read();

      /*Compute all the working error variables*/
      double error1 = Setpoint - Input1;
      ITerm1+= (ki * error1);
      if(ITerm1 > outMax) ITerm1= outMax;
      else if (ITerm1 < outMin) ITerm1 = outMin;
      double dInput1 = (Input1 - lastInput1);
 
      /*Compute PID Output*/
      Output1 = kp * error1 + ITerm1 - kd * dInput1;
      if(Output1 > outMax) Output1 = outMax;
      else if(Output1 < outMin) Output1 = outMin;
 
      /*Remember some variables for next time*/
      lastInput1 = Input1;  

      Input2 = (double) enc_right_read();

      /*Compute all the working error variables*/
      double error2 = Setpoint - Input2;
      ITerm2 += (ki * error2);
      if(ITerm2 > outMax) ITerm2 = outMax;
      else if(ITerm2 < outMin) ITerm2 = outMin;
      double dInput2 = (Input2 - lastInput2);
 
      /*Compute PID Output*/
      Output2 = kp * error2 + ITerm2 - kd * dInput2;
      if(Output2 > outMax) Output2 = outMax;
      else if(Output2 < outMin) Output2 = outMin;
 
      /*Remember some variables for next time*/
      lastInput2 = Input2;  

      motor_run(&motor_a, Output1);
      motor_run(&motor_b, Output2);
}
 
void SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0|| Kd<0) return;
 
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
 
  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}
 
void SetOutputLimits(double Min, double Max)
{
   if(Min > Max) return;
   outMin = Min;
   outMax = Max;
 
   if(Output1 > outMax) Output1 = outMax;
   else if(Output1 < outMin) Output1 = outMin;
   if(Output2 > outMax) Output2 = outMax;
   else if(Output2 < outMin) Output2 = outMin;
 
   if(ITerm1 > outMax) ITerm1 = outMax;
   else if(ITerm1 < outMin) ITerm1 = outMin;

   if(ITerm2 > outMax) ITerm2 = outMax;
   else if(ITerm2 < outMin) ITerm2 = outMin;
}
 
 
void Initialize()
{
   lastInput1 = Input1;
   lastInput2 = Input2;
   ITerm1 = Output1;
   ITerm2 = Output2;
   if(ITerm1 > outMax) ITerm1= outMax;
   if(ITerm2 > outMax) ITerm2= outMax;
   else if(ITerm1 < outMin) ITerm1= outMin;
   else if(ITerm2 < outMin) ITerm2= outMin;
}
 
void SetControllerDirection(int Direction)
{
   controllerDirection = Direction;
}

void motion_set_max_speed(float new_max_speed) {

}

void motion_set_max_accel(float new_max_accel) {

}

void motion_forward(float distance, float exit_speed) {
	float errorRight, errorLeft;
  float idealDistance;
  elapsedMicros moveTime;
	
	if (!initialized) initialize(); // ?? I don't know what this means

  motionCalc straightMove (distance, MAX_VELOCITY_STRAIGHT, exit_speed);
  
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
    idealAccel = strightMove.idealAccel(moveTime); // this is acceleration motors should have at a current time
    

    //run PID loop here.  new PID loop will add or subtract from a predetermined PWM value that was calculated with the motor curve and current ideal speed
  }

 
}

// clockwise angle is positive, angle is in degrees
void motion_rotate(float angle) {
  float distancePerDegree = 3.14159265359 * MM_BETWEEN_WHEELS / 360;
  float idealLinearDistance, errorLeft, errorRight;
  linearDistance = distancePerDegree * angle;
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
    errorRigth = -idealLinearDistance - enc_right_read();
    
    // use instantaneous velocity of each encoder to calculate what the ideal PWM would be
    idealAccel = rotate.idealAccel(moveTime); // this is acceleration motors should have at a current time
    

    //run PID loop here.  new PID loop will add or subtract from a predetermined PWM value that was calculated with the motor curve and current ideal speed
  }
  
  
}

void motion_corner(float angle) {

}

void motion_update() {

}

static void initialize() {
	SetTunings(0.1, 0.002, 0.0);
	SetOutputLimits(0.0, 1.0);
	Initialize();
	Setpoint = 1.0;
	myTimer.begin(Compute, SampleTime);
}


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
	if (!initialized) initialize();
}

void motion_rotate(float angle) {

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


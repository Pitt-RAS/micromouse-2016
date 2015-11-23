#include "conf.h"
#include "motors.h"


/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double ITerm, lastInput;
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
	setTunings(0.1, 0.0, 0.0);
	setOutputLimits(-1.0, 1.0);
	Initialize();
	Setpoint = 0.0;
	myTimer.begin(Compute, SampleTime);
}

/////////


void Compute()
{
	Setpoint += 0.063;
	Input = (double) enc_left_read();

      /*Compute all the working error variables*/
      double error = Setpoint - Input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      double dInput = (Input - lastInput);
 
      /*Compute PID Output*/
      Output = kp * error + ITerm- kd * dInput;
      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
 
      /*Remember some variables for next time*/
      lastInput = Input;  

      motor_run(&motor_a, Output);
      motor_run(&motor_b, Output);
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
 
   if(Output > outMax) Output = outMax;
   else if(Output < outMin) Output = outMin;
 
   if(ITerm > outMax) ITerm= outMax;
   else if(ITerm < outMin) ITerm= outMin;
}
 
 
void Initialize()
{
   lastInput = Input;
   ITerm = Output;
   if(ITerm > outMax) ITerm= outMax;
   else if(ITerm < outMin) ITerm= outMin;
}
 
void SetControllerDirection(int Direction)
{
   controllerDirection = Direction;
}

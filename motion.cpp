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





// Current limitation of the class is that all movement is assumed to be in the positive direction
//   Also assumes that Vstart is always below or very close to Vmax, 
//   both of these are due to the assumption that the start of the move must be positive or 0 acceleration
class forwardDistance { 
    double Vexit, Vmax, Vstart;
    double Dtot, Dspeed, Dslow;
    double aSpeed, aSlow;
    int tSpeed, tConst, tSlow;
    elapsedMicros moveTime;

    // set constants from global.  Do this a different way later to reference conf.h
    Vmax = 1;   // m/s
    aSpeed = 4; // m/s/s
    aSlow = -4; // m/s/s
    
    // do initial calculations
    Vstart = (enc_left_velocity() + enc_right_velocity()) / 2;
    enc_left_write(0);
    enc_left_write(0);
    if (Vstart > Vmax) {
      Vstart = Vmax;
    }
    
    // set distances assuming there is room to reach max speed
    Dspeed = (Vmax * Vmax - Vstart * Vstart) / (2 * aSpeed);
    Dslow = (Vexit * Vexit - Vmax * Vmax) / (2 * aSpeed);
    tConst = 1000000 * (Dtot - Dspeed - Dslow) / Vmax;
    
    // set distances if there is not space to reach max speed
    if (tConst < 0) {
      Dspeed = Vexit * Vexit - Vstart * Vstart - 2 * Dtot * aSlow;
      Dslow = Dtot - Dspeed;
      tConst = 0;
    }

    // calculate tSpeed and tSlow based on Dspeed and Dslow
    tSpeed = 1000000 * (-Vstart + sqrt(Vstart*Vstart + 2 * aSpeed * Dspeed)) / aSpeed;
    tSlow =  1000000 * (-Vexit + sqrt(Vexit*Vexit + 2 * -aSlow * Dslow)) / -aSlow;

    // reset moveTime to 0 for wasted calculation time
    moveTime = 0;
    
  public:
    forwardDistance (double,double);
    double idealDistance () {
      if (moveTime < tSpeed) {
        return ( moveTime / 1000000 * (Vstart + .5 * aSpeed * moveTime / 1000000));  
      }
      else if (moveTime < (tSpeed + tConst)) {
        return (Dspeed + Vmax * (moveTime - tSpeed) / 1000000);
      }
      else if (moveTime < (tSpeed + tConst + tStop)) {
        return ((Dtot - Dslow) + (moveTime - tSpeed - tConst) / 1000000 * (Vmax + .5 * aSlow * (moveTime - tSpeed - tConst) / 1000000));
      }
      else {
        return (Dtot);
      }
    }
    double idealVelocity () {
      if (moveTime <= tSpeed) {
        return (Vstart + aSpeed * moveTime / 1000000);  
      }
      else if (moveTime < (tSpeed + tConst)) {
        return (Vmax);
      }
      else if (moveTime < (tSpeed + tConst + tStop)) {
        return (Vmax + aSlow * (moveTime - tSpeed - tConst) / 1000000);
      }
      else {
        return (Vexit);
      }
    }
};

forwardDistance::forwardDistance (double d, double v) {
  Dtot = d;
  Vexit = v;
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
	if (!initialized) initialize(); // ?? I don't know what this means

  forwardDistance Movement (distance,exit_speed);

  while (Movement.idealDistance() != distance) {
    //Run sensor protocol here.  Sensor protocol should use encoder_left/right_write() to adjust for encoder error
    errorLeft = Movement.idealDistance() - enc_left_read();
    errorRight = Movement.idealDistance() - enc_right_read();

    //run PID loop here.  new PID loop will add or subtract from a predetermined PWM value that was calculated with the motor curve and current ideal speed
  }

 
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


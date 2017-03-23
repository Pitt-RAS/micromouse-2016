#ifndef TRACKER_H
#define TRACKER_H

#include "../FFWFunction.h"
#include "../Profile.h"
#include "LinearFFW.h"
#include "RotationalFFW.h"
#include "LinearCorrector.h"
#include "RotationalCorrector.h"

namespace Motion {

struct TrackerOptions
{
  FFWParameters     linear_ffw_parameters = { 0.0, 0.0 };
  FFWParameters rotational_ffw_parameters = { 0.0, 0.0 };

  PIDParameters encoder_pid_parameters = { 0.0, 0.0, 0.0 };
  PIDParameters    gyro_pid_parameters = { 0.0, 0.0, 0.0 };
  PIDParameters   range_pid_parameters = { 0.0, 0.0, 0.0 };

  bool diagonal_correction = false;

  bool drift = false;

  enum {
    kFinalTime, // when time reaches profile finalTime()
    kGyroAngle  // when gyro passes provided angle
  } end_condition = kFinalTime;

  union {
    AngleUnit angle;
  } end_condition_data = { .angle = AngleUnit::zero() };

  bool end_plant = true;

  bool gyro_safety_check = true;
  bool encoder_safety_check = true;
};

class Tracker
{
  public:
    Tracker(TrackerOptions options, LinearRotationalProfile &profile);

    void run();

  private:
    TrackerOptions options_;
    LinearRotationalProfile &profile_;

    LinearFFW linear_ffw_;
    RotationalFFW rotational_ffw_;
    LinearCorrector linear_corrector_;
    RotationalCorrector rotational_corrector_;

    void reset();
    bool endConditionMet(TimeUnit time);
    void encoderSafetyCheck(LinearRotationalPoint point);
    void gyroSafetyCheck(LinearRotationalPoint point);
    void transition();
};

}

#endif

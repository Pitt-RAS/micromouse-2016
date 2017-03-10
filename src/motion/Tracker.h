#ifndef TRACKER_H
#define TRACKER_H

#include "../device/Encoder.h"
#include "../device/Motor.h"
#include "FFWFunction.h"
#include "Matrix.h"
#include "PIDFunction.h"
#include "Profile.h"

namespace Motion {

struct TrackerOptions
{
  FFWParameters     linear_ffw_parameters = { 0.0, 0.0 };
  FFWParameters rotational_ffw_parameters = { 0.0, 0.0 };

  PIDParameters encoder_pid_parameters = { 0.0, 0.0, 0.0 };
  PIDParameters    gyro_pid_parameters = { 0.0, 0.0, 0.0 };
  PIDParameters   range_pid_parameters = { 0.0, 0.0, 0.0 };

  bool diagonal_correction = false;

  enum {
    kFinalTime, // when time reaches profile finalTime()
    kGyroAngle  // when gyro passes provided angle
  } end_condition = kFinalTime;

  union {
    AngleUnit angle;
  } end_condition_data = { .angle = AngleUnit::zero() };

  bool end_plant = true;
};

class Tracker
{
  public:
    Tracker(TrackerOptions options, LinearRotationalProfile &profile);

    void run();

  private:
    TrackerOptions options_;
    LinearRotationalProfile &profile_;

    Matrix<PIDFunction> encoder_pid_;
    PIDFunction gyro_pid_;
    PIDFunction range_pid_;

    FFWFunction linear_ffw_;
    FFWFunction rotational_ffw_;

    TimeUnit time_;

    LinearRotationalPoint point_;

    bool endConditionMet();

    void safetyCheck();

    void setOutput();
    void transition();

    Matrix<double> linearFFW();
    Matrix<double> rotationalFFW();

    Matrix<double> linearFFW(LinearRotationalPoint point);
    Matrix<double> rotationalFFW(LinearRotationalPoint point);

    Matrix<double> linearCorrection();
    Matrix<double> rotationalCorrection();
};

}

#endif

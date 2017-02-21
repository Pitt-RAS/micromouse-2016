#ifndef TRACKER_H
#define TRACKER_H

#include "PIDFunction.h"
#include "Profile.h"
#include "SkidSteerCar.h"

namespace Motion {

struct TrackerOptions
{
  PIDParameters wheel_pid_parameters = { 0.0, 0.0, 0.0 };
  PIDParameters range_pid_parameters = { 0.0, 0.0, 0.0 };
  PIDParameters  gyro_pid_parameters = { 0.0, 0.0, 0.0 };
};

class Tracker
{
  public:
    Tracker(TrackerOptions options, LinearRotationalProfile &profile);

    void run();

  private:
    WheelOptions wheelOptions(TrackerOptions options);

    void safetyCheck(LinearRotationalPoint &point);

    void addGyroHeading(LinearRotationalPoint &point);
    void addRangeCorrection(LinearRotationalPoint &point);

    LengthUnit kRightWheelX = LengthUnit::fromMeters(0.03725);
    LengthUnit kLeftWheelX = LengthUnit::fromAbstract(-kRightWheelX.abstract());

    TrackerOptions options_;
    LinearRotationalProfile &profile_;

    WheelOnBody wheels_on_body_[4];
    StraightenedSkidSteerCar<4> car_;

    PIDFunction range_pid_;
};

}

#endif

#ifndef ROTATIONAL_CORRECTOR_H
#define ROTATIONAL_CORRECTOR_H

#include "../PIDFunction.h"
#include "../StabilityChecker.h"
#include "StatefulOutput.h"

namespace Motion {

class GyroResetter
{
  public:
    GyroResetter();

    void point(LinearRotationalPoint point);

  private:
    static constexpr unsigned kPoints = 0;
    static const LengthUnit kTotalDistance;

    static const LengthUnit kDistancePerPoint;

    static const AngleUnit kGyroLimit;
    static const AngleUnit kRangeLimit;

    void iteration(LinearRotationalPoint point);

    StabilityChecker<kPoints> gyro_;
    StabilityChecker<kPoints> range_;

    int counter_ = -1;
};

class RotationalCorrector : public StatefulOutput
{
  public:
    RotationalCorrector(PIDParameters gyro_pid_parameters,
                                          PIDParameters range_pid_parameters);

    virtual Matrix<double> output(LinearRotationalPoint target);

    virtual void reset();

  private:
    void updateResetter(LinearRotationalPoint target);

    double gyroResponse(LinearRotationalPoint target);
    double rangeResponse(LinearRotationalPoint target);

    PIDFunction gyro_pid_;
    PIDFunction range_pid_;

    GyroResetter gyro_resetter_;
};

}

#endif

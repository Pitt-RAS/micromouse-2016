#include "../../device/Orientation.h"
#include "../../device/RangeSensorContainer.h"
#include "../../user_interaction/FreakOut.h"
#include "../maneuver/Plant.h"
#include "../matrix/EncoderMatrix.h"
#include "../matrix/MotorMatrix.h"
#include "Tracker.h"

namespace Motion {

namespace {
  // stand-in for point.equals(LinearRotationalPoint::zero())
  //   because TrapezoidalProfile doesn't produce exactly zero when it should
  bool isMoving(LinearRotationalPoint point)
  {
    return point.linear_point.velocity.meters() > 0.001
            || point.rotational_point.velocity.degrees() > 0.1;
  }
}

Tracker::Tracker(TrackerOptions options, LinearRotationalProfile &profile) :
  options_(options),
  profile_(profile),
  linear_ffw_(options.linear_ffw_parameters),
  rotational_ffw_(options.rotational_ffw_parameters),
  linear_corrector_(options.encoder_pid_parameters),
  rotational_corrector_(options.gyro_pid_parameters,
                                              options.range_pid_parameters)
{}

void Tracker::run()
{
  reset();

  elapsedMicros timer;
  TimeUnit time = TimeUnit::fromSeconds(timer / 1e6);

  LinearRotationalPoint point = profile_.pointAtTime(time);

  Orientation::getInstance()->handler_update_ = false;

  while (!endConditionMet(time)) {
    point = profile_.pointAtTime(time);

    safetyCheck(point);

    gMotor.voltage(
      Matrix<double>::zeros()
        .add(linear_ffw_.output(point))
        .add(rotational_ffw_.output(point))
        .add(linear_corrector_.output(point))
        .add(rotational_corrector_.output(point))
    );

    time = TimeUnit::fromSeconds(timer / 1e6);
  }

  transition();

  Orientation::getInstance()->handler_update_ = true;
}

void Tracker::reset()
{
  linear_corrector_.reset();
  rotational_corrector_.reset();
}

bool Tracker::endConditionMet(TimeUnit time)
{
  switch (options_.end_condition) {
    case TrackerOptions::kFinalTime:
      if (time.abstract() > profile_.finalTime().abstract())
        return true;
      break;

    case TrackerOptions::kGyroAngle:
      AngleUnit current = AngleUnit::fromDegrees(
                                    -Orientation::getInstance()->getHeading());
      AngleUnit limit = options_.end_condition_data.angle;

      if (std::fabs(current.abstract()) > std::fabs(limit.abstract()))
        return true;
      break;
  }

  return false;
}

void Tracker::safetyCheck(LinearRotationalPoint point)
{
  {
    AngleUnit current = AngleUnit::fromDegrees(
                                    -Orientation::getInstance()->getHeading());
    AngleUnit  target = point.rotational_point.displacement;
    AngleUnit   limit = AngleUnit::fromDegrees(60.0);

    if (std::fabs(current.abstract() - target.abstract()) > limit.abstract())
      freakOut("GYRO");
  }
  {
    LengthUnit current = gEncoder.averageDisplacement();
    LengthUnit  target = point.linear_point.displacement;
    LengthUnit   limit = LengthUnit::fromMeters(0.05);

    if (std::fabs(current.abstract() - target.abstract()) > limit.abstract())
      freakOut("ENCR");
  }
}

void Tracker::transition()
{
  TimeUnit final_time = profile_.finalTime();
  LinearRotationalPoint final_point = profile_.pointAtTime(final_time);

  gMotor.voltage(
    Matrix<double>::zeros()
      .add(linear_ffw_.output(final_point))
      .add(rotational_ffw_.output(final_point))
  );

  AngleUnit final_heading = final_point.rotational_point.displacement;
  Orientation::getInstance()->incrementHeading(final_heading.degrees());

  if (options_.end_plant && !isMoving(final_point)) {
    gEncoder.zeroLinearDisplacement(final_point.linear_point.displacement);
    Plant().run();
  }
  else {
    gEncoder.zero();
  }
}

}

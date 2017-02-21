#include "../device/Orientation.h"
#include "../device/RangeSensorContainer.h"
#include "../user_interaction/FreakOut.h"
#include "Tracker.h"

namespace Motion {

Tracker::Tracker(TrackerOptions options, LinearRotationalProfile &profile) :
  options_(options),
  profile_(profile),
  wheels_on_body_{
    {
      Wheel(wheelOptions(options), motor_lf, gEncoderLF),
      PointOnBody(kLeftWheelX, LengthUnit::zero())
    },
    {
      Wheel(wheelOptions(options), motor_lb, gEncoderLB),
      PointOnBody(kLeftWheelX, LengthUnit::zero())
    },
    {
      Wheel(wheelOptions(options), motor_rf, gEncoderRF),
      PointOnBody(kRightWheelX, LengthUnit::zero())
    },
    {
      Wheel(wheelOptions(options), motor_rb, gEncoderRB),
      PointOnBody(kRightWheelX, LengthUnit::zero())
    }
  },
  car_(wheels_on_body_),
  range_pid_(options_.range_pid_parameters)
{}

void Tracker::run()
{
  elapsedMicros timer;
  TimeUnit time = TimeUnit::fromSeconds(timer / 1e6);

  Orientation *orientation = Orientation::getInstance();

  orientation->handler_update_ = false;

  while (time.abstract() < profile_.finalTime().abstract()) {
    LinearRotationalPoint point = profile_.pointAtTime(time);

    safetyCheck(point);

    addGyroHeading(point);
    addRangeCorrection(point);

    car_.reference(point);
    car_.update(time);

    time = TimeUnit::fromSeconds(timer / 1e6);
  }

  orientation->incrementHeading(-orientation->getHeading());
  car_.transition();

  orientation->handler_update_ = true;
}

WheelOptions Tracker::wheelOptions(TrackerOptions options)
{
  WheelOptions result;
  result.pid_parameters = options.wheel_pid_parameters;

  return result;
}

void Tracker::safetyCheck(LinearRotationalPoint &point)
{
  Orientation *orientation = Orientation::getInstance();

  AngleUnit heading = AngleUnit::fromDegrees(- orientation->getHeading());
  AngleUnit target = point.rotational_point.displacement;
  AngleUnit limit = AngleUnit::fromDegrees(60.0);

  if (std::fabs(heading.abstract() - target.abstract()) > limit.abstract())
    freakOut("GYRO");
}

void Tracker::addGyroHeading(LinearRotationalPoint &point)
{
  Orientation *orientation = Orientation::getInstance();

  orientation->update();

  double heading_degrees = - orientation->getHeading();
  AngleUnit heading = AngleUnit::fromDegrees(heading_degrees);

  double abstract = point.rotational_point.displacement.abstract();
  abstract -= heading.abstract();
  point.rotational_point.displacement = AngleUnit::fromAbstract(abstract);
}

void Tracker::addRangeCorrection(LinearRotationalPoint &point)
{
  RangeSensors.updateReadings();

  double error = - RangeSensors.errorFromCenter();
  double response = range_pid_.response(error, 0);

  double radians = point.rotational_point.displacement.radians();
  radians += response;
  point.rotational_point.displacement = AngleUnit::fromRadians(radians);
}

}

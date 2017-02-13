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
  car_(wheels_on_body_)
{}

void Tracker::run()
{
  PIDFunction range_pid(options_.range_pid_parameters);
  PIDFunction gyro_pid(options_.gyro_pid_parameters);

  elapsedMicros timer;
  TimeUnit time = TimeUnit::fromSeconds(timer / 1e6);

  Orientation::getInstance()->handler_update_ = false;

  while (time.abstract() < profile_.finalTime().abstract()) {
    LinearRotationalPoint point = profile_.pointAtTime(time);

    addRange(range_pid, point);
    addGyro(gyro_pid, point);

    car_.reference(point);
    car_.update(time);

    time = TimeUnit::fromSeconds(timer / 1e6);
  }

  Orientation::getInstance()->handler_update_ = true;
}

WheelOptions Tracker::wheelOptions(TrackerOptions options)
{
  WheelOptions result;
  result.pid_parameters = options.wheel_pid_parameters;

  return result;
}

void Tracker::addRange(PIDFunction &pid, LinearRotationalPoint &point)
{
  RangeSensors.updateReadings();

  double error = RangeSensors.errorFromCenter();
  double response = pid.response(error, 0);

  double radians = point.rotational_point.displacement.radians();
  radians += response;
  point.rotational_point.displacement = AngleUnit::fromRadians(radians);
}

void Tracker::addGyro(PIDFunction &pid, LinearRotationalPoint &point)
{
  Orientation::getInstance()->update();

  double error = Orientation::getInstance()->getHeading();

  if (std::fabs(error) > 60)
    freakOut("GYRO"); //consider more verbose logging

  double response = pid.response(error, 0);

  double radians = point.rotational_point.displacement.radians();
  radians += response;
  point.rotational_point.displacement = AngleUnit::fromRadians(radians);
}

}

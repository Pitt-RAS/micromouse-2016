#include "../device/Orientation.h"
#include "../device/RangeSensorContainer.h"
#include "../user_interaction/FreakOut.h"
#include "Tracker.h"

namespace Motion {

Tracker::Tracker(TrackerOptions options, LinearRotationalProfile &profile) :
  options_(options),
  profile_(profile),
  encoder_pid_(PIDMatrix(options_.encoder_pid_parameters)),
  gyro_pid_(options_.gyro_pid_parameters),
  range_pid_(options_.range_pid_parameters),
  linear_ffw_(options.linear_ffw_parameters),
  rotational_ffw_(options.rotational_ffw_parameters),
  point_(profile_.pointAtTime(TimeUnit::zero()))
{}

void Tracker::run()
{
  elapsedMicros timer;
  TimeUnit time = TimeUnit::fromSeconds(timer / 1e6);

  Orientation *orientation = Orientation::getInstance();

  orientation->handler_update_ = false;

  while (time.abstract() < profile_.finalTime().abstract()) {
    point_ = profile_.pointAtTime(time);

    safetyCheck();

    setOutput();

    time = TimeUnit::fromSeconds(timer / 1e6);
  }

  transition();

  orientation->handler_update_ = true;
}

void Tracker::safetyCheck()
{
  Orientation *orientation = Orientation::getInstance();

  AngleUnit heading = AngleUnit::fromDegrees(- orientation->getHeading());
  AngleUnit target = point_.rotational_point.displacement;
  AngleUnit limit = AngleUnit::fromDegrees(60.0);

  if (std::fabs(heading.abstract() - target.abstract()) > limit.abstract())
    freakOut("GYRO");
}

void Tracker::setOutput()
{
  Matrix<double> voltage = Matrix<double>::zeros()
                                .add(linearFFW())
                                .add(rotationalFFW())
                                .add(linearCorrection())
                                .add(rotationalCorrection());

  gMotor.forEachWith<double>(
    voltage,
    [] (Motor &motor, double &voltage) -> void {
      motor.voltage(voltage);
    }
  );
}

void Tracker::transition()
{
  Orientation *orientation = Orientation::getInstance();

  TimeUnit final_time = profile_.finalTime();
  LinearRotationalPoint final_point = profile_.pointAtTime(final_time);
  AngleUnit final_heading = final_point.rotational_point.displacement;

  orientation->incrementHeading(final_heading.degrees());

  Matrix<double> voltage = Matrix<double>::zeros()
                                .add(linearFFW(final_point))
                                .add(rotationalFFW(final_point));

  gMotor.forEachWith<double>(
    voltage,
    [] (Motor &motor, double &voltage) -> void {
      motor.voltage(voltage);
    }
  );

  gEncoder.forEach(
    [] (Encoder &encoder) -> void {
      encoder.displacement(LengthUnit::zero());
    }
  );
}

Matrix<double> Tracker::linearFFW()
{
  return linearFFW(point_);
}

Matrix<double> Tracker::rotationalFFW()
{
  return rotationalFFW(point_);
}

Matrix<double> Tracker::linearFFW(LinearRotationalPoint linrot_point)
{
  LinearPoint point = linrot_point.linear_point;

  double acceleration = point.acceleration.meters();
  double velocity = point.velocity.meters();

  double output = linear_ffw_.output(acceleration, velocity);

  return Matrix<double>::ones().multiply(output);
}

Matrix<double> Tracker::rotationalFFW(LinearRotationalPoint linrot_point)
{
  RotationalPoint point = linrot_point.rotational_point;

  double acceleration = point.acceleration.radians();
  double velocity = point.velocity.radians();

  double output = rotational_ffw_.output(acceleration, velocity);

  return Matrix<double>::ones().multiply(output).oppose();
}

Matrix<double> Tracker::linearCorrection()
{
  Matrix<double> current = gEncoder.map<double>(
    [] (Encoder &encoder) -> double {
      return encoder.displacement().meters();
    }
  ).straighten();

  Matrix<double> setpoint = Matrix<double>::ones().multiply(
    point_.linear_point.displacement.meters()
  );

  return encoder_pid_.mapWith<double, double, double>(
    current,
    setpoint,
    [] (PIDFunction &pid, double &current, double &setpoint) -> double {
      return pid.response(current, setpoint);
    }
  );
}

Matrix<double> Tracker::rotationalCorrection()
{
  Orientation *orientation = Orientation::getInstance();

  orientation->update();

  double current = - orientation->getHeading();
  double setpoint = point_.rotational_point.displacement.degrees();

  double response = gyro_pid_.response(current, setpoint);

  return Matrix<double>::ones().multiply(response).oppose();
}

}

#include "units.h"

namespace Motion {

LengthUnit LengthUnit::zero()
{
  return LengthUnit(0.0);
}

LengthUnit LengthUnit::fromAbstract(double value)
{
  return fromMeters(value);
}

LengthUnit LengthUnit::fromMeters(double value)
{
  return LengthUnit(value);
}

LengthUnit LengthUnit::fromCells(double value)
{
  return LengthUnit(kMetersPerCell * value);
}

LengthUnit LengthUnit::fromCounts(double value)
{
  return LengthUnit(kMetersPerCount * value);
}

double LengthUnit::abstract() const
{
  return meters();
}

double LengthUnit::meters() const
{
  return meters_;
}

double LengthUnit::cells() const
{
  return meters_ / kMetersPerCell;
}

double LengthUnit::counts() const
{
  return meters_ / kMetersPerCount;
}

LengthUnit::LengthUnit(double meters) : meters_(meters)
{}

AngleUnit AngleUnit::zero()
{
  return AngleUnit(0.0);
}

AngleUnit AngleUnit::fromAbstract(double value)
{
  return fromDegrees(value);
}

AngleUnit AngleUnit::fromDegrees(double value)
{
  return AngleUnit(value);
}

AngleUnit AngleUnit::fromRadians(double value)
{
  return AngleUnit(kDegreesPerRadian * value);
}

AngleUnit AngleUnit::fromRotations(double value)
{
  return AngleUnit(kDegreesPerRotation * value);
}

double AngleUnit::abstract() const
{
  return degrees();
}

double AngleUnit::degrees() const
{
  return degrees_;
}

double AngleUnit::radians() const
{
  return degrees_ / kDegreesPerRadian;
}

double AngleUnit::rotations() const
{
  return degrees_ / kDegreesPerRotation;
}

AngleUnit::AngleUnit(double degrees) : degrees_(degrees)
{}

TimeUnit TimeUnit::zero()
{
  return TimeUnit(0.0);
}

TimeUnit TimeUnit::fromAbstract(double value)
{
  return fromSeconds(value);
}

TimeUnit TimeUnit::fromSeconds(double value)
{
  return TimeUnit(value);
}

double TimeUnit::abstract() const
{
  return seconds();
}

double TimeUnit::seconds() const
{
  return seconds_;
}

TimeUnit::TimeUnit(double seconds) : seconds_(seconds)
{}

}

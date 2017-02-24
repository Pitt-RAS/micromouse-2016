#ifndef UNITS_H
#define UNITS_H

// disable Arduino macros
#undef degrees
#undef radians

#include <cmath>

namespace Motion {

// displacement reads as is
// velocity is per second
// acceleration is per second squared

class LengthUnit
{
  public:
    static LengthUnit zero();

    static LengthUnit fromAbstract(double value);

    static LengthUnit fromMeters(double value);
    static LengthUnit fromCells(double value);
    static LengthUnit fromCounts(double value);

    double abstract();

    double meters();
    double cells();
    double counts();

  private:
    static constexpr double kMetersPerCell = 0.18;
    static constexpr double kMetersPerCount = 0.000653868;

    LengthUnit(double meters);

    double meters_;
};

// need to define which direction is positive

class AngleUnit
{
  public:
    static AngleUnit zero();

    static AngleUnit fromAbstract(double value);

    static AngleUnit fromDegrees(double value);
    static AngleUnit fromRadians(double value);
    static AngleUnit fromRotations(double value);

    double abstract();

    double degrees();
    double radians();
    double rotations();

  private:
    static constexpr double kDegreesPerRadian = 180.0 / M_PI;
    static constexpr double kDegreesPerRotation = 360.0;

    AngleUnit(double degrees);

    double degrees_;
};

class TimeUnit
{
  public:
    static TimeUnit zero();
    static TimeUnit fromAbstract(double value);
    static TimeUnit fromSeconds(double value);

    double abstract();
    double seconds();

  private:
    TimeUnit(double seconds);

    double seconds_;
};

}

#endif

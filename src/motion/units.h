#ifndef UNITS_H
#define UNITS_H

namespace Motion {

// displacement reads as is
// velocity is per second
// acceleration is per second squared

class LengthUnit
{
  public:
    static LengthUnit fromMeters(double value);
    static LengthUnit fromCells(double value);

    double meters();
    double cells();

  private:
    double cells_;
};

// need to define which direction is positive

class AngleUnit
{
  public:
    static AngleUnit fromDegrees(double value);
    static AngleUnit fromRadians(double value);

    double degrees();
    double radians();

  private:
    double radians_;
};

class TimeUnit
{
  public:
    static TimeUnit fromSeconds(double value);

    double seconds();

  private:
    double seconds_;
};

}

#endif

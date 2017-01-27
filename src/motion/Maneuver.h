#ifndef MANEUVER_H
#define MANEUVER_H

#include "units.h"

namespace Motion {

struct ManeuverConstraints
{
  // consider adding error checking to these numbers via setters/getters

  LengthUnit maxForwardVelocity;
  LengthUnit maxDiagonalVelocity;

  LengthUnit sweepVelocity;

  LengthUnit maxAcceleration;
  LengthUnit maxDeceleration;
};

struct Transition
{
  LengthUnit velocity;
  const AngleUnit rotational_velocity = AngleUnit::fromRadians(0);
};

class Maneuver
{
  public:
    // should be protected by a lock, to be discussed
    static ManeuverConstraints constraints;

    virtual void run() = 0;

  protected:
    void transition(Transition transition);
    Transition transition();

  private:
    static Transition transition_;
};

class ForwardManeuver : public Maneuver
{
  public:
    ForwardManeuver(LengthUnit length);

    virtual void run();
};

class PivotManeuver : public Maneuver
{
  public:
    struct Angle {
      enum { k45, k90, k180 } magnitude;
      enum { left, right } direction;
    };

    PivotManeuver(Angle angle);

    // for the possibility that we need to pivot for small corrections
    PivotManeuver(AngleUnit angle);

    virtual void run();
};

class SweepManeuver : public Maneuver
{
  public:
    struct Angle {
      enum { k45, k90, k135, k180 } magnitude;
      enum { left, right } direction;
    };

    SweepManeuver(Angle angle);

    virtual void run();
};

}

#endif

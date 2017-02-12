#ifndef MANEUVER_H
#define MANEUVER_H

#include "../units.h"

namespace Motion {

struct ManeuverConstraints
{
  // consider adding error checking to these numbers via setters/getters

  LengthUnit max_forward_velocity = LengthUnit::zero();
  LengthUnit max_diagonal_velocity = LengthUnit::zero();

  LengthUnit sweep_velocity = LengthUnit::zero();

  LengthUnit linear_acceleration = LengthUnit::zero();
  LengthUnit linear_deceleration = LengthUnit::zero();

  AngleUnit max_rotational_velocity = AngleUnit::zero();
  AngleUnit rotational_acceleration = AngleUnit::zero();
  AngleUnit rotational_deceleration = AngleUnit::zero();
};

struct Transition
{
  static const AngleUnit rotational_velocity;

  LengthUnit linear_velocity;
};

class Maneuver
{
  public:
    virtual void run() = 0;

  protected:
    // will have public mechanism for setting constraints and obtaining lock
    static ManeuverConstraints constraints();
    static void constraints(ManeuverConstraints constraints);

    static Transition transition();
    static void transition(Transition transition);

  private:
    static ManeuverConstraints constraints_;
    static Transition transition_;
};

}

#endif

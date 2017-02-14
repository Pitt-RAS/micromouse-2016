#include "Maneuver.h"

namespace Motion {

const AngleUnit Transition::rotational_velocity = AngleUnit::zero();

ManeuverConstraints Maneuver::constraints()
{
  return constraints_;
}

ManeuverConstraints Maneuver::constraints_ = ManeuverConstraints();
Transition Maneuver::transition_ = { LengthUnit::zero() };

void Maneuver::constraints(ManeuverConstraints constraints)
{
  constraints_ = constraints;
}

Transition Maneuver::transition()
{
  return transition_;
}

void Maneuver::transition(Transition transition)
{
  transition_ = transition;
}

}

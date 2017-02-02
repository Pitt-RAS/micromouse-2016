#include "TrapezoidalProfile.h"
#include "drive.h"
#include "Maneuver.h"

namespace Motion {

const AngleUnit Transition::rotational_velocity = AngleUnit::zero();

void Maneuver::transition(Transition transition)
{
  transition_ = transition;
}

Transition Maneuver::transition()
{
  return transition_;
}

}

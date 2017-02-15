#include "../../user_interaction/FreakOut.h"
#include "Maneuver.h"

namespace Motion {

const AngleUnit Transition::rotational_velocity = AngleUnit::zero();

ManeuverLock::ManeuverLock(ManeuverConstraints &constraints)
{
  lock();
  Maneuver::constraints(constraints);
}

ManeuverLock::~ManeuverLock()
{
  unlock();
}

void ManeuverLock::unlock()
{
  if (!Maneuver::locked_) freakOut("MUNL"); // consider more verbose logging
  Maneuver::locked_ = false;
}

void ManeuverLock::lock()
{
  if (Maneuver::locked_) freakOut("MLCK"); // consider more verbose logging
  Maneuver::locked_ = true;
}

ManeuverConstraints Maneuver::constraints_ = ManeuverConstraints();
Transition Maneuver::transition_ = { LengthUnit::zero() };
bool Maneuver::locked_ = false;

ManeuverConstraints Maneuver::constraints()
{
  if (!locked_) freakOut("MACC"); // consider more verbose logging
  return constraints_;
}

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

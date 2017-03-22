#include "../../legacy_motion/SweptTurnProfile.h"
#include "../../legacy_motion/motion.h"
#include "../../user_interaction/FreakOut.h"
#include "LegacySweep.h"

namespace Motion {

LegacySweep::LegacySweep(AngleUnit angle, double size_scaling) :
  angle_(angle), size_scaling_(size_scaling)
{}

void LegacySweep::run()
{
  if (transition().linear_velocity.abstract()
                                    != constraints().sweep_velocity.abstract())
    freakOut("LSWP"); // consider more verbose logging

  SweptTurnType type;

  switch ((int) angle_.degrees()) {
    default:
    case   45: type = kLeftTurn45;   break;
    case   90: type = kLeftTurn90;   break;
    case  135: type = kLeftTurn135;  break;
    case  180: type = kLeftTurn180;  break;
    case  -45: type = kRightTurn45;  break;
    case  -90: type = kRightTurn90;  break;
    case -135: type = kRightTurn135; break;
    case -180: type = kRightTurn180; break;
  }

  motion_corner(type, constraints().sweep_velocity.meters(), size_scaling_);

  transition({ constraints().sweep_velocity });
}

}

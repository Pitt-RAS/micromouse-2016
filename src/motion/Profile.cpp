#include "Profile.h"

namespace Motion {

LinearRotationalPoint LinearRotationalPoint::zero()
{
  return { LinearPoint::zero(), RotationalPoint::zero() };
}

}

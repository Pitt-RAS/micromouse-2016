#include "Profile.h"

namespace Motion {

template <typename UnitType>
ProfilePoint<UnitType> ProfilePoint<UnitType>::zero()
{
  return { UnitType::zero(), UnitType::zero(), UnitType::zero() };
}

LinearRotationalPoint LinearRotationalPoint::zero()
{
  return { LinearPoint::zero(), RotationalPoint::zero() };
}

}

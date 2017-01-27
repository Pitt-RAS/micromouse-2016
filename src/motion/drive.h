#ifndef DRIVE_H
#define DRIVE_H

#include "Profile.h"

namespace Motion {

struct DriveOptions
{
  enum { kStraight, kDiagonal, kSweep, kPivot } tuning = kStraight;
  bool use_range = true;
  bool use_gyro = true;
};

void drive(DriveOptions options, LinearRotationalProfile profile);

}

#endif

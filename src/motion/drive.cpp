#include <Arduino.h>
#include "../device/Encoder.h"
#include "../device/Motor.h"
#include "SkidSteerCar.h"
#include "Wheel.h"
#include "drive.h"

namespace Motion {

static LengthUnit kRightWheelX = LengthUnit::fromMeters(0.03725);

static LengthUnit kLeftWheelX =
                          LengthUnit::fromAbstract(- kRightWheelX.abstract());

static WheelOnBody gWheelsOnBody[] = {
  {
    Wheel(motor_lf, gEncoderLF),
    PointOnBody(kLeftWheelX, LengthUnit::zero())
  },
  {
    Wheel(motor_lb, gEncoderLB),
    PointOnBody(kLeftWheelX, LengthUnit::zero())
  },
  {
    Wheel(motor_rf, gEncoderRF),
    PointOnBody(kRightWheelX, LengthUnit::zero())
  },
  {
    Wheel(motor_rb, gEncoderRB),
    PointOnBody(kRightWheelX, LengthUnit::zero())
  }
};

static SkidSteerCar<4> gCar(gWheelsOnBody);

void drive(DriveOptions &options, LinearRotationalProfile &profile)
{
  // set tuning parameters

  elapsedMicros timer;

  TimeUnit time = TimeUnit::fromSeconds(timer / 1e6);

  while (time.abstract() < profile.finalTime().abstract()) {
    time = TimeUnit::fromSeconds(timer / 1e6);

    LinearRotationalPoint point = profile.pointAtTime(time);

    if (options.use_range) {
      // bake in range
    }

    if (options.use_gyro) {
      // bake in gyro
    }

    gCar.reference(point);
    gCar.update(time);
  }
}

}

#include <Arduino.h>
#include "../../user_interaction/FreakOut.h"
#include "../Profile.h"
#include "../tracker/Tracker.h"
#include "Plant.h"

namespace Motion {

namespace {

  class LocalProfile : public LinearRotationalProfile
  {
    public:
      virtual LinearRotationalPoint pointAtTime(TimeUnit time);
      virtual TimeUnit finalTime();

    private:
      static const TimeUnit kFinalTime;
  };

  const TimeUnit LocalProfile::kFinalTime = TimeUnit::fromSeconds(0.0);

  LinearRotationalPoint LocalProfile::pointAtTime(TimeUnit time)
  {
    return LinearRotationalPoint::zero();
  }

  TimeUnit LocalProfile::finalTime()
  {
    return kFinalTime;
  }

}

void Plant::run()
{
  if (transition().linear_velocity.abstract() != LengthUnit::zero().abstract())
    freakOut("PLTL"); // consider more verbose logging

  if (transition().rotational_velocity.abstract()
                                              != AngleUnit::zero().abstract())
    freakOut("PLTR"); // consider more verbose logging

  TrackerOptions options;

  options.encoder_pid_parameters = { 0.0, 0.0, 0.0 };
  options.gyro_pid_parameters = { 0.0, 0.0, 0.0 };

  options.end_plant = false;

  LocalProfile profile;

  transition({ LengthUnit::zero() });

  Tracker(options, profile).run();
}

}

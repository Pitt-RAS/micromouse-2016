#include "SweptTurnProfile.h"

#include <cmath>
#include <cstddef>

// Dependencies within Micromouse
#include "conf.h"

static float calculateAccelerationDuration(float turn_angle,
                                           float inside_trigs,
                                           float omega0)
{
  // angle covered if we accelerate to max rotational velocity
  float angle_for_full_acceleration = omega0 / inside_trigs;

  if (angle_for_full_acceleration > turn_angle / 2) {
    // We don't have time to accelerate to max rotational velocity

    // Solve for t when theta = turn_angle / 2
    float temp = (turn_angle / 2) * inside_trigs / omega0;
    return std::acos(1 - temp) / inside_trigs;
  } else {
    // We have time to accelerate to max rotational velocity

    // Solve for t when omega maxes out
    return (M_PI / 2) / inside_trigs;
  }
}

static float calculateConstVelocityDuration(float turn_angle,
                                            float inside_trigs,
                                            float omega0)
{
  // angle covered if we accelerate to max rotational velocity
  float angle_for_full_acceleration = omega0 / inside_trigs;

  if (angle_for_full_acceleration > turn_angle / 2) {
    // We don't have time to accelerate to max rotational velocity
    return 0;
  } else {
    // We have time to accelerate to max rotational velocity
    float theta_const = turn_angle - 2 * omega0 / inside_trigs;
    return theta_const / omega0;
  }
}

SweptTurnProfile::SweptTurnProfile(float tangential_velocity,
                                   float turn_angle_deg,
                                   float time_step)
    : tangential_velocity_(tangential_velocity),
      turn_angle_(turn_angle_deg * M_PI / 180),
      time_step_(time_step),
      inside_trigs_(ROBOT_MASS * (MM_BETWEEN_WHEELS / 2.0 / 1000.0)
                    * tangential_velocity_ / MOMENT_OF_INERTIA),
      omega0_(ROBOT_MASS * MAX_COEFFICIENT_FRICTION * 9.81 / ROBOT_MASS
              / tangential_velocity),
      acceleration_duration_(calculateAccelerationDuration(
          turn_angle_, inside_trigs_, omega0_)),
      const_velocity_duration_(calculateConstVelocityDuration(
          turn_angle_, inside_trigs_, omega0_)),
      turn_duration_(2 * acceleration_duration_ + const_velocity_duration_)
{
  for (size_t i = 0; i <= getTotalTurnSteps(); i++) {
    offset_table_[i] = getTurnOffset(getAngleAtTime(i * time_step));
  }
}

float SweptTurnProfile::getOffsetAtMicros(unsigned long t) const
{
  float fractional_index = (t / 1000000.0) / time_step_;
  int index = fractional_index;
  float low_offset = offset_table_[index];
  float high_offset = offset_table_[index + 1];
  return (fractional_index - index) * (high_offset - low_offset) + low_offset;
}

size_t SweptTurnProfile::getTotalTurnSteps() const
{
  return (size_t)((turn_duration_) / time_step_ + 1);
}

float SweptTurnProfile::getAngleAtTime(float t) const
{
  if (t <= acceleration_duration_) {
    return (1 - std::cos(inside_trigs_ * t)) * omega0_ / inside_trigs_;
  } else if (t <= acceleration_duration_ + const_velocity_duration_) {
    return getAngleAtTime(acceleration_duration_)
         + getVelocityAtTime(acceleration_duration_) * t;
  } else {
    return turn_angle_ - getAngleAtTime(turn_duration_ - t);
  }
}

float SweptTurnProfile::getVelocityAtTime(float t) const
{
  return omega0_ * std::sin(inside_trigs_ * t);
}

float SweptTurnProfile::getTurnOffset(float angle) const
{
  return (MM_BETWEEN_WHEELS / 2.0) * angle;
}

unsigned long SweptTurnProfile::getTotalTime() const
{
  return turn_duration_ * 1000000;
}

float SweptTurnProfile::getTotalAngle() const
{
  return turn_angle_ * 180 / M_PI;
}

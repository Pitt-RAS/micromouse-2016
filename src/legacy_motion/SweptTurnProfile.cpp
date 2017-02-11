#include <cmath>
#include <cstddef>
#include "../conf.h"

#include "FastTrigs.h"
#include "SweptTurnProfile.h"

// Internal methods for calculating const parameters
static float calculateAccelerationDuration(float turn_angle,
                                           float inside_trigs,
                                           float omega_max)
{
  // angle covered if we accelerate to max rotational velocity
  float angle_for_full_acceleration = omega_max / inside_trigs;

  if (angle_for_full_acceleration > turn_angle / 2) {
    // We don't have time to accelerate to max rotational velocity

    // Solve for t when theta = turn_angle / 2
    float temp = (turn_angle / 2) * inside_trigs / omega_max;
    return std::acos(1 - temp) / inside_trigs;
  } else {
    // We have time to accelerate to max rotational velocity

    // Solve for t when omega maxes out
    return (M_PI / 2) / inside_trigs;
  }
}

static float calculateConstVelocityDuration(float turn_angle,
                                            float inside_trigs,
                                            float omega_max)
{
  // angle covered if we accelerate to max rotational velocity
  float angle_for_full_acceleration = omega_max / inside_trigs;

  if (angle_for_full_acceleration > turn_angle / 2) {
    // We don't have time to accelerate to max rotational velocity
    return 0;
  } else {
    // We have time to accelerate to max rotational velocity
    float theta_const = turn_angle - 2 * omega_max / inside_trigs;
    return theta_const / omega_max;
  }
}

// Constructor
SweptTurnProfile::SweptTurnProfile(float tangential_velocity,
                                   float turn_angle_deg)
    : tangential_velocity_(tangential_velocity),
      turn_angle_(turn_angle_deg * M_PI / 180),
      inside_trigs_(ROBOT_MASS * (MM_BETWEEN_WHEELS / 2.0 / 1000.0)
                    * tangential_velocity_ / MOMENT_OF_INERTIA),
      omega_max_(ROBOT_MASS * MAX_COEFFICIENT_FRICTION * 9.81 / ROBOT_MASS
              / tangential_velocity),
      acceleration_duration_(calculateAccelerationDuration(
          turn_angle_, inside_trigs_, omega_max_)),
      const_velocity_duration_(calculateConstVelocityDuration(
          turn_angle_, inside_trigs_, omega_max_)),
      turn_duration_(2 * acceleration_duration_ + const_velocity_duration_)
{
}

// Public methods
float SweptTurnProfile::getAngle(float t) const
{
  if (t <= acceleration_duration_) {
    return (1 - FastTrigs::cos(inside_trigs_ * t)) * omega_max_ / inside_trigs_;
  } else if (t <= acceleration_duration_ + const_velocity_duration_) {
    return getAngle(acceleration_duration_)
         + (getAngularVelocity(acceleration_duration_)
            * (t - acceleration_duration_));
  } else {
    return turn_angle_ - getAngle(turn_duration_ - t);
  }
}

float SweptTurnProfile::getAngularAcceleration(float t) const
{
  if (t <= acceleration_duration_) {
    return omega_max_ * inside_trigs_ * FastTrigs::cos(inside_trigs_ * t);
  } else if (t <= acceleration_duration_ + const_velocity_duration_) {
    return 0;
  } else {
    return -getAngularAcceleration(turn_duration_ - t);
  }
}

float SweptTurnProfile::getAngularVelocity(float t) const
{
  if (t <= acceleration_duration_) {
    return omega_max_ * FastTrigs::sin(inside_trigs_ * t);
  } else if (t <= acceleration_duration_ + const_velocity_duration_) {
    return getAngularVelocity(acceleration_duration_);
  } else {
    return getAngularVelocity(turn_duration_ - t);
  }
}

float SweptTurnProfile::getTotalAngle() const
{
  return turn_angle_ * (180 / M_PI);
}

float SweptTurnProfile::getTotalTime() const
{
  return turn_duration_;
}

#ifndef MANEUVER_H
#define MANEUVER_H

#include "../legacy_motion/SweptTurnProfile.h"
#include "TrapezoidalProfile.h"
#include "units.h"

namespace Motion {

struct ManeuverConstraints
{
  // consider adding error checking to these numbers via setters/getters

  LengthUnit max_forward_velocity = LengthUnit::zero();
  LengthUnit max_diagonal_velocity = LengthUnit::zero();

  LengthUnit sweep_velocity = LengthUnit::zero();

  LengthUnit linear_acceleration = LengthUnit::zero();
  LengthUnit linear_deceleration = LengthUnit::zero();

  AngleUnit max_rotational_velocity = AngleUnit::zero();
  AngleUnit rotational_acceleration = AngleUnit::zero();
  AngleUnit rotational_deceleration = AngleUnit::zero();
};

struct Transition
{
  static const AngleUnit rotational_velocity;

  LengthUnit linear_velocity;
};

class Maneuver
{
  public:
    virtual void run() = 0;

  protected:
    // will have public mechanism for setting constraints and obtaining lock
    static ManeuverConstraints constraints();
    static void constraints(ManeuverConstraints constraints);

    static Transition transition();
    static void transition(Transition transition);

  private:
    static ManeuverConstraints constraints_;
    static Transition transition_;
};

class Straight : public Maneuver
{
  public:
    Straight(LengthUnit length);

    virtual void run();

  protected:
    Straight(LengthUnit length, bool zero_final_velocity);

  private:
    class Profile : public LinearRotationalProfile
    {
      public:
        Profile(TrapezoidalProfile<LengthUnit> linear_component);

        virtual LinearRotationalPoint pointAtTime(TimeUnit time);

      private:
        TrapezoidalProfile<LengthUnit> linear_component_;
    };

    const LengthUnit length_;
    const bool zero_final_velocity_;
};

class Start : public Straight
{
  public:
    Start();
};

class Stop : public Straight
{
  public:
    Stop();
};

class Pivot : public Maneuver
{
  public:
    struct Angle {
      enum { k45, k90, k180 } magnitude;
      enum { left, right } direction;
    };

    Pivot(Angle angle);

    Pivot(AngleUnit angle);

    virtual void run();

  private:
    class Profile : public LinearRotationalProfile
    {
      public:
        Profile(TrapezoidalProfile<AngleUnit> rotational_component);

        virtual LinearRotationalPoint pointAtTime(TimeUnit time);

      private:
        TrapezoidalProfile<AngleUnit> rotational_component_;
    };

    AngleUnit toContinuousAngle(Angle discrete_angle);

    const AngleUnit angle_;
};

class Sweep : public Maneuver
{
  public:
    struct Angle {
      enum { k45, k90, k135, k180 } magnitude;
      enum { left, right } direction;
    };

    Sweep(Angle angle);

    virtual void run();

  private:
    class Profile : public LinearRotationalProfile
    {
      public:
        Profile(Angle angle, LengthUnit velocity);

        virtual LinearRotationalPoint pointAtTime(TimeUnit time);

      private:
        static const SweptTurnProfile kLegacyProfile45;
        static const SweptTurnProfile kLegacyProfile90;
        static const SweptTurnProfile kLegacyProfile135;
        static const SweptTurnProfile kLegacyProfile180;

        const SweptTurnProfile &legacy_implementation_;

        LengthUnit radius_ = LengthUnit::fromCells(0.5); // is this right?
        LengthUnit velocity_;

        const SweptTurnProfile &toLegacyImplementation(Angle angle);
    };

    Angle angle_;
};

}

#endif

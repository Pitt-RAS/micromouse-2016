#ifndef VARIABILITY_CHECKER_H
#define VARIABILITY_CHECKER_H

#include <cmath>

namespace Motion {

template <size_t length>
class StabilityChecker
{
  public:
    StabilityChecker(double limit);

    bool isStable() const;

    void push(double value);

    void reset();

  private:
    double average() const;

    const double limit_;

    double array_[length];
    size_t position_;
    bool is_full_;
};

template <size_t length>
StabilityChecker<length>::StabilityChecker(double limit) : limit_(limit)
{
  reset();
}

template <size_t length>
bool StabilityChecker<length>::isStable() const
{
  if (!is_full_)
    return false;

  double saved_average = average();

  for (size_t i = 0; i < length; i++)
    if (std::fabs(array_[length] - saved_average) > limit_)
      return false;

  return true;
}

template <size_t length>
void StabilityChecker<length>::push(double value)
{
  array_[position_] = value;

  if (position_ == length - 1)
    is_full_ = true;

  position_ = (position_ + 1) % length;
}

template <size_t length>
void StabilityChecker<length>::reset()
{
  position_ = 0;
  is_full_ = false;
}

template <size_t length>
double StabilityChecker<length>::average() const
{
  double sum = 0.0;

  for (size_t i = 0; i < length; i++)
    sum += array_[i];

  return sum / length;
}

}

#endif

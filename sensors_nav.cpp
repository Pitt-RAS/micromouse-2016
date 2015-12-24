#include "sensors_nav.h"
#include "conf.h"
#include "sensors_range.h"

bool nav_front_wall()
{
  RangeSensor::read(RANGE_FRONT);
}

bool nav_left_wall()
{
  RangeSensor::read(RANGE_LEFT);
}

bool nav_right_wall()
{
  RangeSensor::read(RANGE_RIGHT);
}

bool nav_nextLeft_wall()
{
  RangeSensor::read(RANGE_FRONT_LEFT);
}

bool nav_nextRight_wall()
{
  RangeSensor::read(RANGE_FRONT_RIGHT);
}

enum direction nav_direction()
{
}

#include "driver.h"

#ifdef COMPILE_FOR_PC
#include <iostream>
#include <unistd.h>
#endif

#ifndef COMPILE_FOR_PC
#include <Arduino.h>
#include "data.h"
#include "motion.h"
#include "conf.h"
#include "FreakOut.h"
#include "RangeSensorContainer.h"
#include "Menu.h"
#endif




float Driver::getXFloat()
{
  return x_;
}

float Driver::getYFloat()
{
  return y_;
}

void Driver::setX(float x)
{
  if (x < - 0.5 || x > x_size_ - 0.5)
    return;

  x_ = x;
}

void Driver::setY(float y)
{
  if (y < - 0.5 || y > y_size_ - 0.5)
    return;

  y_ = y;
}

Driver::Driver() :
  kXSize(16), kYSize(16), kInitialXPosition(0.0), kInitialYPosition(0.0)
{
  x_size_ = kXSize;
  y_size_ = kYSize;

  setX(kInitialXPosition);
  setY(kInitialYPosition);
}

int Driver::getX()
{
  return (int) (getXFloat() + 0.5);
}

int Driver::getY()
{
  return (int) (getYFloat() + 0.5);
}

void Driver::move(Path<16, 16> path)
{
  Compass8 movement_direction, next_direction;
  int movement_distance;

  next_direction = path.nextDirection();

  if (path.isEmpty())
    move(next_direction, 1);

  while (!path.isEmpty())
  {
    movement_direction = next_direction;
    movement_distance = 0;

    while (next_direction == movement_direction) {
      movement_distance++;

      if (path.isEmpty())
        break;

      next_direction = path.nextDirection();
    }

    move(movement_direction, movement_distance);
  }
}




Compass8 Turnable::getDir()
{
  return (Compass8) ( (((int) dir_ + 45 / 2) % 360) / 45 );
}

float Turnable::getDirF()
{
  return dir_;
}

void Turnable::setDir(Compass8 dir)
{
  dir_ = 45 * (int) dir;
}

void Turnable::setDir(float dir)
{
  dir = (int) dir % 360 + dir - (int) dir;

  if (dir < 0.0)
    dir += 360.0;

  dir_ = dir;
}

Compass8 Turnable::relativeDir(Compass8 absolute_dir)
{
  int arc;

  arc = (int) absolute_dir - (int) getDir();

  if (arc < 0)
    arc += 8;

  return (Compass8) arc;
}

float Turnable::relativeDirF(float absolute_dir)
{
  float arc;

  absolute_dir = (int) absolute_dir % 360 + absolute_dir - (int) absolute_dir;

  if (absolute_dir < 0.0)
    absolute_dir += 360.0;

  arc = absolute_dir - getDirF();

  if (arc < 0.0)
    arc += 360.0;

  return arc;
}

Turnable::Turnable() : kInitialDirection(0.0)
{
  dir_ = kInitialDirection;
}




void SimulationDriver::turn(Compass8 dir)
{
  setDir(dir);

  sleep();
  update();
}

int SimulationDriver::getSleepTime()
{
  return sleep_time_;
}

SimulationDriver::SimulationDriver()
{
  sleep_time_ = kDefaultSleepTime;

#ifdef COMPILE_FOR_PC
  real_maze_.loadFile("real.maze");
#endif
}

void SimulationDriver::setSleepTime(int time)
{
  if (time <= 0)
    return;

  sleep_time_ = time;
}

bool SimulationDriver::isWall(Compass8 dir)
{
  return real_maze_.isWall(getX(), getY(), dir);
}

void SimulationDriver::move(Compass8 dir, int distance)
{
  int i;

  if (distance < 0)
    return;

  turn(dir);

  for (i = 0; i < distance; i++) {
    switch (dir) {
      case kNorth:
        setY(getYFloat() + 1.0);
        break;

      case kNorthEast:
        setX(getXFloat() + 1.0);
        setY(getYFloat() + 1.0);
        break;

      case kEast:
        setX(getXFloat() + 1.0);
        break;

      case kSouthEast:
        setX(getXFloat() + 1.0);
        setY(getYFloat() - 1.0);
        break;

      case kSouth:
        setY(getYFloat() - 1.0);
        break;

      case kSouthWest:
        setX(getXFloat() - 1.0);
        setY(getYFloat() - 1.0);
        break;

      case kWest:
        setX(getXFloat() - 1.0);
        break;

      case kNorthWest:
        setX(getXFloat() - 1.0);
        setY(getYFloat() + 1.0);
        break;

      default:
        setX(7.0);
        setY(7.0);
        break;
    }

    sleep();
    update();
  }
}




#ifdef COMPILE_FOR_PC




void StdoutDriver::update()
{
  std::cout << "Robot position " << getX() << " " << getY() << " "
      << 45 * getDir() << std::endl;
}

void StdoutDriver::sleep()
{
  usleep(1000 * getSleepTime());
}

StdoutDriver::StdoutDriver()
{
  update();
}




void CanvasDriver::init()
{
  size_t x, y;
  int i;
  std::string strings[4] = {"north", "east", "south", "west"};

  for (x = 0; x < real_maze_.getXSize(); x++)
  for (y = 0; y < real_maze_.getYSize(); y++)
  for (i = 0; i < 4; i++) {
    if (real_maze_.isWall(x, y, (Compass8) (2 * i))) {
      file_ << "wall " << x << " " << y << " " 
          << strings[i] << std::endl;
    }
  }
}

void CanvasDriver::update()
{
  file_ << "robot " << getX() << " " << getY() << " "
      << 45 * getDir() << std::endl;
}

void CanvasDriver::sleep()
{
  file_ << "sleep " << getSleepTime() << std::endl;
}

CanvasDriver::CanvasDriver()
{
  file_.open("commands");

  init();
  update();
}

CanvasDriver::~CanvasDriver()
{
  file_.close();
}




#endif // #ifdef COMPILE_FOR_PC




#ifndef COMPILE_FOR_PC




void SerialDriver::update()
{
  Serial.print("Robot position ");
  Serial.print(getX());
  Serial.print(" ");
  Serial.print(getY());
  Serial.print(" ");
  Serial.print(getDir());
  Serial.println();
}

void SerialDriver::sleep()
{
  delay(getSleepTime());
}

SerialDriver::SerialDriver()
{
  update();
}




bool RobotDriver::onEdge()
{
  float x_offset, y_offset;

  x_offset = getXFloat() - getX();
  y_offset = getYFloat() - getX();

  if (x_offset < -0.25 || x_offset > 0.25)
    return true;
  if (y_offset < -0.25 || y_offset > 0.25)
    return true;

  return false;
}

RobotDriver::RobotDriver() :
    exit_velocity_(0.0), last_direction_(kNorth), turn_advanced_(false)
{
  // Initialize whatever we need to.
}

int RobotDriver::getX()
{
  float x;

  x = getXFloat();

  if ((x + 0.5) - (int) (x + 0.5) == 0.0) {
    switch(getDir()) {
      case kEast:
        return (int) (x + 0.5);
        break;
      case kWest:
        return (int) (x - 0.5);
        break;
      default:
        break;
    }
  }

  return Driver::getX();
}

int RobotDriver::getY()
{
  float y;

  y = getYFloat();

  if ((y + 0.5) - (int) (y + 0.5) == 0.0) {
    switch(getDir()) {
      case kNorth:
        return (int) (y + 0.5);
        break;
      case kSouth:
        return (int) (y - 0.5);
        break;
      default:
        break;
    }
  }

  return Driver::getY();
}

void RobotDriver::turn(Compass8 dir)
{
  float arc_to_turn;
  float distance_to_add;

  arc_to_turn = 45.0 * (int) relativeDir(dir);

  if (arc_to_turn > 180.0)
    arc_to_turn -= 360.0;

  turn_advanced_ = false;
  distance_to_add = 0.0;

  if (onEdge() && exit_velocity_ > 0.0) {
    switch(relativeDir(dir)) {
      case kNorth:
        break;
      case kSouth:
        motion_forward(MM_PER_BLOCK / 2, 0.0);
        motion_rotate(180.0);
        motion_forward(MM_PER_BLOCK / 2, exit_velocity_);
        turn_advanced_ = true;
        distance_to_add = 1.0;
        break;
      case kEast:
        motion_corner(kRightTurn90, exit_velocity_);
        turn_advanced_ = true;
        distance_to_add = 1.0;
        break;
      case kWest:
        motion_corner(kLeftTurn90, exit_velocity_);
        turn_advanced_ = true;
        distance_to_add = 1.0;
        break;
      default:
        motion_forward(MM_PER_BLOCK / 4, 0.0);
        motion_hold(100);
        freakOut("BAD4");
        break;
    }
  }
  else if (!onEdge() && exit_velocity_ == 0.0) {
    motion_rotate(arc_to_turn);
    exit_velocity_ = 0.0;
  }
  else {
    motion_forward(MM_PER_BLOCK / 4, 0.0);
    motion_hold(100);
    freakOut("BAD1");
  }

  switch (dir) {
    case kNorth:
      setY(getYFloat() + distance_to_add);
      break;

    case kSouth:
      setY(getYFloat() - distance_to_add);
      break;

    case kEast:
      setX(getXFloat() + distance_to_add);
      break;

    case kWest:
      setX(getXFloat() - distance_to_add);
      break;
  }

  setDir(dir);
}

bool RobotDriver::isWall(Compass8 dir)
{
  RangeSensors.updateReadings();
  if (getX() == 0 && getY() == 0) {
    switch (relativeDir(dir)) {
      case kNorth:
        return RangeSensors.isWall(front);
        break;
      case kSouth:
        return RangeSensors.isWall(back);
        break;
      case kEast:
        return true;
        break;
      case kWest:
        return true;
        break;
      default:
        return true;
        break;
    }
  }

  RangeSensors.updateReadings();

  switch (relativeDir(dir)) {
    case kNorth:
      return RangeSensors.isWall(front);
      break;
    case kSouth:
      return RangeSensors.isWall(back);
      break;
    case kEast:
      return RangeSensors.isWall(right);
      break;
    case kWest:
      return RangeSensors.isWall(left);
      break;
    default:
      return true;
      break;
  }
}

void RobotDriver::move(Compass8 dir, int distance)
{
  float distance_to_add;

  if (distance == 0) {
    if (onEdge() && exit_velocity_ > 0.0) {
      motion_forward(MM_PER_BLOCK / 2, 0.0);
      motion_hold(100);
      exit_velocity_ = 0.0;
    }
    else if (!onEdge() && exit_velocity_ == 0.0) {
      return;
    }
    else {
      motion_forward(MM_PER_BLOCK / 4, 0.0);
      motion_hold(100);
      freakOut("BAD2");
    }

    return;
  }

  turn(dir);

  if (turn_advanced_)
    distance--;

  if (distance < 1)
    return;

  if (onEdge() && exit_velocity_ > 0.0) {
    motion_forward(MM_PER_BLOCK * distance, exit_velocity_);
    distance_to_add = distance;
  }
  else if (!onEdge() && exit_velocity_ == 0.0) {
    motion_forward(MM_PER_BLOCK / 2 + MM_PER_BLOCK * (distance - 1), SEARCH_VELOCITY);
    exit_velocity_ = SEARCH_VELOCITY;
    distance_to_add = distance - 0.5;
  }
  else {
    motion_forward(MM_PER_BLOCK / 4, 0.0);
    motion_hold(100);
    freakOut("BAD3");
  }

  switch (dir) {
    case kNorth:
      setY(getYFloat() + distance_to_add);
      break;

    case kSouth:
      setY(getYFloat() - distance_to_add);
      break;

    case kEast:
      setX(getXFloat() + distance_to_add);
      break;

    case kWest:
      setX(getXFloat() - distance_to_add);
      break;
  }

  last_direction_ = dir;
}




#endif // #ifndef COMPILE_FOR_PC

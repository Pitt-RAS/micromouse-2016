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
#include "RangeSensorContainer.h"
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




RobotDriver::RobotDriver()
{
  // Initialize whatever we need to.
}

void RobotDriver::turn(Compass8 dir)
{
  int arc_to_turn;
  int current;
  int desired;

  current = (int) getDir();
  desired = (int) dir;

  arc_to_turn = desired - current;

  if (arc_to_turn < 0)
    arc_to_turn += 8;

  if (arc_to_turn != 0) {
    if (arc_to_turn <= 4) {
      motion_rotate(45.0 * arc_to_turn);
    }
    else {
      arc_to_turn -= 8;
      motion_rotate(45.0 * arc_to_turn);
    }
  }

  setDir(dir);
}

bool RobotDriver::isWall(Compass8 dir)
{
  int arc_to_turn;
  int current;
  int desired;

  current = (int) getDir();
  desired = (int) dir;

  arc_to_turn = desired - current;

  if (arc_to_turn < 0)
    arc_to_turn += 8;

  if (arc_to_turn != 0) {
    if (arc_to_turn <= 4) {
      arc_to_turn = arc_to_turn;
    }
    else {
      arc_to_turn -= 8;
    }
  }

  RangeSensors.updateReadings();

  switch (arc_to_turn) {
    case 0:
      return RangeSensors.savedIsWall(front);
      break;
    case 2:
      return RangeSensors.savedIsWall(right);
      break;
    case 4:
      return RangeSensors.savedIsWall(back);
      break;
    case -2:
      return RangeSensors.savedIsWall(left);
      break;
    default:
      // This is unacceptable as a long term solution.
      //
      // TODO: Implement a utility method for conversion from absolute
      //       direction to relative direction
      return true;
      break;
  }
}

void RobotDriver::move(Compass8 dir, int distance)
{
  float destination_x, destination_y;
  float distance_to_move;

  turn(dir);

  destination_x = getX();
  destination_y = getY();

  switch (dir) {
    case kNorth:
      destination_y += distance;
      break;

    case kNorthEast:
      destination_x += distance;
      destination_y += distance;
      break;

    case kEast:
      destination_x += distance;
      break;

    case kSouthEast:
      destination_x += distance;
      destination_y -= distance;
      break;

    case kSouth:
      destination_y -= distance;
      break;

    case kSouthWest:
      destination_x -= distance;
      destination_y -= distance;
      break;

    case kWest:
      destination_x -= distance;
      break;

    case kNorthWest:
      destination_x -= distance;
      destination_y += distance;
      break;
  }

  distance_to_move = hypot(destination_x - getXFloat(),
                            destination_y - getYFloat());

  motion_forward(MM_PER_BLOCK * distance_to_move, 0);
  motion_hold(100);

  setX(destination_x);
  setY(destination_y);
}




#endif // #ifndef COMPILE_FOR_PC

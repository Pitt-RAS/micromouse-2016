#include "driver.h"

#include <queue>

#ifdef COMPILE_FOR_PC
#include <fstream>
#include <iostream>
#include <unistd.h>
#endif

#ifndef COMPILE_FOR_PC
#include <Arduino.h>
#include <EEPROM.h>
#include "data.h"
#include "motion.h"
#include "parser.h"
#include "conf.h"
#include "sensors_encoders.h"
#include "sensors_orientation.h"
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

void Driver::saveState(Maze<16, 16>& maze) {
#ifdef COMPILE_FOR_PC
  std::ofstream out_file;
  out_file.open("saved_state.maze");
#endif

  for (int x = 0; x < 16; x++) {
    for (int y = 0; y < 16; y++) {
      uint8_t out = 0;
      out |= maze.isWall(x, y, kNorth) << 0;
      out |= maze.isWall(x, y, kEast) << 1;
      out |= maze.isWall(x, y, kSouth) << 2;
      out |= maze.isWall(x, y, kWest) << 3;
      out |= maze.isVisited(x, y) << 4;
#ifdef COMPILE_FOR_PC
      out_file << out;
#else
      EEPROM.write(EEPROM_MAZE_LOCATION + 16*x + y, out);
#endif
    }
  }

#ifdef COMPILE_FOR_PC
  out_file.close();
#else
  EEPROM.write(EEPROM_MAZE_FLAG_LOCATION, 1);
#endif
}

void Driver::loadState(Maze<16, 16>& maze) {
#ifdef COMPILE_FOR_PC
  std::ifstream in_file;
  in_file.open("saved_state.maze");
#endif

  for (int x = 0; x < 16; x++) {
    for (int y = 0; y < 16; y++) {
      uint8_t in;
#ifdef COMPILE_FOR_PC
      std::ifstream >> in;
#else
      in = EEPROM.read(EEPROM_MAZE_LOCATION + 16*x + y);
#endif

      if (in & (1 << 0)) {
        maze.addWall(x, y, kNorth);
      } else {
        maze.removeWall(x, y, kNorth);
      }

      if (in & (1 << 1)) {
        maze.addWall(x, y, kEast);
      } else {
        maze.removeWall(x, y, kEast);
      }

      if (in & (1 << 2)) {
        maze.addWall(x, y, kSouth);
      } else {
        maze.removeWall(x, y, kSouth);
      }

      if (in & (1 << 3)) {
        maze.addWall(x, y, kWest);
      } else {
        maze.removeWall(x, y, kWest);
      }

      if (in & (1 << 4)) {
        maze.visit(x, y);
      } else {
        maze.unvisit(x, y);
      }
    }
  }

#ifdef COMPILE_FOR_PC
  in_file.close();
#endif
}

void Driver::updateState(Maze<16, 16>& maze, size_t x, size_t y) {
  uint8_t out = 0;
  out |= maze.isWall(x, y, kNorth) << 0;
  out |= maze.isWall(x, y, kEast) << 1;
  out |= maze.isWall(x, y, kSouth) << 2;
  out |= maze.isWall(x, y, kWest) << 3;
  out |= maze.isVisited(x, y) << 4;

#ifdef COMPILE_FOR_PC
  std::fstream out_file;
  out_file.open("saved_state.maze");
  out_file.seekp(16*x + y);
  out_file << out;
  out_file.close();
#else
  EEPROM.write(EEPROM_MAZE_LOCATION + 16*x + y, out);
#endif
}

void Driver::clearState() {
#ifdef COMPILE_FOR_PC
  remove("saved_state.maze");
#else
  Maze<16, 16> maze;
  saveState(maze);
  EEPROM.write(EEPROM_MAZE_FLAG_LOCATION, 0);
#endif
}

void Driver::resetState() {
#ifndef COMPILE_FOR_PC
  Maze<16, 16> maze;
  saveState(maze);
#endif
}

bool Driver::hasStoredState() {
#ifdef COMPILE_FOR_PC
  std::ifstream test_file ("saved_state.maze");
  if (test_file.good()) {
    test_file.close();
    return true;
  } else {
    test_file.close();
    return false;
  }
#else
  return EEPROM.read(EEPROM_MAZE_FLAG_LOCATION) != 0;
#endif
}


float Turnable::kDefaultInitialDirection = 0.0;

Compass8 Turnable::getDir()
{
  return (Compass8) ( (((int) dir_ + 45 / 2) % 360) / 45 );
}

Compass8 Turnable::getDirIMadeThisPublic()
{
  return getDir();
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

Compass8 Turnable::absoluteDir(Compass8 relative_dir)
{
  int arc;

  arc = (int) relative_dir + (int) getDir();

  if (arc > 7)
    arc -= 8;

  return (Compass8) arc;
}

Turnable::Turnable() : kInitialDirection(kDefaultInitialDirection)
{
  dir_ = kInitialDirection;
}

void Turnable::setDefaultInitialDirection(Compass8 dir) {
  kDefaultInitialDirection = ((float)dir) * 45;
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




bool ContinuousRobotDriver::onEdge()
{
  float x_offset, y_offset;

  x_offset = getXFloat() - getX();
  y_offset = getYFloat() - getY();

  if (x_offset < -0.25 || x_offset > 0.25)
    return true;
  if (y_offset < -0.25 || y_offset > 0.25)
    return true;

  return false;
}

ContinuousRobotDriver::ContinuousRobotDriver() :
    exit_velocity_(0.0), turn_advanced_(false)
{
  // Initialize whatever we need to.
}


int ContinuousRobotDriver::getX()
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

int ContinuousRobotDriver::getY()
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

void ContinuousRobotDriver::turn(Compass8 dir)
{
  float arc_to_turn;

  arc_to_turn = 45.0 * (int) relativeDir(dir);

  if (arc_to_turn > 180.0)
    arc_to_turn -= 360.0;

  turn_advanced_ = false;

  if (onEdge() && exit_velocity_ > 0.0) {
    switch(relativeDir(dir)) {
      case kNorth:
        break;
      case kSouth:
        motion_forward(MM_PER_BLOCK / 2, SEARCH_VELOCITY, 0.0);
        motion_rotate(180.0);
        motion_forward(MM_PER_BLOCK / 2, 0, exit_velocity_);
        turn_advanced_ = true;
        break;
      case kEast:
        motion_corner(kRightTurn90, exit_velocity_);
        turn_advanced_ = true;
        break;
      case kWest:
        motion_corner(kLeftTurn90, exit_velocity_);
        turn_advanced_ = true;
        break;
      default:
        motion_forward(MM_PER_BLOCK / 4, SEARCH_VELOCITY, 0.0);
        motion_hold(100);
        delay(3000);
        freakOut("BAD4");
        break;
    }

    if (relativeDir(dir) == kEast || relativeDir(dir) == kWest) {
      switch (getDir()) {
        case kNorth:
          setY(getYFloat() + 0.5);
          break;
        case kSouth:
          setY(getYFloat() - 0.5);
          break;
        case kEast:
          setX(getXFloat() + 0.5);
          break;
        case kWest:
          setX(getXFloat() - 0.5);
          break;
      }

      switch (dir) {
        case kNorth:
          setY(getYFloat() + 0.5);
          break;
        case kSouth:
          setY(getYFloat() - 0.5);
          break;
        case kEast:
          setX(getXFloat() + 0.5);
          break;
        case kWest:
          setX(getXFloat() - 0.5);
          break;
      }
    }
  }
  else if (!onEdge() && exit_velocity_ == 0.0) {
    motion_rotate(arc_to_turn);
    exit_velocity_ = 0.0;
  }
  else {
    motion_forward(MM_PER_BLOCK / 4, SEARCH_VELOCITY, 0.0);
    motion_hold(100);
    freakOut("BAD1");
  }

  setDir(dir);
}

bool ContinuousRobotDriver::isWall(Compass8 dir)
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

void ContinuousRobotDriver::move(Compass8 dir, int distance)
{

  //char buf[5];
  //snprintf(buf, 5, "%02d%02d", getX(), getY());
  //menu.showString(buf, 4);
  float distance_to_add;
  menu.showInt((int)dir, 4);

  if (distance == 0) {
    if (onEdge() && exit_velocity_ > 0.0) {
      motion_forward(MM_PER_BLOCK / 2, SEARCH_VELOCITY, 0.0);
      motion_hold(10);
      exit_velocity_ = 0.0;

      switch (getDir()) {
        case kNorth:
          setY(getYFloat() + 0.5);
          break;

        case kSouth:
          setY(getYFloat() - 0.5);
          break;

        case kEast:
          setX(getXFloat() + 0.5);
          break;

        case kWest:
          setX(getXFloat() - 0.5);
          break;
      }
      turn(dir);
      return;
    }
    else if (!onEdge() && exit_velocity_ == 0.0) {
      turn(dir);
      return;
    }
    else {
      motion_forward(MM_PER_BLOCK / 4, SEARCH_VELOCITY, 0.0);
      motion_hold(100);
      freakOut("BAD2");
    }
  }

  turn(dir);

  if (turn_advanced_)
    distance--;

  if (distance < 1)
    return;

  if (onEdge() && exit_velocity_ > 0.0) {
    motion_forward(MM_PER_BLOCK * distance, SEARCH_VELOCITY, exit_velocity_);
    distance_to_add = distance;
  }
  else if (!onEdge() && exit_velocity_ == 0.0) {
    motion_forward(MM_PER_BLOCK / 2 + MM_PER_BLOCK * (distance - 1), 0, SEARCH_VELOCITY);
    exit_velocity_ = SEARCH_VELOCITY;
    distance_to_add = distance - 0.5;
  }
  else {
    motion_forward(MM_PER_BLOCK / 4, SEARCH_VELOCITY, 0.0);
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

  motion_forward(MM_PER_BLOCK * distance_to_move, 0, 0);
  motion_hold(10);

  setX(destination_x);
  setY(destination_y);
}




void ContinuousRobotDriverRefactor::turn_in_place(Compass8 dir)
{
  float arc;

  arc = 45.0 * relativeDir(dir);

  if (arc > 180.0)
    arc -= 360.0;

  motion_rotate(arc);

  setDir(dir);
}

void ContinuousRobotDriverRefactor::turn_while_moving(Compass8 dir)
{
  bool is_front_wall, is_left_wall, is_right_wall;

  switch(relativeDir(dir)) {
    case kNorth:
      pivot_turns_in_a_row_ = 0;
      motion_forward(MM_PER_BLOCK, search_velocity_, search_velocity_);
      break;
    case kSouth:
      pivot_turns_in_a_row_ = 0;
      is_front_wall = isWall(absoluteDir(kNorth));
      is_left_wall = isWall(absoluteDir(kWest));
      is_right_wall = isWall(absoluteDir(kEast));
      motion_forward(MM_PER_BLOCK / 2, search_velocity_, 0.0);

      if (is_front_wall) {
        motion_hold_range(MOTION_RESET_HOLD_DISTANCE, 500);

        enc_left_back_write(0);
        enc_right_back_write(0);
        enc_left_front_write(0);
        enc_right_front_write(0);
        Orientation::getInstance()->resetHeading();
      }

      if (is_right_wall) {
        motion_rotate(90);
        motion_hold_range(MOTION_RESET_HOLD_DISTANCE, 500);

        enc_left_back_write(0);
        enc_right_back_write(0);
        enc_left_front_write(0);
        enc_right_front_write(0);
        Orientation::getInstance()->resetHeading();

        motion_rotate(90);
      } else if (is_left_wall) {
        motion_rotate(-90);
        motion_hold_range(MOTION_RESET_HOLD_DISTANCE, 500);

        enc_left_back_write(0);
        enc_right_back_write(0);
        enc_left_front_write(0);
        enc_right_front_write(0);
        Orientation::getInstance()->resetHeading();

        motion_rotate(-90);
      } else {
        motion_rotate(180);
      }

      motion_forward(MM_PER_BLOCK / 2, 0.0, search_velocity_);

      break;
    case kEast:
      if (pivot_turns_in_a_row_ > 30000) {
        bool hold_front = isWall(absoluteDir(kNorth));
        bool backup = isWall(absoluteDir(kWest));
        motion_forward(MM_PER_BLOCK / 2, search_velocity_, 0);
        if (hold_front) motion_hold_range(MM_PER_BLOCK / 2, 500);
        motion_rotate(90);
        if (backup) {
          float old_max_vel = motion_get_maxVel_straight();
          motion_set_maxVel_straight(MOTION_RESET_BACKUP_VEL);
          motion_forward(-MM_FROM_BACK_TO_CENTER - MOTION_RESET_BACKUP_DISTANCE, 0, 0);
          motion_set_maxVel_straight(old_max_vel);

          enc_left_back_write(0);
          enc_right_back_write(0);
          enc_left_front_write(0);
          enc_right_front_write(0);
          Orientation::getInstance()->resetHeading();

          motion_forward(MM_FROM_BACK_TO_CENTER + MM_PER_BLOCK / 2, 0, search_velocity_);
        } else {
          motion_forward(MM_PER_BLOCK / 2, 0, search_velocity_);
        }
        pivot_turns_in_a_row_ = 0;
      }
      else {
        motion_forward(12, search_velocity_, search_velocity_);
        motion_corner(kRightTurn90, search_velocity_, 160./180);
        motion_forward(12, search_velocity_, search_velocity_);
        pivot_turns_in_a_row_++;
      }
      //motion_forward(10, search_velocity_, search_velocity_);
      //motion_corner(kRightTurn90, search_velocity_, 160./180);
      //motion_forward(10, search_velocity_, search_velocity_);
      break;
    case kWest:
      if (pivot_turns_in_a_row_ > 30000) {
        bool hold_front = isWall(absoluteDir(kNorth));
        bool backup = isWall(absoluteDir(kEast));
        motion_forward(MM_PER_BLOCK / 2, search_velocity_, 0);
        if (hold_front) motion_hold_range(MM_PER_BLOCK / 2, 500);
        motion_rotate(-90);
        if (backup) {
          float old_max_vel = motion_get_maxVel_straight();
          motion_set_maxVel_straight(MOTION_RESET_BACKUP_VEL);
          motion_forward(-MM_FROM_BACK_TO_CENTER - MOTION_RESET_BACKUP_DISTANCE, 0, 0);
          motion_set_maxVel_straight(old_max_vel);

          enc_left_back_write(0);
          enc_right_back_write(0);
          enc_left_front_write(0);
          enc_right_front_write(0);
          Orientation::getInstance()->resetHeading();

          motion_forward(MM_FROM_BACK_TO_CENTER + MM_PER_BLOCK / 2, 0, search_velocity_);
        } else {
          motion_forward(MM_PER_BLOCK / 2, 0, search_velocity_);
        }
        pivot_turns_in_a_row_ = 0;
      }
      else {
        motion_forward(12, search_velocity_, search_velocity_);
        motion_corner(kLeftTurn90, search_velocity_, 160./180);
        motion_forward(12, search_velocity_, search_velocity_);
        pivot_turns_in_a_row_++;
      }
      //motion_forward(10, search_velocity_, search_velocity_);
      //motion_corner(kLeftTurn90, search_velocity_, 160./180);
      //motion_forward(10, search_velocity_, search_velocity_);
      break;
    default:
      freakOut("BAG1");
  }
}

void ContinuousRobotDriverRefactor::beginFromCenter(Compass8 dir)
{
  turn_in_place(dir);
  motion_forward(MM_PER_BLOCK / 2, 0.0, search_velocity_);

  switch(dir) {
    case kNorth:
      setY(getY() + 1);
      break;
    case kSouth:
      setY(getY() - 1);
      break;
    case kEast:
      setX(getX() + 1);
      break;
    case kWest:
      setX(getX() - 1);
      break;
    default:
      freakOut("BAG3");
      break;
  }

  setDir(dir);
}

void ContinuousRobotDriverRefactor::beginFromBack(Compass8 dir, int distance)
{
  if (dir != getDir()) {
    motion_forward(MM_FROM_BACK_TO_CENTER, 0, 0);
    turn_in_place(dir);
    setDir(dir);

    if (distance > 0) {
      motion_forward(MM_PER_BLOCK / 2, 0, search_velocity_);

      switch(dir) {
        case kNorth:
          setY(getY() + 1);
          break;
        case kSouth:
          setY(getY() - 1);
          break;
        case kEast:
          setX(getX() + 1);
          break;
        case kWest:
          setX(getX() - 1);
          break;
        default:
          freakOut("BAG3");
          break;
      }

      if (distance > 1)
        proceed(dir, distance - 1);
    }
  } else {
    if (distance > 0) {
      motion_forward(MM_FROM_BACK_TO_CENTER + MM_PER_BLOCK / 2, 0, search_velocity_);

      switch(dir) {
        case kNorth:
          setY(getY() + 1);
          break;
        case kSouth:
          setY(getY() - 1);
          break;
        case kEast:
          setX(getX() + 1);
          break;
        case kWest:
          setX(getX() - 1);
          break;
        default:
          freakOut("BAG3");
          break;
      }

      if (distance > 1)
        proceed(dir, distance - 1);

    } else {
      motion_forward(MM_FROM_BACK_TO_CENTER, 0, 0);
    }
  }
}

void ContinuousRobotDriverRefactor::stop(Compass8 dir)
{
  motion_forward(MM_PER_BLOCK / 2, search_velocity_, 0.0);
  turn_in_place(dir);
  motion_hold(10);

  setDir(dir);
}

void ContinuousRobotDriverRefactor::proceed(Compass8 dir, int distance)
{
  turn_while_moving(dir);

  if (distance > 1)
    motion_forward(MM_PER_BLOCK * (distance - 1), search_velocity_, search_velocity_);

  switch(dir) {
    case kNorth:
      setY(getY() + distance);
      break;
    case kSouth:
      setY(getY() - distance);
      break;
    case kEast:
      setX(getX() + distance);
      break;
    case kWest:
      setX(getX() - distance);
      break;
    default:
      freakOut("BAG2");
      break;
  }

  setDir(dir);
}

ContinuousRobotDriverRefactor::ContinuousRobotDriverRefactor(int x, int y, Compass8 direction, bool begin_from_back) : moving_(false), last_direction_(kNorth), moves_in_this_direction_(0), pivot_turns_in_a_row_(0),
    left_back_wall_(false)
{
  setX(x);
  setY(y);
  setDir(direction);

  uint16_t loaded_int = (uint16_t)EEPROM.read(EEPROM_SEARCH_VEL_LOCATION) << 8;
  loaded_int |= EEPROM.read(EEPROM_SEARCH_VEL_LOCATION + 1);
  search_velocity_ = (float)loaded_int / 100;
  motion_set_maxVel_straight(search_velocity_);

  loaded_int = (uint16_t)EEPROM.read(EEPROM_SEARCH_ACCEL_LOCATION) << 8;
  loaded_int |= EEPROM.read(EEPROM_SEARCH_ACCEL_LOCATION + 1);
  motion_set_maxAccel_straight((float)loaded_int / 10);

  loaded_int = (uint16_t)EEPROM.read(EEPROM_SEARCH_DECEL_LOCATION) << 8;
  loaded_int |= EEPROM.read(EEPROM_SEARCH_DECEL_LOCATION + 1);
  motion_set_maxDecel_straight(-(float)loaded_int / 10);

  if (!begin_from_back)
    left_back_wall_ = true;
}

void ContinuousRobotDriverRefactor::turn(Compass8 dir)
{
  // Not using this method.
}

bool ContinuousRobotDriverRefactor::isWall(Compass8 dir)
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

void ContinuousRobotDriverRefactor::move(Compass8 dir, int distance)
{
  bool will_end_moving;

  will_end_moving = distance > 0;

  if (!left_back_wall_) {
    beginFromBack(dir, distance);
    moving_ = will_end_moving;
    left_back_wall_ = true;
    return;
  }

  if (moving_) {
    if (will_end_moving) {
      proceed(dir, distance);
    }
    else {
      stop(dir);
    }
  }
  else {
    if (will_end_moving) {
      beginFromCenter(dir);

      if (distance > 1)
        proceed(dir, distance - 1);
    }
    else {
      turn_in_place(dir);
    }
  }

  moving_ = will_end_moving;

  if (dir == last_direction_) {
      moves_in_this_direction_++;
  }
  else {
      moves_in_this_direction_ = 0;
  }
  if (moves_in_this_direction_ > 17) {
      //motion_forward(30.0, search_velocity_, search_velocity_);
  }
  last_direction_ == dir;
}

void ContinuousRobotDriverRefactor::move(Path<16, 16>* path)
{
  Driver::move(*path);
}

KaosDriver::KaosDriver()
{
}

float KaosDriver::turn_velocity_ = KAOS_TURN_VEL;
float KaosDriver::max_forward_velocity_ = KAOS_FORWARD_VEL;
float KaosDriver::max_accel_ = 0;
float KaosDriver::max_decel_ = 0;

void KaosDriver::execute(std::queue<int> move_list)
{
  if (move_list.empty()) return;

  uint16_t loaded_int = 0;

  loaded_int = (uint16_t)EEPROM.read(EEPROM_KAOS_TURN_VEL_LOCATION) << 8;
  loaded_int |= EEPROM.read(EEPROM_KAOS_TURN_VEL_LOCATION + 1);
  turn_velocity_ = (float)loaded_int / 100;

  loaded_int = (uint16_t)EEPROM.read(EEPROM_KAOS_FORWARD_VEL_LOCATION) << 8;
  loaded_int |= EEPROM.read(EEPROM_KAOS_FORWARD_VEL_LOCATION + 1);
  max_forward_velocity_ = (float)loaded_int / 100;

  loaded_int = (uint16_t)EEPROM.read(EEPROM_KAOS_ACCEL_LOCATION) << 8;
  loaded_int |= EEPROM.read(EEPROM_KAOS_ACCEL_LOCATION + 1);
  max_accel_ = (float)loaded_int / 10;

  loaded_int = (uint16_t)EEPROM.read(EEPROM_KAOS_DECEL_LOCATION) << 8;
  loaded_int |= EEPROM.read(EEPROM_KAOS_DECEL_LOCATION + 1);
  max_decel_ = -(float)loaded_int / 10;

  float old_max_velocity = motion_get_maxVel_straight();
  motion_set_maxVel_straight(max_forward_velocity_);
  float old_max_accel = motion_get_maxAccel_straight();
  motion_set_maxAccel_straight(max_accel_);
  float old_max_decel = motion_get_maxDecel_straight();
  motion_set_maxDecel_straight(max_decel_);

  int next_move = move_list.front();
  float len = 0;
  move_list.pop();

  motion_forward(MM_FROM_BACK_TO_CENTER, 0, turn_velocity_);
  enc_left_front_write(0);
  enc_left_back_write(0);
  enc_right_front_write(0);
  enc_right_back_write(0);
  Orientation::getInstance()->resetHeading();

  bool should_continue = true;
  while (!move_list.empty() || should_continue) {
    if (move_list.empty()) {
      should_continue = false;
    }

    tone(BUZZER_PIN, 2000, 100);

    switch (next_move) {
      case quarter:
      case half:
      case forward:
        len = 0;
        do {
          if (next_move == half)
            len += 0.5;
          else if(next_move == quarter)
            len += 0.25;
          else
            len += 1;
          if (!move_list.empty()) {
            next_move = move_list.front();
            move_list.pop();
          }
        } while (!move_list.empty() && (next_move == forward || next_move == half || next_move == quarter));
        motion_forward(MM_PER_BLOCK * len, turn_velocity_, turn_velocity_);
        break;
      case left_90:
        motion_corner(kLeftTurn90, turn_velocity_);
        if (!move_list.empty()) {
          next_move = move_list.front();
          move_list.pop();
        }
        break;
      case right_90:
        motion_corner(kRightTurn90, turn_velocity_);
        if (!move_list.empty()) {
          next_move = move_list.front();
          move_list.pop();
        }
        break;
      case diag_left_90:
        motion_corner(kLeftTurn90, turn_velocity_, 1/sqrt(2));
        if (!move_list.empty()) {
          next_move = move_list.front();
          move_list.pop();
        }
        break;
      case diag_right_90:
        motion_corner(kRightTurn90, turn_velocity_, 1/sqrt(2));
        if (!move_list.empty()) {
          next_move = move_list.front();
          move_list.pop();
        }
        break;
      case left_180:
        motion_corner(kLeftTurn180, turn_velocity_);
        if (!move_list.empty()) {
          next_move = move_list.front();
          move_list.pop();
        }
        break;
      case right_180:
        motion_corner(kRightTurn180, turn_velocity_);
        if (!move_list.empty()) {
          next_move = move_list.front();
          move_list.pop();
        }
        break;
      case enter_left_45:
        //motion_forward(MM_PER_BLOCK * (0.75 - 0.25 / (sqrt(2) - 1)), turn_velocity_, turn_velocity_);
        motion_corner(kLeftTurn45, turn_velocity_, (sqrt(2) / 2 / (sqrt(2) - 1)));
        if (!move_list.empty()) {
          next_move = move_list.front();
          move_list.pop();
        }
        break;
      case enter_right_45:
        //motion_forward(MM_PER_BLOCK * (0.75 - 0.25 / (sqrt(2) - 1)), turn_velocity_, turn_velocity_);
        motion_corner(kRightTurn45, turn_velocity_, (sqrt(2) / 2 / (sqrt(2) - 1)));
        if (!move_list.empty()) {
          next_move = move_list.front();
          move_list.pop();
        }
        break;
      case exit_left_45:
        motion_corner(kLeftTurn45, turn_velocity_, (sqrt(2) / 2 / (sqrt(2) - 1)));
        //motion_forward(MM_PER_BLOCK * (0.75 - 0.25 / (sqrt(2) - 1)), turn_velocity_, turn_velocity_);
        if (!move_list.empty()) {
          next_move = move_list.front();
          move_list.pop();
        }
        break;
      case exit_right_45:
        motion_corner(kRightTurn45, turn_velocity_, (sqrt(2) / 2 / (sqrt(2) - 1)));
        //motion_forward(MM_PER_BLOCK * (0.75 - 0.25 / (sqrt(2) - 1)), turn_velocity_, turn_velocity_);
        if (!move_list.empty()) {
          next_move = move_list.front();
          move_list.pop();
        }
        break;
      case diag:
        len = 0;
        do {
          len++;
          if (!move_list.empty()) {
            next_move = move_list.front();
            move_list.pop();
          }
        } while (!move_list.empty() && next_move == diag);
        motion_forward_diag(MM_PER_BLOCK * 0.707 * len, turn_velocity_, turn_velocity_);
        break;
      case pivot_180:
        motion_rotate(180);
        if (!move_list.empty()) {
          next_move = move_list.front();
          move_list.pop();
        }
        break;
      case pivot_left_90:
        motion_rotate(-90);
        if (!move_list.empty()) {
          next_move = move_list.front();
          move_list.pop();
        }
        break;
      case pivot_right_90:
        motion_rotate(90);
        if (!move_list.empty()) {
          next_move = move_list.front();
          move_list.pop();
        }
        break;
      case enter_right_135:
        motion_forward(MM_PER_BLOCK * (.75 - .75/(sqrt(2) + 1)), turn_velocity_, turn_velocity_);
        motion_corner(kRightTurn135, turn_velocity_, TURN_135_SCALING*(3 * sqrt(2) / 2 / (sqrt(2) + 1)));
        if (!move_list.empty()) {
          next_move = move_list.front();
          move_list.pop();
        }
        break;
      case enter_left_135:
        motion_forward(MM_PER_BLOCK * (.75 - .75/(sqrt(2) + 1)), turn_velocity_, turn_velocity_);
        motion_corner(kLeftTurn135, turn_velocity_, TURN_135_SCALING*(3 * sqrt(2) / 2 / (sqrt(2) + 1)));
        if (!move_list.empty()) {
          next_move = move_list.front();
          move_list.pop();
        }
        break;
      case exit_right_135:
        motion_corner(kRightTurn135, turn_velocity_, TURN_135_SCALING*(3 * sqrt(2) / 2 / (sqrt(2) + 1)));
        motion_forward(MM_PER_BLOCK * (.75 - .75/(sqrt(2) + 1)), turn_velocity_, turn_velocity_);
        if (!move_list.empty()) {
          next_move = move_list.front();
          move_list.pop();
        }
        break;
      case exit_left_135:
        motion_corner(kLeftTurn135, turn_velocity_, TURN_135_SCALING*(3 * sqrt(2) / 2 / (sqrt(2) + 1)));
        motion_forward(MM_PER_BLOCK * (.75 - .75/(sqrt(2) + 1)), turn_velocity_, turn_velocity_);
        if (!move_list.empty()) {
          next_move = move_list.front();
          move_list.pop();
        }
        break;
      default:
        freakOut("DAMN");
        break;
    }
  }

  motion_forward(MM_PER_BLOCK / 6, turn_velocity_, 0);
  motion_forward(-MM_PER_BLOCK / 6, 0, 0);

  motion_set_maxVel_straight(old_max_velocity);
  motion_set_maxAccel_straight(old_max_accel);
  motion_set_maxDecel_straight(old_max_decel);
}

#endif // #ifndef COMPILE_FOR_PC

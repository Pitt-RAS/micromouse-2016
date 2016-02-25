#ifndef DRIVER_H
#define DRIVER_H

#include "data.h"

#ifdef COMPILE_FOR_PC
#include <fstream>
#endif

#ifndef COMPILE_FOR_PC
#include <Arduino.h>
#endif

// This class is a highly portable interface between a high-level algorithm and
// a low-level physical implementation.
//
// Why:
//   - Forces us to abstract away low-level concepts as early as possible.
//   - Automatically provides the highest-level position information in the
//     form of integer box coordinates. Floating point coordinates are only
//     accessible from within this class.
//   - An algorithm that uses this class can be used with a very physically
//     different robot or with a simulator by simply dropping in a different
//     derived class.
//
// The idea is to use a Driver reference that points to an instance of a
// derived class. This way, a different derived class can be dropped in without
// changing the high-level code.
//
//   DerivedDriverClass derived_driver;
//
//   derived_driver.someDerivedSpecificMethod();
//   derived_driver.someOtherDerivedSpecificMethod(some_data);
//
//   Driver &driver = derived_driver;
//
//   driver.move(kNorth, 3);
//   driver.move(kEast, 2);
//
class Driver
{
  private:
    const int kXSize;
    const int kYSize;

    const float kInitialXPosition;
    const float kInitialYPosition;

    float x_, y_, dir_;
    float x_size_, y_size_;

  protected:
    // Returns the robot x position (floating point).
    // unit: blocks East of the SW corner
    float getXFloat();

    // Returns the robot y position (floating point).
    // unit: blocks North of the SW corner
    float getYFloat();


    // Sets the robot x position.
    // unit: blocks East of the SW corner
    void setX(float x);

    // Sets the robot y position.
    // unit: blocks North of the SW corner
    void setY(float y);

  public:
    Driver();


    // Returns the robot x position (integer).
    // unit: blocks East of the SW corner
    int getX();

    // Returns the robot y position (integer).
    // unit: blocks North of the SW corner
    int getY();

    // Then next two methods must be implemented in a derived class.

    // Returns whether there is a wall in the given direction.
    virtual bool isWall(Compass8 dir) = 0;

    // Moves the robot a number of blocks in a given direction.
    virtual void move(Compass8 dir, int distance) = 0;
};

// Standard driver interface for a robot that must be turned (as opposed to one
// that has no particular forward facing direction)
class Turnable
{
  private:
    const float kInitialDirection;

    float dir_;

  protected:
    // Returns the stored robot direction as a Compass8 direction.
    Compass8 getDir();

    // Returns the stored robot direction as a floating point compass direction
    // in degrees. The returned value will always be between 0.0 and 360.0.
    float getDirF();

    // Sets the stored robot direction given a Compass8 direction.
    void setDir(Compass8 dir);

    // Sets the stored robot direction given a floating point compass direction
    // in degrees. Values less than 0.0 or greater 360.0 are allowed.
    void setDir(float dir);

    // The next method must be implemented in a derived class.

    // Turns the robot to the given direction. This method is responsible for
    // physically turning the robot. It should keep track of the robot's
    // direction by using getDir() and setDir().
    virtual void turn(Compass8 dir) = 0;

  public:
    Turnable();
};

// Standard interface for a simulation driver (i.e. a driver that does not
// correspond to a physical robot but that exists completely in software)
//
// Uses the turnable interface.
class SimulationDriver : public Driver, public Turnable
{
  private:
    const static int kDefaultSleepTime = 500;

    int sleep_time_;

    // implementation of the virtual method in Turnable
    void turn(Compass8 dir);

    // The next two methods must be implemented in a derived class.

    // Updates the simulation backend. This method should give the simulation
    // any new information and cause it to redisplay or redraw.
    virtual void update() = 0;

    // This method should use a system-specific utility to block program
    // execution for the amount of time returned by getSleepTime().
    virtual void sleep() = 0;
  protected:
    Maze<16, 16> real_maze_;

    // Returns the amount of time for which sleep() should block execution.
    int getSleepTime();

  public:
    SimulationDriver();

    // Sets the amount of time for which sleep() should block execution.
    void setSleepTime(int time);

    // implementations of the methods in Driver
    bool isWall(Compass8 dir);
    void move(Compass8 dir, int distance);
};

#ifdef COMPILE_FOR_PC

// Simulation driver that displays the robot position on standard output
class StdoutDriver : public SimulationDriver
{
  private:
    // implementations of the methods in SimulationDriver
    void update();
    void sleep();

  public:
    StdoutDriver();
};

// Simulation driver that writes drawing commands to an output file intended to
// be read by micromousecanvas
class CanvasDriver : public SimulationDriver
{
  private:
    std::ofstream file_;

    void init();

    // implementations of the methods in SimulationDriver
    void update();
    void sleep();
  public:
    CanvasDriver();
    ~CanvasDriver();
};

#endif // #ifdef COMPILE_FOR_PC

#ifndef COMPILE_FOR_PC

// Simulation driver that displays the robot position on the Arduino serial
// output
class SerialDriver : public SimulationDriver
{
  private:
    // implementations of the methods in SimulationDriver
    void update();
    void sleep();

  public:
    SerialDriver();
};

// Driver for our Micromouse robot
class RobotDriver : public Driver, public Turnable
{
  public:
    RobotDriver();

    void turn(Compass8 dir);

    bool isWall(Compass8 dir);
    void move(Compass8 dir, int distance);
};

#endif // #ifndef COMPILE_FOR_PC

#endif // #ifndef DRIVER_H
#ifndef MICROMOUSE_NAV_H_
#define MICROMOUSE_NAV_H_

#include "data.h"
#include "driver.h"
#include "Menu.h"

// Everything in this file MUST be portable code.
//
// These things are NOT portable:
//   - Serial, Serial1, Serial2...
//   - printf, cin, cout, cerr...
//   - Arduino libraries
//   - Any libraries at all
//   - Anything that requires the Arduino IDE
//   - Anything that requires a PC
//   - Using the derived_driver variable
//
// The Goal: It should be easy as 1-2-3.
//   1. Copy this file to the mazesim-rewrite source tree.
//   2. Compile the program there.
//   3. Run the simulator on your Linux/Mac/Windows PC.
//
// Example usage:
//
//   Navigator<RobotDriver> navigator1;
//   navigator1.runDevelopmentCode();
//
//   Navigator<CanvasDriver> navigator2;
//   navigator2.runDevelopmentCode();
//
template <typename driver_type>
class Navigator
{
  private:
    driver_type derived_driver;
    Driver &driver;
    Maze<16, 16> maze;

  public:
    Navigator();

    void nod();

    void driveBox2UnitsLeftTurns();
    void driveBox2UnitsRightTurns();

    void driveCardboardMaze();

    void findBox(int x, int y);

    // For development. Fill this method in with whatever you're working on.
    // This method is what will be called by mazesim-rewrite.
    void runDevelopmentCode();
};




// The methods have to be defined in this file because this is a template.




template <typename driver_type>
Navigator<driver_type>::Navigator() : derived_driver(), driver(derived_driver)
{
}

template <typename driver_type>
void Navigator<driver_type>::nod()
{
  driver.move(kNorthWest, 0);
  driver.move(kNorthEast, 0);
  driver.move(kNorthWest, 0);
  driver.move(kNorthEast, 0);
  driver.move(kNorth, 0);
}

template <typename driver_type>
void Navigator<driver_type>::driveBox2UnitsLeftTurns()
{
  driver.move(kNorth, 2);
  driver.move(kWest, 2);
  driver.move(kSouth, 2);
  driver.move(kEast, 2);
  driver.move(kNorth, 0);
}

template <typename driver_type>
void Navigator<driver_type>::driveBox2UnitsRightTurns()
{
  driver.move(kNorth, 2);
  driver.move(kEast, 2);
  driver.move(kSouth, 2);
  driver.move(kWest, 2);
  driver.move(kNorth, 0);
}

template <typename driver_type>
void Navigator<driver_type>::driveCardboardMaze()
{
  driver.move(kNorth, 3);
  driver.move(kEast, 1);
  driver.move(kWest, 1);
  driver.move(kSouth, 1);
  driver.move(kEast, 2);
  driver.move(kNorth, 1);
  driver.move(kSouth, 0);
}

template <typename driver_type>
void Navigator<driver_type>::findBox(int x, int y)
{
  while (driver.getX() != x || driver.getY() != y) {
    if (driver.isWall(kNorth))
      maze.addWall(driver.getX(), driver.getY(), kNorth);

    if (driver.isWall(kSouth))
      maze.addWall(driver.getX(), driver.getY(), kSouth);

    if (driver.isWall(kEast))
      maze.addWall(driver.getX(), driver.getY(), kEast);

    if (driver.isWall(kWest))
      maze.addWall(driver.getX(), driver.getY(), kWest);

    maze.visit(driver.getX(), driver.getY());

    {
      FloodFillPath<16, 16>
        flood_path(maze, driver.getX(), driver.getY(), x, y);

      KnownPath<16, 16>
        known_path(maze, driver.getX(), driver.getY(), x, y, flood_path);

      if (known_path.isEmpty())
        break;

      driver.move(known_path);
    }
  }

  driver.move(kNorth, 0);
}

template <typename driver_type>
void Navigator<driver_type>::runDevelopmentCode()
{
  nod();
}




#endif

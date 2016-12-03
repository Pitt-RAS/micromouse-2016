#include <Arduino.h>

#include "data.h"
#include "driver.h"
#include "Navigator.h"
#include "PersistantStorage.h"
#include "Orientation.h"
#include "PlayMelodies.h"
#include "sensors_encoders.h"
#include "utility.h"
#include "parser.h"
#include "motion.h"
#include "Menu.h"

Menu::Menu()
{
  gUserInterface.begin();
  Turnable::setDefaultInitialDirection(PersistantStorage::getDefaultDirection());
}

void Menu::main()
{
  const char* names[] = {
    "RUN",
    "KAOS",
    "TURN",
    "CHK",
    "OPT"
  };

  switch (gUserInterface.getString(names, 5, 4))
  {
    case 0:
      run();
      break;
    case 1:
      kaos();
      break;
    case 2:
      turn();
      break;
    case 3:
      check();
      break;
    case 4:
      options();
      break;
    default:
      break;
  }
}

void Menu::run()
{
  Navigator<ContinuousRobotDriver> navigator;
  Orientation* orientation = Orientation::getInstance();

  gUserInterface.waitForHand();
  startMelody();

  enc_left_front_write(0);
  enc_right_front_write(0);
  enc_left_back_write(0);
  enc_right_back_write(0);
  orientation->resetHeading();

  navigator.findBox(PersistantStorage::getTargetXLocation(),
                    PersistantStorage::getTargetYLocation());
  searchFinishMelody();
  navigator.findBox(0, 0);
  stopMelody();
}

void Menu::kaos()
{
  uint8_t target_x = PersistantStorage::getTargetXLocation();
  uint8_t target_y = PersistantStorage::getTargetYLocation();
  if (knowsBestPath(target_x, target_y)) {
    Compass8 absolute_end_direction;

    ContinuousRobotDriver maze_load_driver;
    Maze<16, 16> maze;
    maze_load_driver.loadState(maze);
    FloodFillPath<16, 16> flood_path (maze, 0, 0, target_x, target_y);
    KnownPath<16, 16> known_path (maze, 0, 0, target_x, target_y, flood_path);
    PathParser parser (&known_path);
    KaosDriver driver;

    gUserInterface.waitForHand();
    speedRunMelody();

    absolute_end_direction = parser.getEndDirection();

    driver.execute(parser.getMoveList());
    char buf[5];

    snprintf(buf, 5, "%02d%02d", parser.end_x, parser.end_y);
    gUserInterface.showString(buf, 4);
    searchFinishMelody();

    ContinuousRobotDriver other_driver(parser.end_x, parser.end_y, absolute_end_direction, false);

    {
      FloodFillPath<16, 16>
        flood_path(maze, other_driver.getX(), other_driver.getY(), 0, 0);

      KnownPath<16, 16>
        known_path(maze, other_driver.getX(), other_driver.getY(), 0, 0, flood_path);

      if (known_path.isEmpty())
        return;

      other_driver.move(known_path);

      snprintf(buf, 5, "%02d%02d", other_driver.getX(), other_driver.getY());
      gUserInterface.showString(buf, 4);

    }

    other_driver.move(kNorth, 0);
  }
}

void Menu::turn()
{
  gUserInterface.waitForHand();
  playNote(2000, 200);
  delay(1000);

  enc_left_front_write(0);
  enc_right_front_write(0);
  enc_left_back_write(0);
  enc_right_back_write(0);
  Orientation::getInstance()->resetHeading();

  motion_forward(180, 0, 0);
  motion_rotate(180);
  motion_forward(180, 0, 0);
  motion_hold(100);
}

void Menu::check()
{
  uint8_t target_x = PersistantStorage::getTargetXLocation();
  uint8_t target_y = PersistantStorage::getTargetYLocation();
  if (knowsBestPath(target_x, target_y)) {
    gUserInterface.showString("YES", 4);
  } else {
    gUserInterface.showString("NO", 4);
  }

  while (!gUserInterface.buttonOkPressed()) {
    // wait
  }
  delay(500);
}

void Menu::options()
{
  const char* names[] = {
    "CLR",
    "SDIR",
    "SPDS",
    "TRGT",
    "BACK"
  };

  switch (gUserInterface.getString(names, 5, 4)) {
    case 0:
      clear();
      break;
    case 1:
      startDirection();
      break;
    case 2:
      speeds();
      break;
    case 3:
      targetCell();
      break;
    case 4:
      // back
      break;
    default:
      break;
  }
}

void Menu::clear()
{
  RobotDriver driver;
  driver.clearState();
}

void Menu::startDirection()
{
  const char* names[] = {
    "NRTH",
    "EAST"
  };

  switch (gUserInterface.getString(names, 2, 4)) {
    case 0: { // NORTH
      Turnable::setDefaultInitialDirection(kNorth);
      PersistantStorage::setDefaultDirection(kNorth);
      break;
    }
    case 1: { // EAST
      Turnable::setDefaultInitialDirection(kEast);
      PersistantStorage::setDefaultDirection(kEast);
      break;
    }
    default: {
      break;
    }
  }
}

void Menu::speeds()
{
  const char* names[] = {
    "SVEL",
    "SACC",
    "SDEC",
    "K FV",
    "K DV",
    "K TV",
    "KACC",
    "KDEC",
    "BACK"
  };

  while (1) {
    bool back = false;
    switch (gUserInterface.getString(names, 9, 4)) {
      case 0: { // SEARCH VELOCITY
        uint16_t result = PersistantStorage::getRawSearchVelocity();
        result = gUserInterface.getInt(0, 9999, result, 4);
        PersistantStorage::setRawSearchVelocity(result);
        break;
      }
      case 1: { // SEARCH FORWARD ACCEL
        uint16_t result = PersistantStorage::getRawSearchAccel();
        result = gUserInterface.getInt(0, 9999, result, 4);
        PersistantStorage::setRawSearchAccel(result);
        break;
      }
      case 2: { // SEARCH DECEL
        uint16_t result = PersistantStorage::getRawSearchDecel();
        result = gUserInterface.getInt(0, 9999, result, 4);
        PersistantStorage::setRawSearchDecel(result);
        break;
      }
      case 3: { // KAOS FORWARD VELOCITY
        uint16_t result = PersistantStorage::getRawKaosForwardVelocity();
        result = gUserInterface.getInt(0, 9999, result, 4);
        PersistantStorage::setRawKaosForwardVelocity(result);
        break;
      }
      case 4: { // KAOS DIAGONAL VELOCITY
        uint16_t result = PersistantStorage::getRawKaosDiagVelocity();
        result = gUserInterface.getInt(0, 9999, result, 4);
        PersistantStorage::setRawKaosDiagVelocity(result);
        break;
      }
      case 5: { // KAOS TURN VELOCITY
        uint16_t result = PersistantStorage::getRawKaosTurnVelocity();
        result = gUserInterface.getInt(0, 9999, result, 4);
        PersistantStorage::setRawKaosTurnVelocity(result);
        break;
      }
      case 6: { // KAOS FORWARD ACCEL
        uint16_t result = PersistantStorage::getRawKaosAccel();
        result = gUserInterface.getInt(0, 9999, result, 4);
        PersistantStorage::setRawKaosAccel(result);
        break;
      }
      case 7: { // KAOS DECEL
        uint16_t result = PersistantStorage::getRawKaosDecel();
        result = gUserInterface.getInt(0, 9999, result, 4);
        PersistantStorage::setRawKaosDecel(result);
        break;
      }
      case 8: { // BACK
        back = true;
        break;
      }
    }
    if (back) break;
  }
}

void Menu::targetCell()
{
  const char* names[] = {
    "X",
    "Y",
    "BACK"
  };

  while (1) {
    bool back = false;
    switch (gUserInterface.getString(names, 3, 4)) {
      case 0: { // X LOCATION
        uint8_t result = PersistantStorage::getTargetXLocation();
        result = gUserInterface.getInt(0, 15, result, 4);
        PersistantStorage::setTargetXLocation(result);
        break;
      }
      case 1: { // Y LOCATION
        uint8_t result = PersistantStorage::getTargetYLocation();
        result = gUserInterface.getInt(0, 15, result, 4);
        PersistantStorage::setTargetYLocation(result);
        break;
      }
      case 2: { // BACK
        back = true;
        break;
      }
    }
    if (back) break;
  }
}

#include <Arduino.h>

#include <LedDisplay.h>
#include "conf.h"
#include "data.h"
#include "driver.h"
#include "Logger.h"
#include "Navigator.h"
#include "Menu.h"
#include "motion.h"
#include "RangeSensorContainer.h"
#include "motors.h"
#include "sensors_encoders.h"
#include "sensors_orientation.h"
#include "EncoderMod.h"
#include "IdealSweptTurns.h"
#include <I2Cdev.h>
#include <MPU9150.h>

bool knowsBestPath();

void setup()
{
  // Set higher pwm frequency for smoother motor control.
  analogWriteFrequency(MOTOR_LF_PWM_PIN, 46875);
  analogWriteFrequency(MOTOR_RF_PWM_PIN, 46875);
  analogWriteFrequency(MOTOR_LB_PWM_PIN, 46875);
  analogWriteFrequency(MOTOR_RB_PWM_PIN, 46875);

  // PWM resolution is 0-1023.
  analogWriteResolution(10);

  pinMode(EMITTER_DIAG_LEFT_PIN, OUTPUT);
  pinMode(EMITTER_DIAG_RIGHT_PIN, OUTPUT);
  pinMode(EMITTER_FRONT_LEFT_PIN, OUTPUT);
  pinMode(EMITTER_FRONT_RIGHT_PIN, OUTPUT);

  digitalWrite(EMITTER_DIAG_LEFT_PIN, LOW);
  digitalWrite(EMITTER_DIAG_RIGHT_PIN, LOW);
  digitalWrite(EMITTER_FRONT_LEFT_PIN, LOW);
  digitalWrite(EMITTER_FRONT_RIGHT_PIN, LOW);

  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);

  pinMode(13, OUTPUT);
  digitalWrite(13, 0);

  Serial.begin(BAUD);

  //while (!menu.buttonOkPressed()) {}
  //delay(1000);
  //motion_forward(180, 0, 0.4);
  //motion_corner(kLeftTurn45, 0.4, 0.75);
  //motion_forward(180*sqrt(2), 0.4, 0);
  //while (1) {}

  menu.begin();
}

char* primary_options[] = {
  "RUN",
  "KAOS",
  "CHK",
  "OPT"
};

char* secondary_options[] = {
  "CLR",
  "SDIR",
  "BACK"
};

char* direction_options[] = {
  "NRTH",
  "EAST"
};

void loop()
{
  switch (menu.getString(primary_options, 4, 4)) {
    case 0: { // RUN
      Navigator<ContinuousRobotDriverRefactor> navigator;
      Orientation* orientation = Orientation::getInstance();

      menu.waitForHand();
      startMelody();

      enc_left_front_write(0);
      enc_right_front_write(0);
      enc_left_back_write(0);
      enc_right_back_write(0);
      orientation->resetHeading();

      navigator.runDevelopmentCode();
      break;
    }
    case 1: { // KAOS
      if (knowsBestPath()) {
        ContinuousRobotDriverRefactor maze_load_driver;
        Maze<16, 16> maze;
        maze_load_driver.loadState(maze);
        FloodFillPath<16, 16> flood_path (maze, 0, 0, 8, 8);
        KnownPath<16, 16> known_path (maze, 0, 0, 8, 8, flood_path);
        PathParser parser (&known_path);
        KaosDriver driver;

        menu.waitForHand();
        speedRunMelody();

        driver.execute(parser.getMoveList());
      }
      break;
    }
    case 2: { // CHK (checks if entire path has been discovered)
      if (knowsBestPath()) {
        menu.showString("YES", 4);
      } else {
        menu.showString("NO", 4);
      }

      while (!menu.buttonOkPressed()) {
        // wait
      }
      delay(500);
      break;
    }
    case 3: { // OPT
      switch (menu.getString(secondary_options, 3, 4)) {
        case 0: { // CLR
          RobotDriver driver;
          driver.clearState();
          break;
        }
        case 1: { // SDIR
          switch (menu.getString(direction_options, 2, 4)) {
            case 0: { // NORTH
              Turnable::setDefaultInitialDirection(kNorth);
              menu.storeDefaultDirection(kNorth);
              break;
            }
            case 1: { // EAST
              Turnable::setDefaultInitialDirection(kEast);
              menu.storeDefaultDirection(kEast);
              break;
            }
            default: {
              break;
            }
          }
        }
        case 2: // BACK
        default: {
          break;
        }
      }
      break;
    }
    default: {
      break;
    }
  }
}

bool knowsBestPath() {
  ContinuousRobotDriverRefactor driver;
  Maze<16, 16> maze;
  bool success = true;
  if (driver.hasStoredState()) {
    driver.loadState(maze);
    FloodFillPath<16, 16> flood_path1 (maze, 0, 0, 8, 8);
    FloodFillPath<16, 16> flood_path2 (maze, 0, 0, 8, 8);
    KnownPath<16, 16> known_path (maze, 0, 0, 8, 8, flood_path1);

    if (flood_path2.isEmpty()) {
      success = false;
    }

    while (!flood_path2.isEmpty()) {
      if (known_path.isEmpty()) {
        success = false;
        break;
      }
      if (flood_path2.nextDirection() != known_path.nextDirection()) {
        success = false;
        break;
      }
    }
  } else {
    success = false;
  }
  return success;
}

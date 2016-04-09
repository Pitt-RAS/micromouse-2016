#include <Arduino.h>

#include <EEPROM.h>
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
  "TURN",
  "CHK",
  "OPT"
};

char* secondary_options[] = {
  "CLR",
  "SDIR",
  "SPDS",
  "BACK"
};

char* direction_options[] = {
  "NRTH",
  "EAST"
};

char* speed_options[] = {
  "SVEL",
  "SACC",
  "SDEC",
  "K FV",
  "K TV",
  "KACC",
  "KDEC",
  "BACK"
};

void loop()
{
  switch (menu.getString(primary_options, 5, 4)) {
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
        int absolute_start_direction, absolute_end_direction;
        int relative_end_direction,

        ContinuousRobotDriverRefactor maze_load_driver;
        Maze<16, 16> maze;
        maze_load_driver.loadState(maze);
        FloodFillPath<16, 16> flood_path (maze, 0, 0, 8, 8);
        KnownPath<16, 16> known_path (maze, 0, 0, 8, 8, flood_path);
        PathParser parser (&known_path);
        KaosDriver driver;

        menu.waitForHand();
        speedRunMelody();

        absolute_start_direction = driver.getDirIMadeThisPublic();
        relative_end_direction = parser.getEndDirection();
        absolute_end_direction = (int) absolute_start_direction
                                        + (int) relative_end_direction;
        if (absolute_end_direction > 7)
          absolute_end_direction -= 8;

        driver.execute(parser.getMoveList());

        searchFinishMelody();

        ContinuousRobotDriverRefactor return_driver(parser.end_x, parser.end_y,
                                            (Compass8) absolute_end_direction);
        return_driver.loadState(maze);
        FloodFillPath<16, 16> return_path (maze, 8, 8, 0, 0);
        KnownPath<16, 16> return_best_path (maze, 8, 8, 0, 0, return_path);
        return_driver.move(return_best_path);
      }
      break;
    }
    case 2: { // TURN (goes out of the start cell, turns around, and comes back)
      menu.waitForHand();
      playNote(2000, 0.2);
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
      break;
    }
    case 3: { // CHK (checks if entire path has been discovered)
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
    case 4: { // OPT
      switch (menu.getString(secondary_options, 4, 4)) {
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
        case 2: { // SPDS
          while (1) {
            bool back = false;
            switch (menu.getString(speed_options, 8, 4)) {
              case 0: { // SEARCH VELOCITY
                uint16_t result = (uint16_t)EEPROM.read(EEPROM_SEARCH_VEL_LOCATION) << 8;
                result |= EEPROM.read(EEPROM_SEARCH_VEL_LOCATION + 1);
                result = menu.getInt(0, 9999, result, 4);
                EEPROM.write(EEPROM_SEARCH_VEL_LOCATION, result >> 8);
                EEPROM.write(EEPROM_SEARCH_VEL_LOCATION + 1, result & 0xFF);
                break;
              }
              case 1: { // SEARCH FORWARD ACCEL
                uint16_t result = (uint16_t)EEPROM.read(EEPROM_SEARCH_ACCEL_LOCATION) << 8;
                result |= EEPROM.read(EEPROM_SEARCH_ACCEL_LOCATION + 1);
                result = menu.getInt(0, 9999, result, 4);
                EEPROM.write(EEPROM_SEARCH_ACCEL_LOCATION, result >> 8);
                EEPROM.write(EEPROM_SEARCH_ACCEL_LOCATION + 1, result & 0xFF);
                break;
              }
              case 2: { // SEARCH DECEL
                uint16_t result = (uint16_t)EEPROM.read(EEPROM_SEARCH_DECEL_LOCATION) << 8;
                result |= EEPROM.read(EEPROM_SEARCH_DECEL_LOCATION + 1);
                result = menu.getInt(0, 9999, result, 4);
                EEPROM.write(EEPROM_SEARCH_DECEL_LOCATION, result >> 8);
                EEPROM.write(EEPROM_SEARCH_DECEL_LOCATION + 1, result & 0xFF);
                break;
              }
              case 3: { // KAOS FORWARD VELOCITY
                uint16_t result = (uint16_t)EEPROM.read(EEPROM_KAOS_FORWARD_VEL_LOCATION) << 8;
                result |= EEPROM.read(EEPROM_KAOS_FORWARD_VEL_LOCATION + 1);
                result = menu.getInt(0, 9999, result, 4);
                EEPROM.write(EEPROM_KAOS_FORWARD_VEL_LOCATION, result >> 8);
                EEPROM.write(EEPROM_KAOS_FORWARD_VEL_LOCATION + 1, result & 0xFF);
                break;
              }
              case 4: { // KAOS TURN VELOCITY
                uint16_t result = (uint16_t)EEPROM.read(EEPROM_KAOS_TURN_VEL_LOCATION) << 8;
                result |= EEPROM.read(EEPROM_KAOS_TURN_VEL_LOCATION + 1);
                result = menu.getInt(0, 9999, result, 4);
                EEPROM.write(EEPROM_KAOS_TURN_VEL_LOCATION, result >> 8);
                EEPROM.write(EEPROM_KAOS_TURN_VEL_LOCATION + 1, result & 0xFF);
                break;
              }
              case 5: { // KAOS FORWARD ACCEL
                uint16_t result = (uint16_t)EEPROM.read(EEPROM_KAOS_ACCEL_LOCATION) << 8;
                result |= EEPROM.read(EEPROM_KAOS_ACCEL_LOCATION + 1);
                result = menu.getInt(0, 9999, result, 4);
                EEPROM.write(EEPROM_KAOS_ACCEL_LOCATION, result >> 8);
                EEPROM.write(EEPROM_KAOS_ACCEL_LOCATION + 1, result & 0xFF);
                break;
              }
              case 6: { // KAOS DECEL
                uint16_t result = (uint16_t)EEPROM.read(EEPROM_KAOS_DECEL_LOCATION) << 8;
                result |= EEPROM.read(EEPROM_KAOS_DECEL_LOCATION + 1);
                result = menu.getInt(0, 9999, result, 4);
                EEPROM.write(EEPROM_KAOS_DECEL_LOCATION, result >> 8);
                EEPROM.write(EEPROM_KAOS_DECEL_LOCATION + 1, result & 0xFF);
                break;
              }
              case 7: { // BACK
                back = true;
                break;
              }
            }
            if (back) break;
          }
          break;
        }
        case 3: // BACK
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

#include <Arduino.h>

#include <LedDisplay.h>
#include "conf.h"
#include "data.h"
#include "driver.h"
#include "Logger.h"
#include "Navigator.h"
#include "Menu.h"
#include "motion.h"
#include "PersistantStorage.h"
#include "RangeSensorContainer.h"
#include "motors.h"
#include "sensors_encoders.h"
#include "Orientation.h"
#include "EncoderMod.h"
#include "utility.h"
#include "IdealSweptTurns.h"
#include <I2Cdev.h>
#include <MPU9150.h>

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

  menu.begin();
  Turnable::setDefaultInitialDirection(PersistantStorage::getDefaultDirection());
}

const char* primary_options[] = {
  "RUN",
  "KAOS",
  "TURN",
  "CHK",
  "OPT"
};

const char* secondary_options[] = {
  "CLR",
  "SDIR",
  "SPDS",
  "TRGT",
  "BACK"
};

const char* direction_options[] = {
  "NRTH",
  "EAST"
};

const char* speed_options[] = {
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

const char* cell_options[] = {
  "X",
  "Y",
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

      navigator.findBox(PersistantStorage::getTargetXLocation(),
                        PersistantStorage::getTargetYLocation());
      searchFinishMelody();
      navigator.findBox(0, 0);
      stopMelody();
      break;
    }
    case 1: { // KAOS
      uint8_t target_x = PersistantStorage::getTargetXLocation();
      uint8_t target_y = PersistantStorage::getTargetYLocation();
      if (knowsBestPath(target_x, target_y)) {
        Compass8 absolute_end_direction;

        ContinuousRobotDriverRefactor maze_load_driver;
        Maze<16, 16> maze;
        maze_load_driver.loadState(maze);
        FloodFillPath<16, 16> flood_path (maze, 0, 0, target_x, target_y);
        KnownPath<16, 16> known_path (maze, 0, 0, target_x, target_y, flood_path);
        PathParser parser (&known_path);
        KaosDriver driver;

        menu.waitForHand();
        speedRunMelody();

        absolute_end_direction = parser.getEndDirection();

        driver.execute(parser.getMoveList());
        char buf[5];

        snprintf(buf, 5, "%02d%02d", parser.end_x, parser.end_y);
        menu.showString(buf, 4);
        searchFinishMelody();

        ContinuousRobotDriverRefactor other_driver(parser.end_x, parser.end_y, absolute_end_direction, false);

        {
          FloodFillPath<16, 16>
            flood_path(maze, other_driver.getX(), other_driver.getY(), 0, 0);

          KnownPath<16, 16>
            known_path(maze, other_driver.getX(), other_driver.getY(), 0, 0, flood_path);

          if (known_path.isEmpty())
            break;

          other_driver.move(&known_path);

            snprintf(buf, 5, "%02d%02d", other_driver.getX(), other_driver.getY());
            menu.showString(buf, 4);

        }

        other_driver.move(kNorth, 0);
        //ContinuousRobotDriverRefactor return_driver(parser.end_x, parser.end_y,
        //                                    absolute_end_direction);
        //return_driver.loadState(maze);
        //FloodFillPath<16, 16> return_path (maze, 8, 8, 0, 0);
        //KnownPath<16, 16> return_best_path (maze, 8, 8, 0, 0, return_path);
        //return_driver.move(&return_best_path);
      }
      break;
    }
    case 2: { // TURN (goes out of the start cell, turns around, and comes back)
      menu.waitForHand();
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
      break;
    }
    case 3: { // CHK (checks if entire path has been discovered)
      uint8_t target_x = PersistantStorage::getTargetXLocation();
      uint8_t target_y = PersistantStorage::getTargetYLocation();
      if (knowsBestPath(target_x, target_y)) {
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
          break;
        }
        case 2: { // SPDS
          while (1) {
            bool back = false;
            switch (menu.getString(speed_options, 8, 4)) {
              case 0: { // SEARCH VELOCITY
                uint16_t result = PersistantStorage::getRawSearchVelocity();
                result = menu.getInt(0, 9999, result, 4);
                PersistantStorage::setRawSearchVelocity(result);
                break;
              }
              case 1: { // SEARCH FORWARD ACCEL
                uint16_t result = PersistantStorage::getRawSearchAccel();
                result = menu.getInt(0, 9999, result, 4);
                PersistantStorage::setRawSearchAccel(result);
                break;
              }
              case 2: { // SEARCH DECEL
                uint16_t result = PersistantStorage::getRawSearchDecel();
                result = menu.getInt(0, 9999, result, 4);
                PersistantStorage::setRawSearchDecel(result);
                break;
              }
              case 3: { // KAOS FORWARD VELOCITY
                uint16_t result = PersistantStorage::getRawKaosForwardVelocity();
                result = menu.getInt(0, 9999, result, 4);
                PersistantStorage::setRawKaosForwardVelocity(result);
                break;
              }
              case 4: { // KAOS DIAGONAL VELOCITY
                uint16_t result = PersistantStorage::getRawKaosDiagVelocity();
                result = menu.getInt(0, 9999, result, 4);
                PersistantStorage::setRawKaosDiagVelocity(result);
                break;
              }
              case 5: { // KAOS TURN VELOCITY
                uint16_t result = PersistantStorage::getRawKaosTurnVelocity();
                result = menu.getInt(0, 9999, result, 4);
                PersistantStorage::setRawKaosTurnVelocity(result);
                break;
              }
              case 6: { // KAOS FORWARD ACCEL
                uint16_t result = PersistantStorage::getRawKaosAccel();
                result = menu.getInt(0, 9999, result, 4);
                PersistantStorage::setRawKaosAccel(result);
                break;
              }
              case 7: { // KAOS DECEL
                uint16_t result = PersistantStorage::getRawKaosDecel();
                result = menu.getInt(0, 9999, result, 4);
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
          break;
        }
        case 3: { // TRGT
          while (1) {
            bool back = false;
            switch (menu.getString(cell_options, 8, 4)) {
              case 0: { // X LOCATION
                uint8_t result = PersistantStorage::getTargetXLocation();
                result = menu.getInt(0, 15, result, 4);
                PersistantStorage::setTargetXLocation(result);
                break;
              }
              case 1: { // Y LOCATION
                uint8_t result = PersistantStorage::getTargetYLocation();
                result = menu.getInt(0, 15, result, 4);
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
          break;
        }
        case 4: // BACK
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

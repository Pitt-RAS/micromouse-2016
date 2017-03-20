#include <Arduino.h>

// External libraries
#include <I2CdevPittMicromouse.h>
#include <LedDisplay.h>
#include <MPU9150PittMicromouse.h>
#include <EncoderPittMicromouse.h>

// Dependencies within Micromouse
#include "device/Motor.h"
#include "device/Orientation.h"
#include "device/PersistantStorage.h"
#include "device/sensors_encoders.h"
#include "device/RangeSensorContainer.h"
#include "legacy_motion/motion.h"
#include "user_interaction/PlayMelodies.h"
#include "user_interaction/Log.h"
#include "user_interaction/Logger.h"
#include "user_interaction/Menu.h"
#include "user_interaction/UserInterface.h"
#include "Navigator.h"
#include "conf.h"
#include "data.h"
#include "driver.h"
#include "parser.h"
#include "knows_best_path.h"

#define PATCH_VER_MESSAGE "Pitt Micromouse patched library version mismatch"
static_assert(PITT_MICROMOUSE_I2CDEV_PATCH_VERSION == 1, PATCH_VER_MESSAGE);
static_assert(PITT_MICROMOUSE_MPU9150_PATCH_VERSION == 1, PATCH_VER_MESSAGE);
static_assert(PITT_MICROMOUSE_ENCODER_PATCH_VERSION == 1, PATCH_VER_MESSAGE);

static_assert(F_CPU == 144000000, "Clock speed is not set to 144 MHz");

static void autoMode();
static void autoRun();
static void autoKaos(uint8_t);
static void mazeClear();
static void run();
static void kaos();
static void turn();
static void check();
static void options();
static void clear();
static void startDirection();
static void speeds();
static void targetCell();

void micromouse_main()
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

  Turnable::setDefaultInitialDirection(PersistantStorage::getDefaultDirection());

  LOG_INIT();

  Orientation::getInstance().resetHeading();
    
  MenuItem items[] = {
        { "AUTO", autoMode },
        { "CLR", clear },
        { "MZCLR", mazeClear },
        {}
    };
    /*MenuItem items[] = {
        { "RUN", run },
        { "KAOS", kaos },
        { "TURN", turn },
        { "CHK", check },
        { "OPT", options },
        {}
      };*/

  Menu menu(items, false);

  for (;;) menu.run();
}

void autoMode(){
    uint8_t target_x = PersistantStorage::getTargetXLocation();
    uint8_t target_y = PersistantStorage::getTargetYLocation();
    uint8_t state = PersistantStorage::getState();
    uint8_t runs = PersistantStorage::getNumRuns();
    
    while (runs > 0) {
        state = PersistantStorage::getState();
        runs = PersistantStorage::getNumRuns();

        if (state == 0){
            if (runs == 5 || !knowsBestPath(target_x, target_y)){
                autoRun();
            }
            else if (runs<4){
                //increase speed if run!=4
                autoKaos(state);
            }
            else{
                autoKaos(state);
            }
        }
        else if (state == 1)
        {
            if (runs == 4 || !knowsBestPath(target_x, target_y)){
                autoRun();
            }
            else{
                //reduce speeds back first
                autoKaos(state);
            }
            PersistantStorage::setState(0);
        }
        else {}

        runs--;
        PersistantStorage::setNumRuns(runs);
        delay(1000);
    }
}

void autoRun() {
    Navigator<ContinuousRobotDriver> navigator;
    Orientation& orientation = Orientation::getInstance();

    gUserInterface.waitForHand();
    startMelody();

    enc_left_front_write(0);
    enc_right_front_write(0);
    enc_left_back_write(0);
    enc_right_back_write(0);
    orientation.resetHeading();

    navigator.findBox(PersistantStorage::getTargetXLocation(),
                    PersistantStorage::getTargetYLocation());
                    
    navigator.findBox(0, 0);
    
    motion_rotate(180.0);
}

void autoKaos(uint8_t state) {
    uint8_t target_x = PersistantStorage::getTargetXLocation();
    uint8_t target_y = PersistantStorage::getTargetYLocation();

    Orientation& orientation = Orientation::getInstance();

    if (knowsBestPath(target_x, target_y)){
        ContinuousRobotDriver maze_load_driver;
        Maze<16, 16> maze;
        maze_load_driver.loadState(maze);
        FloodFillPath<16, 16> flood_path (maze, 0, 0, target_x, target_y);
        KnownPath<16, 16> known_path (maze, 0, 0, target_x, target_y, flood_path);
        PathParser parser (&known_path);
        KaosDriver driver;
        
        if (state != 0){
            gUserInterface.waitForHand();
        }
        
        enc_left_front_write(0);
        enc_right_front_write(0);
        enc_left_back_write(0);
        enc_right_back_write(0);
        orientation.resetHeading();

        Compass8 start_dir = maze_load_driver.getDirIMadeThisPublic();
        Compass8 delta_dir = parser.getTotalRotation();
        Compass8 end_dir = (Compass8)(((int)start_dir + (int)delta_dir) % 8);

        driver.execute(parser.getMoveList());
        char buf[5];

        snprintf(buf, 5, "%02d%02d", parser.end_x, parser.end_y);
        gUserInterface.showString(buf, 4);

        ContinuousRobotDriver other_driver(parser.end_x, parser.end_y, end_dir, false);

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

        FloodFillPath<16, 16> path(maze, 0, 0, 0, 0);
        other_driver.move(path);
    }
    motion_rotate(180.0);
}

void run()
{
  Navigator<ContinuousRobotDriver> navigator;
  Orientation& orientation = Orientation::getInstance();

  gUserInterface.waitForHand();
  startMelody();

  enc_left_front_write(0);
  enc_right_front_write(0);
  enc_left_back_write(0);
  enc_right_back_write(0);
  orientation.resetHeading();

  navigator.findBox(PersistantStorage::getTargetXLocation(),
                    PersistantStorage::getTargetYLocation());
  searchFinishMelody();
  navigator.findBox(0, 0);
  stopMelody();
}

void kaos()
{
  uint8_t target_x = PersistantStorage::getTargetXLocation();
  uint8_t target_y = PersistantStorage::getTargetYLocation();

  Orientation& orientation = Orientation::getInstance();

  if (knowsBestPath(target_x, target_y)) {
    ContinuousRobotDriver maze_load_driver;
    Maze<16, 16> maze;
    maze_load_driver.loadState(maze);
    FloodFillPath<16, 16> flood_path (maze, 0, 0, target_x, target_y);
    KnownPath<16, 16> known_path (maze, 0, 0, target_x, target_y, flood_path);
    PathParser parser (&known_path);
    KaosDriver driver;

    gUserInterface.waitForHand();
    speedRunMelody();

    enc_left_front_write(0);
    enc_right_front_write(0);
    enc_left_back_write(0);
    enc_right_back_write(0);
    orientation.resetHeading();

    Compass8 start_dir = maze_load_driver.getDirIMadeThisPublic();
    Compass8 delta_dir = parser.getTotalRotation();
    Compass8 end_dir = (Compass8)(((int)start_dir + (int)delta_dir) % 8);

    driver.execute(parser.getMoveList());
    char buf[5];

    snprintf(buf, 5, "%02d%02d", parser.end_x, parser.end_y);
    gUserInterface.showString(buf, 4);
    searchFinishMelody();

    ContinuousRobotDriver other_driver(parser.end_x, parser.end_y, end_dir, false);

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

    FloodFillPath<16, 16> path(maze, 0, 0, 0, 0);
    other_driver.move(path);
  }
}

void turn()
{
  gUserInterface.waitForHand();
  playNote(2000, 200);
  delay(1000);

  enc_left_front_write(0);
  enc_right_front_write(0);
  enc_left_back_write(0);
  enc_right_back_write(0);
  Orientation::getInstance().resetHeading();

  motion_forward(180, 0, 0);
  motion_rotate(180);
  motion_forward(180, 0, 0);
  motion_hold(100);
}

void check()
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

void options()
{
  MenuItem items[] = {
    { "CLR", clear },
    { "SDIR", startDirection },
    { "SPDS", speeds },
    { "TRGT", targetCell },
    {}
  };

  Menu(items).run();
}

void clear(){
  RobotDriver driver;
  driver.clearState();
  
  PersistantStorage::setNumRuns(NUM_RUNS);
  PersistantStorage::setState(0);
}

void mazeClear(){
  RobotDriver driver;
  driver.clearState();
}

void startDirection()
{
  MenuItem items[] = {
    {
      "NRTH", menuFunction {
        Turnable::setDefaultInitialDirection(kNorth);
        PersistantStorage::setDefaultDirection(kNorth);
      }
    },
    {
      "EAST", menuFunction {
        Turnable::setDefaultInitialDirection(kEast);
        PersistantStorage::setDefaultDirection(kEast);
      }
    },
    {}
  };

  Menu(items, false).run();
}

void speeds()
{
  MenuItem items[] = {
    {
      "SVEL", menuFunction {
        uint16_t result = PersistantStorage::getRawSearchVelocity();
        result = gUserInterface.getInt(0, 9999, result, 4);
        PersistantStorage::setRawSearchVelocity(result);
      }
    },
    {
      "SACC", menuFunction {
        uint16_t result = PersistantStorage::getRawSearchAccel();
        result = gUserInterface.getInt(0, 9999, result, 4);
        PersistantStorage::setRawSearchAccel(result);
      }
    },
    {
      "SDEC", menuFunction {
        uint16_t result = PersistantStorage::getRawSearchDecel();
        result = gUserInterface.getInt(0, 9999, result, 4);
        PersistantStorage::setRawSearchDecel(result);
      }
    },
    {
      "K FV", menuFunction {
        uint16_t result = PersistantStorage::getRawKaosForwardVelocity();
        result = gUserInterface.getInt(0, 9999, result, 4);
        PersistantStorage::setRawKaosForwardVelocity(result);
      }
    },
    {
      "K DV", menuFunction {
        uint16_t result = PersistantStorage::getRawKaosDiagVelocity();
        result = gUserInterface.getInt(0, 9999, result, 4);
        PersistantStorage::setRawKaosDiagVelocity(result);
      }
    },
    {
      "K TV", menuFunction {
        uint16_t result = PersistantStorage::getRawKaosTurnVelocity();
        result = gUserInterface.getInt(0, 9999, result, 4);
        PersistantStorage::setRawKaosTurnVelocity(result);
      }
    },
    {
      "KACC", menuFunction {
        uint16_t result = PersistantStorage::getRawKaosAccel();
        result = gUserInterface.getInt(0, 9999, result, 4);
        PersistantStorage::setRawKaosAccel(result);
      }
    },
    {
      "KDEC", menuFunction {
        uint16_t result = PersistantStorage::getRawKaosDecel();
        result = gUserInterface.getInt(0, 9999, result, 4);
        PersistantStorage::setRawKaosDecel(result);
      }
    },
    {}
  };

  Menu menu(items);
  while (menu.run());
}

void targetCell()
{
  MenuItem items[] = {
    {
      "X", menuFunction {
        uint8_t result = PersistantStorage::getTargetXLocation();
        result = gUserInterface.getInt(0, 15, result, 4);
        PersistantStorage::setTargetXLocation(result);
      }
    },
    {
      "Y", menuFunction {
        uint8_t result = PersistantStorage::getTargetYLocation();
        result = gUserInterface.getInt(0, 15, result, 4);
        PersistantStorage::setTargetYLocation(result);
      }
    },
    {}
  };

  Menu menu(items);
  while (menu.run());
}

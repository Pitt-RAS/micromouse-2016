#ifndef MICROMOUSE_PERSISTANT_STORAGE_H_
#define MICROMOUSE_PERSISTANT_STORAGE_H_

#include <Arduino.h>

// Dependencies within Micromouse
#include "data.h"

class PersistantStorage {
  private:
    // Loads an int from EEPROM, assuming the low byte of the int is
    // immediately after the high byte
    static int loadIntFromLocation(uint16_t high_byte_location);

    static void writeIntToLocation(uint16_t n, uint16_t high_byte_location);

  public:
    // Writes the current state to persistent memory
    static void saveMaze(Maze<16, 16>& maze);

    // Loads the saved state into the given maze object
    static void loadSavedMaze(Maze<16, 16>& maze);

    // Saves the state of the given cell only
    static void updateSavedMaze(Maze<16, 16>& maze, size_t x, size_t y);

    // Removes the saved state
    static void clearSavedMaze();

    // Sets the saved maze to an empty maze
    static void resetSavedMaze();

    // True if there is a maze stored in memory
    static bool hasSavedMaze();

    static Compass8 getDefaultDirection();
    static void setDefaultDirection(Compass8 dir);

    static uint8_t getTargetXLocation();
    static void setTargetXLocation(uint8_t x);
    static uint8_t getTargetYLocation();
    static void setTargetYLocation(uint8_t y);

    // The following provide access to all of the velocity and acceration settings
    static float getSearchVelocity();
    static void setSearchVelocity(float velocity);
    static float getSearchAccel();
    static void setSearchAccel(float accel);
    static float getSearchDecel();
    static void setSearchDecel(float decel);
    static float getKaosForwardVelocity();
    static void setKaosForwardVelocity(float velocity);
    static float getKaosDiagVelocity();
    static void setKaosDiagVelocity(float velocity);
    static float getKaosTurnVelocity();
    static void setKaosTurnVelocity(float velocity);
    static float getKaosAccel();
    static void setKaosAccel(float accel);
    static float getKaosDecel();
    static void setKaosDecel(float decel);

    static uint16_t getRawSearchVelocity();
    static void setRawSearchVelocity(uint16_t velocity);
    static uint16_t getRawSearchAccel();
    static void setRawSearchAccel(uint16_t accel);
    static uint16_t getRawSearchDecel();
    static void setRawSearchDecel(uint16_t decel);
    static uint16_t getRawKaosForwardVelocity();
    static void setRawKaosForwardVelocity(uint16_t velocity);
    static uint16_t getRawKaosDiagVelocity();
    static void setRawKaosDiagVelocity(uint16_t velocity);
    static uint16_t getRawKaosTurnVelocity();
    static void setRawKaosTurnVelocity(uint16_t velocity);
    static uint16_t getRawKaosAccel();
    static void setRawKaosAccel(uint16_t accel);
    static uint16_t getRawKaosDecel();
    static void setRawKaosDecel(uint16_t decel);

    static void setReadyForKaosFlag(uint8_t ready_for_kaos);
    static uint8_t getReadyForKaosFlag();
};

#endif

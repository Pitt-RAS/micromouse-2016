#include <EEPROM.h>
#include "../conf.h"
#include "PersistantStorage.h"

int PersistantStorage::loadIntFromLocation(uint16_t high_byte_location) {
  uint16_t result = (uint16_t)EEPROM.read(high_byte_location) << 8;
  result |= EEPROM.read(high_byte_location + 1);
  return result;
}

void PersistantStorage::writeIntToLocation(uint16_t n, uint16_t high_byte_location) {
  EEPROM.write(high_byte_location, n >> 8);
  EEPROM.write(high_byte_location + 1, n & 0xFF);
}

void PersistantStorage::saveMaze(Maze<16, 16>& maze) {
  for (int x = 0; x < 16; x++) {
    for (int y = 0; y < 16; y++) {
      uint8_t out = 0;
      out |= maze.isWall(x, y, kNorth) << 0;
      out |= maze.isWall(x, y, kEast) << 1;
      out |= maze.isWall(x, y, kSouth) << 2;
      out |= maze.isWall(x, y, kWest) << 3;
      out |= maze.isVisited(x, y) << 4;
      EEPROM.write(EEPROM_MAZE_LOCATION + 16*x + y, out);
    }
  }
  EEPROM.write(EEPROM_MAZE_FLAG_LOCATION, 1);
}

void PersistantStorage::loadSavedMaze(Maze<16, 16>& maze) {
  for (int x = 0; x < 16; x++) {
    for (int y = 0; y < 16; y++) {
      uint8_t in = EEPROM.read(EEPROM_MAZE_LOCATION + 16*x + y);

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
}

void PersistantStorage::updateSavedMaze(Maze<16, 16>& maze, size_t x, size_t y) {
  uint8_t out = 0;
  out |= maze.isWall(x, y, kNorth) << 0;
  out |= maze.isWall(x, y, kEast) << 1;
  out |= maze.isWall(x, y, kSouth) << 2;
  out |= maze.isWall(x, y, kWest) << 3;
  out |= maze.isVisited(x, y) << 4;
  EEPROM.write(EEPROM_MAZE_LOCATION + 16*x + y, out);
}

void PersistantStorage::clearSavedMaze() {
  Maze<16, 16> maze;
  saveMaze(maze);
  EEPROM.write(EEPROM_MAZE_FLAG_LOCATION, 0);
}

void PersistantStorage::resetSavedMaze() {
  Maze<16, 16> maze;
  saveMaze(maze);
}

bool PersistantStorage::hasSavedMaze() {
  return EEPROM.read(EEPROM_MAZE_FLAG_LOCATION) != 0;
}

Compass8 PersistantStorage::getDefaultDirection() {
  return (Compass8) EEPROM.read(EEPROM_INITIAL_DIRECTION_LOCATION);
}

void PersistantStorage::setDefaultDirection(Compass8 dir) {
  EEPROM.write(EEPROM_INITIAL_DIRECTION_LOCATION, (uint8_t)dir);
}

uint8_t PersistantStorage::getTargetXLocation() {
  return EEPROM.read(EEPROM_TARGET_X_LOCATION);
}

void PersistantStorage::setTargetXLocation(uint8_t x) {
  if (x < 16) {
    EEPROM.write(EEPROM_TARGET_X_LOCATION, x);
  }
}

uint8_t PersistantStorage::getTargetYLocation() {
  return EEPROM.read(EEPROM_TARGET_Y_LOCATION);
}

void PersistantStorage::setTargetYLocation(uint8_t y) {
  if (y < 16) {
    EEPROM.write(EEPROM_TARGET_Y_LOCATION, y);
  }
}

float PersistantStorage::getSearchVelocity() {
  return getRawSearchVelocity() / pow(10, PERSISTANT_STORAGE_VELOCITY_DIGITS);
}

void PersistantStorage::setSearchVelocity(float velocity) {
  setRawSearchVelocity(velocity * pow(10, PERSISTANT_STORAGE_VELOCITY_DIGITS));
}

float PersistantStorage::getSearchAccel() {
  return getRawSearchAccel() / pow(10, PERSISTANT_STORAGE_ACCEL_DIGITS);
}

void PersistantStorage::setSearchAccel(float accel) {
  setRawSearchAccel(accel * pow(10, PERSISTANT_STORAGE_ACCEL_DIGITS));
}

float PersistantStorage::getSearchDecel() {
  return getRawSearchDecel() / pow(10, PERSISTANT_STORAGE_ACCEL_DIGITS);
}

void PersistantStorage::setSearchDecel(float decel) {
  setRawSearchDecel(decel * pow(10, PERSISTANT_STORAGE_ACCEL_DIGITS));
}

float PersistantStorage::getKaosForwardVelocity() {
  return getRawKaosForwardVelocity() / pow(10, PERSISTANT_STORAGE_VELOCITY_DIGITS);
}

void PersistantStorage::setKaosForwardVelocity(float velocity) {
  setRawKaosForwardVelocity(velocity * pow(10, PERSISTANT_STORAGE_VELOCITY_DIGITS));
}

float PersistantStorage::getKaosDiagVelocity() {
  return getRawKaosDiagVelocity() / pow(10, PERSISTANT_STORAGE_VELOCITY_DIGITS);
}

void PersistantStorage::setKaosDiagVelocity(float velocity) {
  setRawKaosDiagVelocity(velocity * pow(10, PERSISTANT_STORAGE_VELOCITY_DIGITS));
}

float PersistantStorage::getKaosTurnVelocity() {
  return getRawKaosTurnVelocity() / pow(10, PERSISTANT_STORAGE_VELOCITY_DIGITS);
}

void PersistantStorage::setKaosTurnVelocity(float velocity) {
  setRawKaosTurnVelocity(velocity * pow(10, PERSISTANT_STORAGE_VELOCITY_DIGITS));
}

float PersistantStorage::getKaosAccel() {
  return getRawKaosAccel() / pow(10, PERSISTANT_STORAGE_ACCEL_DIGITS);
}

void PersistantStorage::setKaosAccel(float accel) {
  setRawKaosAccel(accel * pow(10, PERSISTANT_STORAGE_ACCEL_DIGITS));
}

float PersistantStorage::getKaosDecel() {
  return getRawKaosDecel() / pow(10, PERSISTANT_STORAGE_ACCEL_DIGITS);
}

void PersistantStorage::setKaosDecel(float decel) {
  setRawKaosDecel(decel * pow(10, PERSISTANT_STORAGE_ACCEL_DIGITS));
}

uint16_t PersistantStorage::getRawSearchVelocity() {
  return loadIntFromLocation(EEPROM_SEARCH_VEL_LOCATION);
}

void PersistantStorage::setRawSearchVelocity(uint16_t velocity) {
  writeIntToLocation(velocity, EEPROM_SEARCH_VEL_LOCATION);
}

uint16_t PersistantStorage::getRawSearchAccel() {
  return loadIntFromLocation(EEPROM_SEARCH_ACCEL_LOCATION);
}

void PersistantStorage::setRawSearchAccel(uint16_t accel) {
  writeIntToLocation(accel, EEPROM_SEARCH_ACCEL_LOCATION);
}

uint16_t PersistantStorage::getRawSearchDecel() {
  return loadIntFromLocation(EEPROM_SEARCH_DECEL_LOCATION);
}

void PersistantStorage::setRawSearchDecel(uint16_t decel) {
  writeIntToLocation(decel, EEPROM_SEARCH_DECEL_LOCATION);
}

uint16_t PersistantStorage::getRawKaosForwardVelocity() {
  return loadIntFromLocation(EEPROM_KAOS_FORWARD_VEL_LOCATION);
}

void PersistantStorage::setRawKaosForwardVelocity(uint16_t velocity) {
  writeIntToLocation(velocity, EEPROM_KAOS_FORWARD_VEL_LOCATION);
}

uint16_t PersistantStorage::getRawKaosDiagVelocity() {
  return loadIntFromLocation(EEPROM_KAOS_DIAG_VEL_LOCATION);
}

void PersistantStorage::setRawKaosDiagVelocity(uint16_t velocity) {
  writeIntToLocation(velocity, EEPROM_KAOS_DIAG_VEL_LOCATION);
}

uint16_t PersistantStorage::getRawKaosTurnVelocity() {
  return loadIntFromLocation(EEPROM_KAOS_TURN_VEL_LOCATION);
}

void PersistantStorage::setRawKaosTurnVelocity(uint16_t velocity) {
  writeIntToLocation(velocity, EEPROM_KAOS_TURN_VEL_LOCATION);
}

uint16_t PersistantStorage::getRawKaosAccel() {
  return loadIntFromLocation(EEPROM_KAOS_ACCEL_LOCATION);
}

void PersistantStorage::setRawKaosAccel(uint16_t accel) {
  writeIntToLocation(accel, EEPROM_KAOS_ACCEL_LOCATION);
}

uint16_t PersistantStorage::getRawKaosDecel() {
  return loadIntFromLocation(EEPROM_KAOS_DECEL_LOCATION);
}

void PersistantStorage::setRawKaosDecel(uint16_t decel) {
  writeIntToLocation(decel, EEPROM_KAOS_DECEL_LOCATION);
}

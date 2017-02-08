#include <Arduino.h>
#include "driver.h"
#include "utility.h"

bool knowsBestPath(size_t target_x, size_t target_y) {
  ContinuousRobotDriver driver;
  Maze<16, 16> maze;
  bool success = true;
  if (driver.hasStoredState()) {
    driver.loadState(maze);
    FloodFillPath<16, 16> flood_path1 (maze, 0, 0, target_x, target_y);
    FloodFillPath<16, 16> flood_path2 (maze, 0, 0, target_x, target_y);
    KnownPath<16, 16> known_path (maze, 0, 0, target_x, target_y, flood_path1);

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

#ifndef MICROMOUSE_MENU_H
#define MICROMOUSE_MENU_H

#include "UserInterface.h"

class Menu
{
  public:
    Menu();

    // Runs the main menu
    void main();

  private:
    // Does a discovery run.
    void run();

    // Does a kaos mode run.
    void kaos();

    // Goes out of the start cell, turns around, comes back.
    void turn();

    // Checks if the entire path has been discovered.
    void check();

    // Configures options.
    void options();

    // Clears the maze memory.
    void clear();

    // Sets the start direction.
    void startDirection();

    // Sets speeds.
    void speeds();

    // Sets the target cell.
    void targetCell();
};

#endif

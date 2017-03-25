#include <Arduino.h>
#include "../device/Motor.h"
#include "../device/PersistantStorage.h"
#include "PlayMelodies.h"
#include "UserInterface.h"
#include "Logger.h"
#include "Menu.h"
#include "FreakOut.h"

void freakOut(const char* msg) {
  motor_lf.Set(0, 0);
  motor_lb.Set(0, 0);
  motor_rf.Set(0, 0);
  motor_rb.Set(0, 0);
  
  uint8_t runs = PersistantStorage::getNumRuns();
  
  runs--;
  
  PersistantStorage::setNumRuns(runs);
  PersistantStorage::setState(1);
  
  gUserInterface.showString(msg);
  crashMelody();
  
  while (!gUserInterface.buttonOkPressed()) {
    // wait for button press before dumping logs
  }
  delay(500);
  logger.dump();

  
  while (1) {}
}

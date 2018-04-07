#include <Arduino.h>

#include "Logger.h"
#include "Menu.h"
#include "PlayMelodies.h"
#include "motors.h"

void freakOut(char* msg) {
  motor_lf.Set(0, 0);
  motor_lb.Set(0, 0);
  motor_rf.Set(0, 0);
  motor_rb.Set(0, 0);
  menu.showString(msg);
  crashMelody();
  while (!menu.buttonOkPressed()) {
    // wait for button press before dumping logs
  }
  delay(500);
//  logger.dump();
  while (1) {}
}

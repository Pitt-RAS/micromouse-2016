#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

#include "../conf.h"
#include "../data.h"

class Logger {
 private:
  Queue<float, LOG_SIZE> forward_accel_log_;
  Queue<float, LOG_SIZE> radial_accel_log_;
  Queue<float, LOG_SIZE> gyro_log_;
  Queue<float, LOG_SIZE> heading_log_;
  Queue<char, LOG_SIZE> motion_type_log_;
  Queue<float, LOG_SIZE> primary_pid_log_;
  Queue<float, LOG_SIZE> secondary_pid_log_;
  Queue<float, LOG_SIZE> tertiary_pid_log_;

  size_t cycles_;
 public:
  Logger();

  void logForwardAccel(float value);
  void logRadialAccel(float value);
  void logGyro(float value);
  void logHeading(float value);
  void logMotionType(char value);
  void logPrimaryPID(float value);
  void logSecondaryPID(float value);
  void logTertiaryPID(float value);

  void nextCycle();

  // writes out the contents of this logger
  void dump();
};

extern Logger logger;

#endif

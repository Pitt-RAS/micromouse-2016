#include "Logger.h"

Logger::Logger() : cycles_(0) {}

void Logger::logForwardAccel(float value) {
  forward_accel_log_.enqueue(value);
}

void Logger::logRadialAccel(float value) {
  radial_accel_log_.enqueue(value);
}

void Logger::logGyro(float value) {
  gyro_log_.enqueue(value);
}

void Logger::logHeading(float value) {
  heading_log_.enqueue(value);
}

void Logger::logMotionType(char value) {
  motion_type_log_.enqueue(value);
}

void Logger::logPrimaryPID(char value) {
  primary_pid_log_.enqueue(value);
}

void Logger::logSecondaryPID(char value) {
  secondary_pid_log_.enqueue(value);
}

void Logger::logTertiaryPID(char value) {
  tertiary_pid_log_.enqueue(value);
}

void Logger::nextCycle() {
  cycles_++;

  if (gyro_log_.size() < cycles_) {
    gyro_log_.enqueue(0);
  }

  if (accel_log_.size() < cycles_) {
    accel_log_.enqueue(0);
  }

  if (motion_type_log_.size() < cycles_) {
    motion_type_log_.enqueue(' ');
  }

  if (primary_pid_log_.size() < cycles_) {
    primary_pid_log_.enqueue(0);
  }

  if (secondary_pid_log_.size() < cycles_) {
    secondary_pid_log_.enqueue(0);
  }

  if (tertiary_pid_log_.size() < cycles_) {
    tertiary_pid_log_.enqueue(0);
  }
}

void Logger::dump() {
  nextCycle();
  cycles_--;
  while (cycles_ > 0) {
    Serial.print(motion_type_log_.dequeue());
    Serial.print("\t");
    Serial.print(gyro_log_.dequeue());
    Serial.print("\t");
    Serial.print(accel_log_.dequeue());
    Serial.print("\t");
    Serial.print(primary_pid_log_.dequeue());
    Serial.print("\t");
    Serial.print(secondary_pid_log_.dequeue());
    Serial.print("\t");
    Serial.print(primary_pid_log_.dequeue());
    Serial.println();
  }
}

Logger logger;

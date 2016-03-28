#ifndef LOGGER_H
#define LOGGER_H

#include "conf.h"

template<typename T, size_t capacity>
class LogQueue {
 private:
  T buffer_[capacity];
  size_t head_;
  size_t tail_;
  size_t size_;
 public:
  LogQueue();
  void enqueue(T value);
  T dequeue();
  size_t size();
};

template<typename T, size_t capacity>
LogQueue<T, capacity>::LogQueue() : head_(0), tail_(0), size_(0) {
  memset(buffer_, 0, sizeof(buffer_));
}

template<typename T, size_t capacity>
void LogQueue<T, capacity>::enqueue(T value) {
  buffer_[tail_] = value;
  tail_ = (tail_ + 1) % capacity;

  if (size_ < capacity) {
    size_++;
  } else {
    head_ = (head_ + 1) % capacity;
  }
}

template<typename T, size_t capacity>
T LogQueue<T, capacity>::dequeue() {
  T result = buffer_[head_];
  head_ = (head_ + 1) % capacity;
  size_--;
  return result;
}

class Logger {
 private:
  LogQueue<float, LOG_SIZE> gyro_log_;
  LogQueue<float, LOG_SIZE> accel_log_;
  LogQueue<char, LOG_SIZE> motion_type_log_;
  LogQueue<float, LOG_SIZE> primary_pid_log_;
  LogQueue<float, LOG_SIZE> secondary_pid_log_;
  LogQueue<float, LOG_SIZE> tertiary_pid_log_;

  size_t cycles_;
 public:
  Logger();

  void logGyro(float value);
  void logAccel(float value);
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

#ifndef LOGGER_H
#define LOGGER_H

template<typename T, size_t capacity>
class Logger {
 private:
  T buffer_[capacity];
  size_t head_;
  size_t tail_;
  size_t size_;
 public:
  Logger();

  // adds value to the log
  void log(T value);

  // writes out the contents of this logger
  void dump();
};

template<typename T, size_t capacity>
Logger<T, capacity>::Logger() : head_(0), tail_(0), size_(0) {
  memset(buffer_, 0, sizeof(buffer_));
}

template<typename T, size_t capacity>
void Logger<T, capacity>::log(T value) {
  buffer_[tail_] = value;
  tail_ = (tail_ + 1) % capacity;

  if (size_ < capacity) {
    size_++;
  } else {
    head_ = (head_ + 1) % capacity;
  }
}

template<typename T, size_t capacity>
void Logger<T, capacity>::dump() {
  while (size_ > 0) {
    Serial.println(buffer_[head_]);
    head_ = (head_ + 1) % capacity;
    size_--;
  }
}

#endif

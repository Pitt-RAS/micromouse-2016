#ifndef LOG_H
#define LOG_H

#define LOG_ENABLED true

#define LOG_INIT_DELAY 0

// Bluetooth
//#define LOG_SERIAL Serial1
//#define LOG_BAUD 115200

// USB
#define LOG_SERIAL Serial
#define LOG_BAUD 9600

#if LOG_ENABLED
  #define LOG_INIT() \
    gUserInterface.showString("INIT"); \
    delay(LOG_INIT_DELAY); \
    LOG_SERIAL.begin(LOG_BAUD)

  // Very simple logging macro for convenience. Use it like printf().
  #define LOG(...) LOG_SERIAL.printf(__VA_ARGS__)
#else
  #define LOG_INIT()
  #define LOG(...)
#endif

#endif

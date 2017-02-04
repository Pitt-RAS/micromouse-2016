#ifndef FAST_TRIGS_H
#define FAST_TRIGS_H

#include <cstdlib>

// Class that provides fast trig functions using a lookup table
// 
// Currently only has sin and cos

class FastTrigs {
  public:
    static float sin(float x);
    static float cos(float x);
  private:
    FastTrigs();

    static FastTrigs& inst();

    static const constexpr size_t kTableSize = 500;
    float table_[kTableSize];
};

#endif // include guard

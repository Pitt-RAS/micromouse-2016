#include <cmath>

#include "FastTrigs.h"

float FastTrigs::sin(float x) {
  if (x > 0) x = std::fmod(x, 2*M_PI);
  if (x < 0) x = std::fmod(x, 2*M_PI) + 2*M_PI;

  if (2*x <= M_PI) {
    float target_index = 2*x / M_PI * (kTableSize - 1);
    size_t left_index = target_index;

    if (left_index >= kTableSize) return inst().table_[kTableSize - 1];

    // Interpolate between table values
    return inst().table_[left_index]
         + (inst().table_[left_index + 1] - inst().table_[left_index])
           * (target_index - left_index);

  } else if (x < M_PI) {
    return FastTrigs::sin(M_PI - x);
  } else {
    return -FastTrigs::sin(2*M_PI - x);
  }
}

FastTrigs& FastTrigs::inst() {
  static FastTrigs inst;
  return inst;
}

float FastTrigs::cos(float x) {
  return FastTrigs::sin(M_PI / 2 - x);
}

FastTrigs::FastTrigs() {
  for (size_t i = 0; i < kTableSize; i++) {
    table_[i] = std::sin((M_PI / 2) * (float(i) / (kTableSize - 1)));
  }
}

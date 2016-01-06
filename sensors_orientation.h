#ifndef MICROMOUSE_SENSORS_ORIENTATION_H_
#define MICROMOUSE_SENSORS_ORIENTATION_H_

#include "MPU6050_6Axis_MotionApps20.h"

class Orientation {
  private:
    Orientation();
    static void interruptHandler();

    static volatile bool mpu_interrupt_;

    Orientation* instance_ = NULL;
    MPU6050 mpu_;
    bool dmp_ready_ = false;
    uint16_t packet_size_;
    uint16_t fifo_count_ = 0;

    float raw_heading_;
    float heading_offset_ = 0;

    float max_forward_accel_ = 0;
    float max_radial_accel_ = 0;
  public:
    Orientation* getInstance();

    void update();

    void resetHeading();
    float getHeading();
    
    void resetMaxForwardAccel();
    void resetMaxRadialAccel();
    float getMaxForwardAccel();
    float getMaxRadialAccel();
};

#endif

#ifndef MICROMOUSE_SENSORS_ORIENTATION_H_
#define MICROMOUSE_SENSORS_ORIENTATION_H_

#include "MPU6050_6Axis_MotionApps20.h"

class Orientation {
  private:
    Orientation();
    static void interruptHandler();

    static volatile bool mpu_interrupt_;

    static Orientation* instance_;

    MPU6050 mpu_;
    bool dmp_ready_ = false;
    uint16_t packet_size_;
    uint16_t fifo_count_ = 0;

    float raw_heading_ = NAN;
    float heading_offset_ = 0;
    int completed_rotations_ = 0;

    float max_forward_accel_ = 0;
    float max_radial_accel_ = 0;
  public:
    static Orientation* getInstance();

    // get the latest data from the IMU
    void update();

    // designates the current heading as 0 degrees
    void resetHeading();

    // Returns the current heading in degrees
    // Clockwise is positive
    // This does not wrap around, so it will continue increasing past 360
    //   or decreasing past -360
    float getHeading();
    
    void resetMaxForwardAccel();
    void resetMaxRadialAccel();
    float getMaxForwardAccel();
    float getMaxRadialAccel();
};

#endif

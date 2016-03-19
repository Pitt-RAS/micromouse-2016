#ifndef MICROMOUSE_SENSORS_ORIENTATION_H_
#define MICROMOUSE_SENSORS_ORIENTATION_H_

#include <Arduino.h>

#include <helper_3dmath.h>
#define MPU6050_INCLUDE_DMP_MOTIONAPPS20
#include <MPU6050.h>

class Orientation {
  private:
    Orientation();
    static void interruptHandler();

    static volatile bool mpu_interrupt_;

    static Orientation* instance_;

    MPU6050 mpu_;
    uint16_t packet_size_ = 2;
    uint16_t fifo_count_ = 0;

    float secondary_gyro_offset_ = 0;

    float raw_heading_ = 0;
    int16_t last_gyro_reading_ = 0;
    unsigned long last_update_time_ = 0;
    volatile unsigned long next_update_time_ = 0;
    
    float max_forward_accel_ = 0;
    float max_radial_accel_ = 0;
  public:
    static Orientation* getInstance();

    // determine the right offset for the gyro
    void calibrate();

    // get the latest data from the IMU
    bool update();

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

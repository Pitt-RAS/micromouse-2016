#include "sensors_orientation.h"
#include "conf.h"

#include <Arduino.h>

#include <I2Cdev.h>

#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>

volatile bool Orientation::mpu_interrupt_ = false;
Orientation* Orientation::instance_ = NULL;

Orientation::Orientation() {
  Wire.begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);

  mpu_.initialize();
  mpu_.testConnection();
  Serial2.println(mpu_.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  uint8_t dev_status = mpu_.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu_.setXGyroOffset(0);
  mpu_.setYGyroOffset(-60);
  mpu_.setZGyroOffset(-24);
  mpu_.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (dev_status == 0) {
    // turn on the DMP, now that it's ready
    Serial2.println(F("Enabling DMP..."));
    mpu_.setDMPEnabled(true);

    // enable interrupt detection
    Serial2.println(F("Enabling MPU6050 interrupt detection..."));
    pinMode(IMU_INTERRUPT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), interruptHandler, RISING);

    // set our DMP Ready flag so the main update() function knows it's okay to use it
    Serial2.println(F("DMP ready! Waiting for first interrupt..."));
    dmp_ready_ = true;

    // get expected DMP packet size for later comparison
    packet_size_ = mpu_.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial2.print(F("DMP Initialization failed (code "));
    Serial2.print(dev_status);
    Serial2.println(F(")"));
  }
}

void Orientation::interruptHandler() {
  mpu_interrupt_ = true;
}

Orientation* Orientation::getInstance() {
  if (instance_ == NULL) {
    mpu_interrupt_ = false;
    instance_ = new Orientation();
  }
  return instance_;
}

void Orientation::update() {
  if (!dmp_ready_) return;

  if (!mpu_interrupt_ && fifo_count_ < packet_size_) {
    return;
  }

  uint8_t mpu_int_status;
  uint8_t fifo_buffer[64];
  Quaternion orientation;
  VectorFloat gravity;
  float tait_bryan[3];
  VectorInt16 accel, world_accel;
  float cos_heading, sin_heading;
  float curr_forward_accel, curr_radial_accel;

  // reset interrupt flag and get INT_STATUS byte
  mpu_interrupt_ = false;
  mpu_int_status = mpu_.getIntStatus();

  // get current FIFO count
  fifo_count_ = mpu_.getFIFOCount();

  // check for FIFO overflow
  if ((mpu_int_status & 0x10) || fifo_count_ == 1024) {
    // reset so we can continue cleanly
    mpu_.resetFIFO();
    Serial2.println(F("FIFO overflow!"));

  // check for DMP data ready interrupt
  } else if (mpu_int_status & 0x01) {
    // wait for correct available data length
    while (fifo_count_ < packet_size_) {
      fifo_count_ = mpu_.getFIFOCount();
    }

    // read a packet from FIFO
    mpu_.getFIFOBytes(fifo_buffer, packet_size_);
    
    // track FIFO count here in case there is > 1 packet available
    fifo_count_ -= packet_size_;

    mpu_.dmpGetQuaternion(&orientation, fifo_buffer);
    mpu_.dmpGetGravity(&gravity, &orientation);
    mpu_.dmpGetYawPitchRoll(tait_bryan, &orientation, &gravity);

    if (raw_heading_ == NAN) {
      heading_offset_ = -tait_bryan[0];
    }

    if (raw_heading_ < -PI/2 && tait_bryan[0] > PI/2) {
      completed_rotations_--;
    } else if (raw_heading_ > PI/2 && tait_bryan[0] < -PI/2) {
      completed_rotations_++;
    }
    raw_heading_ = tait_bryan[0];

    mpu_.dmpGetAccel(&accel, fifo_buffer);
    mpu_.dmpGetLinearAccel(&accel, &accel, &gravity);
    mpu_.dmpGetLinearAccelInWorld(&world_accel, &accel, &orientation);

    cos_heading = cos(raw_heading_ * M_PI / 180.0);
    sin_heading = sin(raw_heading_ * M_PI / 180.0);
    curr_forward_accel = cos_heading * world_accel.x + sin_heading * world_accel.y;
    curr_forward_accel = abs(curr_forward_accel) * 9.8 / 8192; // convert to m/s
    curr_radial_accel = -sin_heading * world_accel.x + cos_heading * world_accel.y;
    curr_radial_accel = abs(curr_radial_accel) * 9.8 / 8192; // convert to m/s

    max_forward_accel_ = max(max_forward_accel_, curr_forward_accel);
    max_radial_accel_ = max(max_radial_accel_, curr_radial_accel);
  }
    //Serial2.println(orientation->getHeading());
    //Serial2.println(buf[i - 2]);
}

void Orientation::resetHeading() {
  heading_offset_ = -raw_heading_;
  completed_rotations_ = 0;
}

float Orientation::getHeading() {
  return -(raw_heading_ + heading_offset_) * RAD_TO_DEG - 360 * completed_rotations_;
}

void Orientation::resetMaxForwardAccel() {
  max_forward_accel_ = 0;
}

void Orientation::resetMaxRadialAccel() {
  max_radial_accel_ = 0;
}

float Orientation::getMaxForwardAccel() {
  return max_forward_accel_;
}

float Orientation::getMaxRadialAccel() {
  return max_radial_accel_;
}


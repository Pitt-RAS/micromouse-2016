#include "sensors_orientation.h"
#include "conf.h"

#include <Arduino.h>

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>

volatile bool Orientation::mpu_interrupt_ = false;
Orientation* Orientation::instance_ = NULL;

Orientation::Orientation() {
#ifdef CORE_TEENSY
  Wire1.begin(I2C_MASTER, 0, I2C_PINS_29_30, I2C_PULLUP_EXT, I2C_RATE_400);
#else
  Wire.begin();
  TWBR = 24;
#endif

  mpu_.initialize();
  Serial.println(mpu_.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  mpu_.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu_.setZGyroOffset(GYRO_OFFSET_SETTING);
  mpu_.setZAccelOffset(1788); // 1688 factory default for my test chip

  // enable interrupt detection
  Serial.println(F("Enabling MPU6050 interrupt detection..."));
  pinMode(IMU_INTERRUPT_PIN, INPUT);

#ifdef CORE_TEENSY
  attachInterrupt(IMU_INTERRUPT_PIN, interruptHandler, RISING);
#else
  attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), interruptHandler, RISING);
#endif

  mpu_.setRate(1); // Sample rate of 500Hz (assuming LPF is enabled)
  mpu_.setDLPFMode(MPU6050_DLPF_BW_188); // Enable DLPF
  mpu_.setFIFOEnabled(true);
  mpu_.setZGyroFIFOEnabled(true);

  mpu_.setIntFIFOBufferOverflowEnabled(true);
  mpu_.setIntDataReadyEnabled(true);
}

void Orientation::interruptHandler() {
  mpu_interrupt_ = true;
  if (Orientation::instance_ != NULL) {
    Orientation::instance_->next_update_time_ = micros();
  }
}

Orientation* Orientation::getInstance() {
  if (instance_ == NULL) {
    mpu_interrupt_ = false;
    instance_ = new Orientation();
  }
  return instance_;
}

void Orientation::calibrate() {
  Serial.println("Calibrating gyro...");
  mpu_.setZGyroOffset(0);
  int32_t total = 0;
  int16_t offset = 0;

  for (int round = 0; round < GYRO_CALIBRATION_ROUNDS; round++) {
    total = 0;

    for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
      while (!update()) {
        // wait for actual data
      }

      if (i > 0 && abs(total / i - last_gyro_reading_) > 100) {
        Serial.println("Calibration failed, restarting calibration...");
        calibrate();
        return;
      }

      total += last_gyro_reading_;
    }

    offset += total / GYRO_CALIBRATION_SAMPLES;
    mpu_.setZGyroOffset(-offset);
  }
  Serial.println(offset);

  total = 0;
  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    while (!update()) {
      // wait for actual data
    }

    if (i > 0 && abs(total / i - last_gyro_reading_) > 100) {
      Serial.println("Calibration failed, restarting calibration...");
      calibrate();
      return;
    }

    total += last_gyro_reading_;
  }

  secondary_gyro_offset_ = (float)total / GYRO_CALIBRATION_SAMPLES;
  Serial.println(secondary_gyro_offset_, 10);

  raw_heading_ = 0;
  Serial.println("Gyro calibration successful");
}

bool Orientation::update() {
  if (!mpu_interrupt_ && fifo_count_ < packet_size_) {
    return false;
  }

  uint8_t mpu_int_status;
  uint8_t fifo_buffer[64];

  // reset interrupt flag and get INT_STATUS byte
  mpu_interrupt_ = false;
  mpu_int_status = mpu_.getIntStatus();

  // get current FIFO count
  fifo_count_ = mpu_.getFIFOCount();

  // check for FIFO overflow
  if ((mpu_int_status & 0x10) || fifo_count_ == 1024) {
    // reset so we can continue cleanly
    mpu_.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return false;

  // check for data ready interrupt
  } else if (mpu_int_status & 0x01) {
    // wait for correct available data length
    while (fifo_count_ < packet_size_) {
      fifo_count_ = mpu_.getFIFOCount();
    }

    // read a packet from FIFO
    mpu_.getFIFOBytes(fifo_buffer, packet_size_);
    
    // track FIFO count here in case there is > 1 packet available
    fifo_count_ -= packet_size_;

    float dt = (next_update_time_ - last_update_time_) / 1000000.0;
    raw_heading_ += last_gyro_reading_ / GYRO_LSB_PER_DEG_PER_S * dt;

    last_gyro_reading_ = (uint16_t)fifo_buffer[0] << 8 | fifo_buffer[1];
    last_gyro_reading_ -= secondary_gyro_offset_;

    last_update_time_ = next_update_time_;

    return true;
  } else {
    return false;
  }
}

void Orientation::resetHeading() {
  raw_heading_ = 0;
  last_update_time_ = micros();
}

float Orientation::getHeading() {
  float elapsed_time = (micros() - last_update_time_) / 1000000.0;
  return raw_heading_ + last_gyro_reading_ * elapsed_time;
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


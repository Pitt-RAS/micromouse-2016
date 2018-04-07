#include "sensors_orientation.h"
#include "conf.h"
#include "FreakOut.h"
#include "Logger.h"

#include <Arduino.h>

#include <I2CdevPittMicromouse.h>
#include <MPU9150PittMicromouse.h>

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
  Serial.println(mpu_.testConnection() ? F("MPU9150PittMicromouse connection successful") : F("MPU9150 connection failed"));

  mpu_.setFullScaleGyroRange(MPU9150_GYRO_FS_2000);
  mpu_.setFullScaleAccelRange(MPU9150_ACCEL_FS_16);
  mpu_.setZGyroOffset(GYRO_OFFSET_SETTING);
  mpu_.setZAccelOffset(1788); // 1688 factory default for my test chip

  // enable interrupt detection
  Serial.println(F("Enabling MPU9150PittMicromouse interrupt detection..."));
  pinMode(IMU_INTERRUPT_PIN, INPUT);

#ifdef CORE_TEENSY
  attachInterrupt(IMU_INTERRUPT_PIN, interruptHandler, RISING);
#else
  attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), interruptHandler, RISING);
#endif

  mpu_.setRate(1); // Sample rate of 500Hz (assuming LPF is enabled)
  mpu_.setDLPFMode(MPU9150_DLPF_BW_188); // Enable DLPF
  mpu_.setFIFOEnabled(true);
  mpu_.setAccelFIFOEnabled(true);
  mpu_.setZGyroFIFOEnabled(true);

  mpu_.setIntFIFOBufferOverflowEnabled(true);
  mpu_.setIntDataReadyEnabled(true);

  // there's no good magnetometer interface in the MPU9150PittMicromouse library, so here it is

  // allow us to access the magnetometer over I2C
  //I2Cdev::writeByte(MPU9150_DEFAULT_ADDRESS, MPU9150_RA_INT_PIN_CFG, 0x02);
  //delay(10);

  //I2Cdev::writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01);
  //delay(10);
  //uint8_t mag_buf[4];
  //I2Cdev::readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 4, mag_buf);
  //int16_t mx = (((int16_t)mag_buf[0]) << 8) | mag_buf[1];
  //int16_t my = (((int16_t)mag_buf[2]) << 8) | mag_buf[3];
  //last_mag_heading_ = atan2(mx, my) * RAD_TO_DEG;
  //mag_heading_offset_ = -last_mag_heading_;
}

void Orientation::interruptHandler() {
  mpu_interrupt_ = true;
  if (Orientation::instance_ != NULL) {
    Orientation::instance_->next_update_time_ = micros();
    if (Orientation::instance_->handler_update_)
      Orientation::instance_->update();
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
  Serial.print("Primary offset: ");
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
  Serial.print("Secondary offset: ");
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
  float dt;
  uint16_t accel_x, accel_y;

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

    // Accelerometer update
    accel_x = (uint16_t)fifo_buffer[0] << 8 | fifo_buffer[1];
    accel_y = (uint16_t)fifo_buffer[2] << 8 | fifo_buffer[3];

    accel_x /= ACCEL_LSB_PER_G;
    accel_y /= ACCEL_LSB_PER_G;

    if (abs(accel_x) > FAILSAFE_ACCEL_THRESHOLD
            || abs(accel_y) > FAILSAFE_ACCEL_THRESHOLD) {
      //freakOut("OUCH");
    }

    if (abs(accel_x) > max_radial_accel_) {
      max_radial_accel_ = accel_x;
    }

    if (abs(accel_y) > max_forward_accel_) {
      max_forward_accel_ = accel_y;
    }

    // Gyro update
    dt = (next_update_time_ - last_update_time_) / 1000000.0;
    raw_heading_ -= last_gyro_reading_ / GYRO_LSB_PER_DEG_PER_S * dt;

    if (abs(last_gyro_reading_) > FAILSAFE_GYRO_THRESHOLD) {
      over_gyro_threshold_ = true;
      angle_past_gyro_threshold_ -= last_gyro_reading_ / GYRO_LSB_PER_DEG_PER_S * dt;
      if (abs(angle_past_gyro_threshold_) > FAILSAFE_GYRO_ANGLE) {
        //freakOut("FUCK");
      }
    } else {
      over_gyro_threshold_ = false;
      angle_past_gyro_threshold_ = 0;
    }

    last_gyro_reading_ = (uint16_t)fifo_buffer[6] << 8 | fifo_buffer[7];
    last_gyro_reading_ -= secondary_gyro_offset_;

    // Magnetometer update
    //updates_since_mag_reading_++;
    //if (updates_since_mag_reading_ == MAG_CYCLES_PER_UPDATE) {
    //  updates_since_mag_reading_ = 0;
    //  uint8_t mag_buf[4];
    //  I2Cdev::readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 4, mag_buf);
    //  int16_t mx = (((int16_t)mag_buf[0]) << 8) | mag_buf[1];
    //  int16_t my = (((int16_t)mag_buf[2]) << 8) | mag_buf[3];
    //  float mag_heading = atan2(mx, my) * RAD_TO_DEG + mag_heading_offset_;
    //  //Serial.print(mag_heading);
    //  //Serial.print("\t");
    //  // tell the magnetometer to take another reading
    //  //   so that it'll be ready by the time we get back around
    //  I2Cdev::writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01);
    //  delay(6);
    //  I2Cdev::readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 4, mag_buf);
    //  mx = (((int16_t)mag_buf[0]) << 8) | mag_buf[1];
    //  my = (((int16_t)mag_buf[2]) << 8) | mag_buf[3];

    //  mag_heading = atan2(mx, my) * RAD_TO_DEG + mag_heading_offset_;
    //  //Serial.println(mag_heading);

    //  // make sure mag_heading is in the same rotation as the current heading
    //  //
    //  // this is necessary because our output heading is allowed to wrap,
    //  //   but the magnetometer reading obviously won't
    //  while (mag_heading - raw_heading_ > 180) {
    //    mag_heading -= 360;
    //  }

    //  while (mag_heading - raw_heading_ < -180) {
    //    mag_heading += 360;
    //  }

    //  raw_heading_ = raw_heading_ * (1 - MAG_COMPLEMENTARY_FILTER)
    //                   + mag_heading * MAG_COMPLEMENTARY_FILTER;
    //}

    //logger.logForwardAccel(accel_y);
    //logger.logRadialAccel(accel_x);
    //logger.logGyro(last_gyro_reading_);
    //logger.logHeading(raw_heading_);

    last_update_time_ = next_update_time_;

    return true;
  } else {
    return false;
  }
}

void Orientation::resetHeading() {
  raw_heading_ = 0;
  last_update_time_ = micros();
  mag_heading_offset_ = -last_mag_heading_;
}

void Orientation::incrementHeading(float offset) {
  raw_heading_ += offset;
  mag_heading_offset_ -= offset;
  if (mag_heading_offset_ < -180) {
    mag_heading_offset_ += 360;
  } else if (mag_heading_offset_ > 180) {
    mag_heading_offset_ -= 360;
  }
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


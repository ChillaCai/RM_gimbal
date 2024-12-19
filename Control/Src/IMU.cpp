//
// Created by chill on 2024/11/17.
//

#include "../Inc/IMU.h"
#include "../Inc/BMI088.h"
#include <cmath>

#define g 9.81

float loop_rate = 0.001;
extern uint8_t rx_acc_data[6];
extern uint8_t rx_gyro_data[6];

uint8_t rx_acc_range_raw;
int rx_acc_range_k;
uint8_t rx_gyro_range_raw;
int rx_gyro_range_k;

void IMU::AccCalculate() {

//  uint8_t rx_acc_range_raw[2];
  BMI088_accel_read_reg(0x41, &rx_acc_range_raw, 1);
  rx_acc_range_k = (pow(2, (int) rx_acc_range_raw)) * 3;

  BMI088_accel_read_reg(0x12, rx_acc_data, 6);

  acc_x_ = (int16_t)((uint16_t)rx_acc_data[0] | (uint16_t) rx_acc_data[1] << 8) / 32768.f * rx_acc_range_k * g;
  acc_y_ = (int16_t)((uint16_t)rx_acc_data[2] | (uint16_t) rx_acc_data[3] << 8) / 32768.f * rx_acc_range_k * g;
  acc_z_ = (int16_t)((uint16_t)rx_acc_data[4] | (uint16_t) rx_acc_data[5] << 8) / 32768.f * rx_acc_range_k * g;
}

void IMU::GyroCalculate() {

//  uint8_t rx_gyro_range_raw;
  BMI088_gyro_read_reg(0x0F, &rx_gyro_range_raw, 1);
  rx_gyro_range_k = pow(2, (int) rx_gyro_range_raw) ;

  BMI088_gyro_read_reg(0x02, rx_gyro_data, 6);

  gyro_x_ = (int16_t)((uint16_t)rx_gyro_data[0] | (uint16_t) rx_gyro_data[1] << 8) / 32768.f * 2000.f / rx_gyro_range_k;
  gyro_y_ = (int16_t)((uint16_t)rx_gyro_data[2] | (uint16_t) rx_gyro_data[3] << 8) / 32768.f * 2000.f / rx_gyro_range_k;
  gyro_z_ = (int16_t)((uint16_t)rx_gyro_data[4] | (uint16_t) rx_gyro_data[5] << 8) / 32768.f * 2000.f / rx_gyro_range_k;
}

void IMU::IMUCalculate() {

  AccCalculate();
  GyroCalculate();

  float acc_y = 0.f;
  float acc_x = 0.f;

  roll_acc_ = atan2(acc_y, acc_z_) * 180. / 3.1415 ;
  pitch_acc_ = -atan2(acc_x, sqrt(acc_y_ * acc_y_ + acc_z_ * acc_z_)) * 180. / 3.1415;

  float roll_rad = roll_gyro_ * 3.1415 / 180;
  float pitch_rad = pitch_gyro_ * 3.1415 / 180;
  float yaw_rad = yaw_gyro_ * 3.1415 / 180;
  float dot_yaw_gyro = sin(roll_rad) / cos(pitch_rad) * gyro_y_ + cos(roll_rad) / cos(pitch_rad) * gyro_z_;
  float dot_roll_gyro = gyro_x_ + sin(pitch_rad) * sin(roll_rad) / cos(pitch_rad) * gyro_y_ + cos(roll_rad) * sin(pitch_rad) / cos(pitch_rad) * gyro_z_;
  float dot_pitch_gyro = cos(roll_rad) * gyro_y_ - sin(roll_rad) * gyro_z_;

  yaw_gyro_ = yaw_gyro_ + dot_yaw_gyro * loop_rate;
  roll_gyro_ = (roll_gyro_ + dot_roll_gyro * loop_rate) * (1 - K) + roll_acc_ * K;
  pitch_gyro_ = (pitch_gyro_ + dot_pitch_gyro * loop_rate) * (1 - K) + pitch_acc_ * K;
}

IMU::IMU(float k):K(k){
  yaw_gyro_ = 0.;
  pitch_gyro_ = 0.;
  roll_gyro_ =0.;
}

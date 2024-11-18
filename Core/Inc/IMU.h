//
// Created by chill on 2024/11/17.
//

#ifndef RM_GIMBAL_IMU_H
#define RM_GIMBAL_IMU_H

#include <cstdint>

class IMU {
private:
  float acc_x_;
  float acc_y_;
  float acc_z_;
  float gyro_x_;
  float gyro_y_;
  float gyro_z_;

  float yaw_gyro_;
  float pitch_gyro_;
  float roll_gyro_;

  float pitch_acc_;
  float roll_acc_;

  float yaw_;
  float pitch_;
  float roll_;

  float K;

public:
  // todo
  IMU(float k);
  void AccCalculate();
  void GyroCalculate();
  void IMUCalculate();

  float IMUYaw(){return yaw_;}
  float IMUPitch(){return pitch_;}
  float IMURoll(){return roll_;}
};

void BMI088_Init();

void BMI088_gyro_read_reg(uint8_t reg, uint8_t *return_data, uint8_t length);
void BMI088_accel_read_reg(uint8_t reg, uint8_t *return_data, uint8_t length);
void BMI088_accel_write_single_reg(uint8_t reg, uint8_t data);


#endif // RM_GIMBAL_IMU_H

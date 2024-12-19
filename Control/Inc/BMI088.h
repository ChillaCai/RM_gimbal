//
// Created by chill on 2024/12/18.
//

#ifndef RM_GIMBAL_BMI088_H
#define RM_GIMBAL_BMI088_H

#include <cstdint>

void BMI088_Init();

void BMI088_gyro_read_reg(uint8_t reg, uint8_t *return_data, uint8_t length);
void BMI088_accel_read_reg(uint8_t reg, uint8_t *return_data, uint8_t length);
void BMI088_accel_write_single_reg(uint8_t reg, uint8_t data);

#endif // RM_GIMBAL_BMI088_H

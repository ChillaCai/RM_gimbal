//
// Created by chill on 2024/11/17.
//

#include "IMU.h"
#include "spi.h"
#include "math.h"

#define g 9.81

float loop_rate = 0.001;
extern uint8_t rx_acc_data[6];
extern uint8_t rx_gyro_data[6];

// 片选
void BMI088_ACCEL_NS_L(void)
{
  HAL_GPIO_WritePin(ACC_GPIO_Port, ACC_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
  HAL_GPIO_WritePin(ACC_GPIO_Port, ACC_Pin,  GPIO_PIN_SET);
}
void BMI088_GYRO_NS_L(void)
{
  HAL_GPIO_WritePin(GYRO_GPIO_Port, GYRO_Pin,  GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
  HAL_GPIO_WritePin(GYRO_GPIO_Port, GYRO_Pin,  GPIO_PIN_SET);
}

void BMI088_write_byte(uint8_t txdata){
  HAL_SPI_Transmit(&hspi1, &txdata, 1, 1000);
  while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
}

void BMI088_read_byte(uint8_t *rxdata, uint8_t length){
  HAL_SPI_Receive(&hspi1, rxdata, length, 1000);
  while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
}

void BMI088_accel_write_single_reg(uint8_t reg, uint8_t data){

  BMI088_GYRO_NS_H();
  BMI088_ACCEL_NS_L();
  HAL_Delay(1);

  BMI088_write_byte(reg & 0x7F);
  BMI088_write_byte(data);

  HAL_Delay(1);
  BMI088_ACCEL_NS_H();
}

void BMI088_accel_read_reg(uint8_t reg, uint8_t *return_data, uint8_t length){

  BMI088_GYRO_NS_H();
  BMI088_ACCEL_NS_L();
  HAL_Delay(1);

  BMI088_write_byte(reg | 0x80);
  BMI088_read_byte(return_data, 1);
  BMI088_read_byte(return_data, length);

  HAL_Delay(1);
  BMI088_ACCEL_NS_H();
}

void BMI088_gyro_write_single_reg(uint8_t reg, uint8_t data){

  BMI088_ACCEL_NS_H();
  BMI088_GYRO_NS_L();
  HAL_Delay(1);

  BMI088_write_byte(reg & 0x7F);
  BMI088_write_byte(data);

  HAL_Delay(1);
  BMI088_GYRO_NS_H();
}

void BMI088_gyro_read_reg(uint8_t reg, uint8_t *return_data, uint8_t length){

  BMI088_ACCEL_NS_H();
  BMI088_GYRO_NS_L();
  HAL_Delay(1);

  BMI088_write_byte(reg | 0x80);
  BMI088_read_byte(return_data, length);

  HAL_Delay(1);
  BMI088_GYRO_NS_H();
}

void BMI088_write_reg(uint8_t reg, uint8_t data){
  BMI088_write_byte(reg & 0x7F);
  BMI088_write_byte(data);
}

void BMI088_Init(void) {
  // Soft Reset ACCEL
  BMI088_ACCEL_NS_L();
  BMI088_write_reg(0x7E, 0xB6); // Write 0xB6 to ACC_SOFTRESET(0x7E)
  HAL_Delay(1);
  BMI088_ACCEL_NS_H();

  // Soft Reset GYRO
  BMI088_GYRO_NS_L();
  BMI088_write_reg(0x14, 0xB6); // Write 0xB6 to GYRO_SOFTRESET(0x14)
  HAL_Delay(30);
  BMI088_GYRO_NS_H();

  // Switch ACCEL to Normal Mode
  BMI088_ACCEL_NS_L();
  HAL_Delay(1);
  BMI088_write_reg(0x7D, 0x04); // Write 0x04 to ACC_PWR_CTRL(0x7D)
  HAL_Delay(1);
  BMI088_ACCEL_NS_H();
}



void IMU::AccCalculate() {

  uint8_t rx_acc_range_raw[2];
  BMI088_accel_read_reg(0x41, rx_acc_range_raw, 2);
  int rx_acc_range_k = ((int) rx_acc_range_raw[1] + 1) * 3;

  BMI088_accel_read_reg(0x12, rx_acc_data, 6);

  acc_x_ = (int16_t)((uint16_t)rx_acc_data[0] | (uint16_t) rx_acc_data[1] << 8) / 65536.f * 2.f * rx_acc_range_k * g;
  acc_y_ = (int16_t)((uint16_t)rx_acc_data[2] | (uint16_t) rx_acc_data[3] << 8) / 65536.f * 2.f * rx_acc_range_k * g;
  acc_z_ = (int16_t)((uint16_t)rx_acc_data[4] | (uint16_t) rx_acc_data[5] << 8) / 65536.f * 2.f * rx_acc_range_k * g;
}

void IMU::GyroCalculate() {

  uint8_t rx_gyro_range_raw;
  BMI088_gyro_read_reg(0x0F, &rx_gyro_range_raw, 1);
  int rx_gyro_range_k = ((int) rx_gyro_range_raw + 1) * 2;

  BMI088_gyro_read_reg(0x02, rx_gyro_data, 6);

  gyro_x_ = (int16_t)((uint16_t)rx_gyro_data[0] | (uint16_t) rx_gyro_data[1] << 8) / 65536.f * 2.f * 4000.f / rx_gyro_range_k;
  gyro_y_ = (int16_t)((uint16_t)rx_gyro_data[2] | (uint16_t) rx_gyro_data[3] << 8) / 65536.f * 2.f * 4000.f / rx_gyro_range_k;
  gyro_z_ = (int16_t)((uint16_t)rx_gyro_data[4] | (uint16_t) rx_gyro_data[5] << 8) / 65536.f * 2.f * 4000.f / rx_gyro_range_k;
}

void IMU::IMUCalculate() {

  AccCalculate();
  GyroCalculate();

  roll_acc_ = atan2(acc_y_, acc_z_);
  pitch_acc_ = -atan2(acc_x_, sqrt(acc_y_*acc_y_ + acc_z_*acc_z_));

  float dot_yaw_gyro = sin(roll_)/cos(pitch_)*gyro_y_ + cos(roll_)/cos(pitch_)*gyro_z_;
  float dot_roll_gyro = gyro_x_ + sin(pitch_)*sin(roll_)/cos(pitch_)*gyro_y_ + cos(roll_)*sin(pitch_)/cos(pitch_)*gyro_z_;
  float dot_pitch_gyro = cos(roll_)*gyro_y_ - sin(roll_)*gyro_z_;

  // todo 此处用loop_rate好吗?
  yaw_gyro_ = yaw_gyro_ + dot_yaw_gyro * loop_rate;
  roll_gyro_ = roll_gyro_ + dot_roll_gyro * loop_rate;
  pitch_gyro_ = pitch_gyro_ + dot_pitch_gyro * loop_rate;

  yaw_ = yaw_gyro_;
  roll_ = pitch_ + (roll_acc_ - roll_gyro_) * K;
  pitch_ = pitch_acc_ + (pitch_acc_ - pitch_gyro_) * K;
}

IMU::IMU(float k):K(k){
  // todo: init 可以放在这个位置吗？
  BMI088_Init();

  yaw_gyro_ = 0.;
  pitch_gyro_ = 0.;
  roll_gyro_ =0.;
}

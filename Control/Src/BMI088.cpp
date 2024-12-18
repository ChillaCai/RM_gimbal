//
// Created by chill on 2024/12/18.
//

#include "BMI088.h"
#include "spi.h"
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
  //  HAL_Delay(1);

  BMI088_write_byte(reg | 0x80);
  BMI088_read_byte(return_data, 1);
  BMI088_read_byte(return_data, length);

  //  HAL_Delay(1);
  BMI088_ACCEL_NS_H();
}

void BMI088_gyro_write_single_reg(uint8_t reg, uint8_t data){

  BMI088_ACCEL_NS_H();
  BMI088_GYRO_NS_L();
  //  HAL_Delay(1);

  BMI088_write_byte(reg & 0x7F);
  BMI088_write_byte(data);

  //  HAL_Delay(1);
  BMI088_GYRO_NS_H();
}

void BMI088_gyro_read_reg(uint8_t reg, uint8_t *return_data, uint8_t length){

  BMI088_ACCEL_NS_H();
  BMI088_GYRO_NS_L();
  //  HAL_Delay(1);

  BMI088_write_byte(reg | 0x80);
  BMI088_read_byte(return_data, length);

  //  HAL_Delay(1);
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

  // Set ACCEL RANGE
  BMI088_ACCEL_NS_L();
  HAL_Delay(1);
  BMI088_write_reg(0x41, 0x02);
  HAL_Delay(1);
  BMI088_ACCEL_NS_H();

  // Set Gyro RANGE
  BMI088_GYRO_NS_L();
  HAL_Delay(1);
  BMI088_write_reg(0x0F, 0x01);
  HAL_Delay(1);
  BMI088_GYRO_NS_H();
}
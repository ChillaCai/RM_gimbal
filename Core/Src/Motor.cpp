//
// Created by chill on 2024/11/9.
//

#include "Motor.h"
#include <cstdint>

#define pi 3.14159

uint8_t tx_data[8];
int current_tx;

float LinearMappingInt2Float(int16_t in, int16_t in_mid, int16_t in_max, float out_mid, float out_max);

int LinearMappingFloat2Int(float in, float in_min, float in_max, int out_min, int out_max);

Motor::Motor(Motor::MotorType type, float ratio, float output_max, uint16_t StdID, uint8_t ID, const PID& pid_vel, const PID& pid_ang, bool is_imu_fdb, float ecd_angle)
    : motor_type_(type), ratio_(ratio), stdid_(StdID), id_(ID), output_max_(output_max),
      pid_vel_(pid_vel),
      pid_ang_(pid_ang), ecd_angle_(ecd_angle), is_imu_fdb_(is_imu_fdb)
{
  angle_ = 0.0f; 	        // deg 输出端累计转动角度
  delta_angle_ = 0.0f; 		// deg 输出端新转动的角度
//  ecd_angle_ = 0.0f; 		// deg 当前电机编码器角度
  last_ecd_angle_ = 0.0f;	// deg 上次电机编码器角度
  delta_ecd_angle_ = 0.0f; 	// deg 编码器端新转动的角度
  rotate_speed_ = 0.0f;         // dps 反馈转子转速
  torque_ = 0.0f;               // 反馈转矩
  temp_ = 0.0f;                 // 反馈电机温度

  ref_vel_ = 0.0f;              // 单环控制预期转速
  target_current_ = 0.0f;       // 目标输入电流

  angle_imu_ = 0.0f;

  stop_flag_ = false;
  forward_current_ = 0.0f;
  forward_voltage_ = 0.0f;
}

void Motor::CanRxMsgCallback(const uint8_t rx_data[8]){

  last_ecd_angle_ = ecd_angle_;
  int16_t ecd_angle = (int16_t)((uint16_t)rx_data[0] << 8) | rx_data[1];
  ecd_angle_ = LinearMappingInt2Float(ecd_angle, 0, 8191, 0.0, 360.0);

  float delta = ecd_angle_ - last_ecd_angle_;
  // 编码器变化角度过零点修正
  if (delta < -180.0) delta_ecd_angle_ = delta + 360.0;
  else if (delta > 180.0) delta_ecd_angle_ = delta - 360.0;
  else delta_ecd_angle_ = delta;

  int16_t rotate_speed_rpm = (int16_t)((uint16_t)rx_data[2] << 8) | rx_data[3];
  rotate_speed_ = rotate_speed_rpm * 6.0;

  delta_angle_ = delta_ecd_angle_ / ratio_;
  angle_ += delta_angle_;

  torque_ = (int16_t)((uint16_t)rx_data[4] << 8) | rx_data[5];

  if (motor_type_ == M3508 | motor_type_ == GM6020) temp_ = (int16_t)rx_data[6];

  CalculatePID();
}

void Motor::CalculatePID(){
  if (!stop_flag_){
    if(is_imu_fdb_){
      ref_vel_ = pid_ang_.calc(ref_ang_, angle_imu_);
    }
    else{
      ref_vel_ = pid_ang_.calc(ref_ang_, angle_);
    }
    target_current_ = pid_vel_.calc(ref_vel_, rotate_speed_);
  }
  else target_current_= 0.0;

  forward_current_ = 0.0f;
  forward_voltage_ = -0.0049 * angle_ + 0.0136;

  if(stdid_ == GM6020_StdID_CONTROL_VOLTAGE1 | stdid_ == GM6020_StdID_CONTROL_VOLTAGE2){
    current_tx = LinearMappingFloat2Int(target_current_ + forward_voltage_, -output_max_, output_max_, -25000, 25000);
  }
  else{
    current_tx = LinearMappingFloat2Int(target_current_ + forward_current_, -output_max_, output_max_, -16384, 16384);
  }
}

void Motor::Handle() {
  if(id_ < 0x05){
    tx_data[2 * id_ - 1] = current_tx;
    tx_data[2 * id_ - 2] = current_tx >> 8;
  }
  else{
    tx_data[2 * (id_ - 4) - 1] = current_tx;
    tx_data[2 * (id_ - 4) - 2] = current_tx >> 8;
  }

  CAN_TxHeaderTypeDef TxHeader = {(uint16_t)(stdid_), 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};

  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_data, CAN_FilterFIFO0);
}

void Motor::Stop(){ stop_flag_ = true;}

void Motor::RCControl(float channel_data) {
  stop_flag_ = false;
  ref_ang_ += channel_data;
}

void Motor::SetIMUAngle(float angle) {
  angle_imu_ = angle;
}

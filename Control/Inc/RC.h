//
// Created by chill on 2024/11/9.
//

#ifndef RM_GIMBAL_RC_H
#define RM_GIMBAL_RC_H

#include <cstdint>
#include "usart.h"

#define RC_RX_BUF_SIZE 18
#define RC_RX_DATA_SIZE 18

class RC {
private:
  uint8_t rx_buf_[RC_RX_BUF_SIZE];
  uint8_t rx_data_[RC_RX_DATA_SIZE];

public:
  enum RCSwitchState_e{
    UP=0, MID, DOWN
  };
  struct RCChannel{
    float r_row;
    float r_col;
    float l_row;
    float l_col;
  } channel_;
  struct RCSwitch {
    RCSwitchState_e l;
    RCSwitchState_e r;
  } switch_;

  RC();
  void ReadDbus();
  void ParseDbus();

  // gain = 目标角度变化 / 左侧摇杆row/col数据
  float yaw_gain_;
  float pitch_gain_;
};

#endif // RM_GIMBAL_RC_H

//
// Created by chill on 2024/11/9.
//

#include "iwdg.h"
#include "Controller.h"

void RCHandle();

CAN_RxHeaderTypeDef rx_header;

void ControlLoop(){
  HAL_IWDG_Refresh(&hiwdg);

  // 将rc的add_on量加给target
  RCHandle();
  // can发布消息
  pitch_motor.Handle();
//  yaw_motor.Handle();
}

void RCHandle(){
  // 右下关闭所有电机
//  if(rc.switch_.r == RC::DOWN){
//    yaw_motor.Stop();
//    pitch_motor.Stop();
//    shooter_motor1.Stop();
//    shooter_motor2.Stop();
//  }
  // 右中/上控制电机 左拨杆上下pitch，左右yaw
//  else{
    yaw_motor.RCControl(rc.yaw_gain_ * rc.channel_.l_row);
    pitch_motor.RCControl(rc.pitch_gain_ * rc.channel_.l_col);
//  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim == &htim6){
    ControlLoop();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if (huart == &huart3){
    rc.ReadDbus();
    rc.ParseDbus();
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
  if (hcan == &hcan1){
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(&hcan1, CAN_FilterFIFO0, &rx_header, rx_data);

    if (rx_header.StdId == yaw_motor_feedback_stdid) yaw_motor.CanRxMsgCallback(rx_data);
    else if (rx_header.StdId == pitch_motor_feedback_stdid) pitch_motor.CanRxMsgCallback(rx_data);
    else if (rx_header.StdId == shooter1_feedback_stdid) shooter_motor1.CanRxMsgCallback(rx_data);
    else if (rx_header.StdId == shooter2_feedback_stdid) shooter_motor2.CanRxMsgCallback(rx_data);
    else {}
  }
}


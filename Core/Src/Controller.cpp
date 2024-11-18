//
// Created by chill on 2024/11/9.
//

#include "iwdg.h"

#include "tim.h"
#include "RC.h"
#include "Motor.h"
#include "IMU.h"

// todo
#define YAW_MOTOR_ID 0x02
#define PITCH_MOTOR_ID 0x01
#define SHOOTER1_MOTOR_ID 0x01
#define SHOOTER2_MOTOR_ID 0x01

uint16_t yaw_motor_control_stdid = GM6020_StdID_CONTROL_VOLTAGE2;
uint16_t pitch_motor_control_stdid = GM6020_StdID_CONTROL_VOLTAGE1;
uint16_t shooter1_control_stdid = M3508_StdID_CONTROL1;
uint16_t shooter2_control_stdid = M3508_StdID_CONTROL2;

uint16_t yaw_motor_feedback_stdid = GM6020_StdID_FEEDBACK + YAW_MOTOR_ID;
uint16_t pitch_motor_feedback_stdid = GM6020_StdID_FEEDBACK + PITCH_MOTOR_ID;
uint16_t shooter1_feedback_stdid = M3508_StdID_FEEDBACK + SHOOTER1_MOTOR_ID;
uint16_t shooter2_feedback_stdid = M3508_StdID_FEEDBACK + SHOOTER2_MOTOR_ID;

IMU imu(0.4);
RC rc;
// todo
Motor yaw_motor(Motor::GM6020, GM6020_RATIO, GM6020_MAXCURRENT, yaw_motor_control_stdid, YAW_MOTOR_ID,
                PID(0,0,0,0,0,GM6020_MAXCURRENT),
                PID(0,0,0,0,0,0),0 , 0);
Motor pitch_motor(Motor::GM6020, GM6020_RATIO, GM6020_MAXCURRENT,pitch_motor_control_stdid,PITCH_MOTOR_ID,
                  PID(0.004,0,0,0,0,GM6020_MAXCURRENT),
                  PID(60,3,200,0.2,15,600),0 , 217.36174);
Motor shooter_motor1(Motor::M3508, M3508_RATIO, M3508_MAXCURRENT,shooter1_control_stdid,SHOOTER1_MOTOR_ID,
                     PID(0,0,0,0,0,M3508_MAXCURRENT),
                     PID(0,0,0,0,0,0),0 ,0);
Motor shooter_motor2(Motor::M3508, M3508_RATIO, M3508_MAXCURRENT,shooter2_control_stdid, SHOOTER2_MOTOR_ID,
                     PID(0,0,0,0,0,M3508_MAXCURRENT),
                     PID(0,0,0,0,0,0),0 , 0);

void RCHandle();
void IMUHandle();

CAN_RxHeaderTypeDef rx_header;

void ControlLoop(){
  HAL_IWDG_Refresh(&hiwdg);

  // 将rc的 add_on量加给 target
  RCHandle();
  // IMU解算 & 赋值给电机
  IMUHandle();
  // can 控制电机
  pitch_motor.Handle();
//  yaw_motor.Handle();
}

// todo
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
    // todo 若要用遥控器控制：软件限位
}

// todo
void IMUHandle(){
    imu.IMUCalculate();

    pitch_motor.SetIMUAngle(imu.IMUPitch());
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
  }
}


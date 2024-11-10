//
// Created by chill on 2024/11/9.
//

#ifndef RM_GIMBAL_CONTROLLER_H
#define RM_GIMBAL_CONTROLLER_H

#include "tim.h"
#include "RC.h"
#include "Motor.h"

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


RC rc;
// todo
Motor yaw_motor(Motor::GM6020, GM6020_RATIO, GM6020_MAXCURRENT, yaw_motor_control_stdid, YAW_MOTOR_ID,
                PID(0,0,0,0,GM6020_MAXCURRENT),
                PID(0,0,0,0,0));
Motor pitch_motor(Motor::GM6020, GM6020_RATIO, GM6020_MAXCURRENT,pitch_motor_control_stdid,PITCH_MOTOR_ID,
                  PID(0.007,0,0,0,GM6020_MAXCURRENT),
                  PID(0,0,0,0,0));
Motor shooter_motor1(Motor::M3508, M3508_RATIO, M3508_MAXCURRENT,shooter1_control_stdid,SHOOTER1_MOTOR_ID,
                     PID(0,0,0,0,M3508_MAXCURRENT),
                     PID(0,0,0,0,0));
Motor shooter_motor2(Motor::M3508, M3508_RATIO, M3508_MAXCURRENT,shooter2_control_stdid, SHOOTER2_MOTOR_ID,
                     PID(0,0,0,0,M3508_MAXCURRENT),
                     PID(0,0,0,0,0));


#endif // RM_GIMBAL_CONTROLLER_H

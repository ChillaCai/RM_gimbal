//
// Created by chill on 2024/11/9.
//

#ifndef RM_GIMBAL_PID_H
#define RM_GIMBAL_PID_H

#include <cstdint>

class PID {
public:
  PID(float kp, float ki, float kd, float kfilter_d, float i_max, float out_max);
  float calc(float ref, float fdb);
  float ref_, fdb_;
  float kfilter_d_;
  float kp_, ki_, kd_;
  float i_max_, out_max_;
private:
  float output_;
  float err_, err_sum_, last_err_;
  float pout_, iout_, dout_;
};

#endif // RM_GIMBAL_PID_H

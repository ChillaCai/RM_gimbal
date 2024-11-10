//
// Created by chill on 2024/11/9.
//

#include "PID.h"

float set_limit(float limit, float data);

PID::PID(float kp, float ki, float kd, float kfilter_d, float i_max, float out_max)
    :kp_(kp), ki_(ki), kd_(kd), kfilter_d_(kfilter_d), i_max_(i_max), out_max_(out_max)
{
  output_ = 0.0f;
  ref_ = 0.0f;
  fdb_ = 0.0f;
  err_ = 0.0f;
  err_sum_ = 0.0f;
  last_err_ = 0.0f;
  pout_ = 0.0f;
  iout_ = 0.0f;
  dout_ = 0.0f;
}

float PID::calc(float ref, float fdb)
{
  ref_ = ref;
  fdb_ = fdb;

  last_err_ = err_;
  err_ = ref_ - fdb_;
  err_sum_ = set_limit(i_max_, err_sum_ + err_);

  pout_ = kp_ * err_;
  iout_ = ki_ * err_sum_;
  dout_ = (1 - kfilter_d_) * kd_ * (err_ - last_err_) + kfilter_d_ * dout_;

  output_ = set_limit(out_max_, pout_ + iout_ + dout_);
  return  output_;
}
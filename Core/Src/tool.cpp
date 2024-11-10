//
// Created by chill on 2024/11/9.
//
#include <cstdint>

float LinearMappingInt2Float(int16_t in, int16_t in_mid, int16_t in_max, float out_mid, float out_max){
  if (in_mid == in_max){
    return out_mid;
  }
  else{
    float out = out_mid + (out_max - out_mid) / (in_max - in_mid) * (in - in_mid);
    return out;
  }
}

int LinearMappingFloat2Int(float in, float in_min, float in_max, int out_min, int out_max){
  if (out_min == out_max) return out_min;
  else return (int)(out_min + (out_max - out_min) * (in - in_min) / (in_max - in_min));
}

float set_limit(float limit, float data){
  if (data >= limit) data = limit;
  else if (data <= -limit) data = -limit;
  return data;
}
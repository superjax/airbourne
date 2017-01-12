#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "vector3.h"

class IMU
{
public:
  IMU();
  bool init(){}
  void read_all(vector3* accel, vector3* gyro, float* temp){}
  void request_update(volatile uint8_t *status){}

  void register_external_interrupt_callback(void (*CB)(void)){}
  void register_data_ready_callback(void (*CB)()){}
};

#endif // IMU_H

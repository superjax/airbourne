#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "vector3.h"

class IMU
{
public:
  IMU();
  virtual bool init() = 0;
  virtual void read_all(vector3* accel, vector3* gyro, float* temp) = 0;
  virtual void request_update(volatile uint8_t *status) = 0;

  virtual void register_external_interrupt_callback(void (*CB)(void)) = 0;
  virtual void register_data_ready_callback(void (*CB)()) = 0;
};

#endif // IMU_H

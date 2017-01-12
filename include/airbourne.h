#ifndef AIRBOURNE_H
#define AIRBOURNE_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "barometer.h"
#include "eeprom.h"
#include "gpio.h"
#include "hmc5883l.h"
#include "i2c.h"
#include "imu.h"
#include "led.h"
#include "magnetometer.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "printf.h"
#include "pwm.h"
#include "rc.h"
#include "rc_dsm.h"
#include "rc_ppm.h"
#include "serialport.h"
#include "uart.h"

#include "vector3.h"
#include "quaternion.h"
#include "turbotrig.h"

void airbourne_init();

extern LED LED0;
extern LED LED1;

#endif // AIRBOURNE_H

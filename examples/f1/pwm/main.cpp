/*
   ms5611read.c : read values from MS5611 barometer using I^2C

   Copyright (C) 2016 Simon D. Levy

   This file is part of BreezySTM32.

   BreezySTM32 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   BreezySTM32 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with BreezySTM32.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "printf.h"
#include "uart.h"
#include "pwm.h"

UART Serial1;

static void _putc(void *p, char c)
{
  (void)p; // avoid compiler warning about unused variable
  Serial1.put_byte((uint8_t*)&c, 1);
}

bool available = false;

int main(void)
{
  SystemInit();
  OSCinit();
  enableInterrupts();
  enablePeripherals();
  startWallClock();

  PWM_Out pwm_pins[6];
  PWM_Out rc_pins[8];
  for(int i = 0; i < 6; i ++)
  {
    pwm_pins[i].init(PWM_PORT, i + 1, 490, 2000, 1000);
  }
  for(int i = 0; i < 8; i++)
  {
    rc_pins[i].init(RC_PORT, i+1, 490, 2000, 1000);
  }

  float throttle = 0.0;
  while(1)
  {
    for(int i = 0; i < 6; i++)
    {
      pwm_pins[i].write(throttle);
    }
    for(int i = 0; i < 8; i++)
    {
      rc_pins[i].write(throttle);
    }
    delay_ms(1);
    throttle += 0.001;
    if(throttle > 1.0)
      throttle = 0.0;
  }
} 

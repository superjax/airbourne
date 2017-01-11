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
#include "i2c.h"
#include "ms5611.h"
#include "uart.h"

UART Serial1;
MS5611 baro;

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    Serial1.put_byte((uint8_t*)&c, 1);
}

bool available = false;

int main(void)
{
    enableInterrupts();
    enablePeripherals();
    startWallClock();

    I2C i2c1;
    i2c1.init(2);

    Serial1.init(1, 115200, UART::MODE_DMA_TX_RX);
    init_printf(NULL, _putc);


    available = baro.init(&i2c1);

    // Not sure why the ms5611 needs this, but without this line it doesn't work
    while(1)
    {
        float pressure;
        float temp;
        if (available)
        {
            baro.update();
            baro.read(&pressure, &temp);
            printf("Pressure: %d Pa    \t Temperature: %d.%d deg C\n",
                   (uint32_t)(pressure),
                   (uint32_t)temp,
                   (uint32_t)(temp*100)%100);
            delay_ms(20);
        }
        else
            printf("MS5611 unavailable\n");

    }
} 

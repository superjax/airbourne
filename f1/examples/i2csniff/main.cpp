#include "i2c.h"
#include "board.h"
#include "uart.h"
#include "printf.h"

#include "mpu6050.h"

UART Serial1;

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    Serial1.put_byte((uint8_t*)&c, 1);
}

int main()
{
    SystemInit();
    OSCinit();
    enableInterrupts();
    enablePeripherals();
    startWallClock();

    I2C i2c1;
    i2c1.init(2);

    Serial1.init(1, 115200, UART::MODE_IT);
    init_printf(NULL, _putc);

    uint8_t addr = 0x77;
    while(1)
    {
        for(addr = 0; addr < 128; addr++)
        {
            if(i2c1.write_byte(addr, 0x00, 0x00))
            {
                    printf("found device on %d\n", addr);
            }
        }


        printf("===============================================");
    }
}

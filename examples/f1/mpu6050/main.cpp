#include "i2c.h"
#include "board.h"
#include "uart.h"
#include "printf.h"
#include "mpu6050.h"
#include "vector3.h"

UART Serial1;
MPU6050 mpu;

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    Serial1.put_byte((uint8_t*)&c, 1);
}


vector3 accel, gyro;
float temp;
uint64_t time;

static void print_measurements(void)
{
    mpu.async_read_all(&accel, &gyro, &temp, &time);
    printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
           (int32_t) (accel.x*1000.0),
           (int32_t) (accel.y*1000.0),
           (int32_t) (accel.z*1000.0),
           (int32_t) (gyro.x*1000.0),
           (int32_t) (gyro.y*1000.0),
           (int32_t) (gyro.z*1000.0),
           (int32_t) (temp*1000.0));
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

    Serial1.init(1, 115200, UART::MODE_DMA_TX_RX);
    init_printf(NULL, _putc);

    bool polling = false;

    mpu.init(&i2c1, MPU6050::ACCEL_FSR_8G, MPU6050::GYRO_FSR_2000DPS, MPU6050::LPF_42HZ, !polling);

    // Register the print measurement CB for when I2C transmission is complete from the IMU
    if(!polling)
    {
        mpu.register_transfer_complete_CB(&print_measurements);
    }

    while(1)
    {
        if(polling)
        {
            mpu.read_all(&accel, &gyro, &temp);
            printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
                   (int32_t) (accel.x*1000.0),
                   (int32_t) (accel.y*1000.0),
                   (int32_t) (accel.z*1000.0),
                   (int32_t) (gyro.x*1000.0),
                   (int32_t) (gyro.y*1000.0),
                   (int32_t) (gyro.z*1000.0),
                   (int32_t) (temp*1000.0));
        }

        // If we are letting the MPU run using the EXTI, we don't do anything in this loop, we just let the IMU crank
        // and use the callbacks
    }
}

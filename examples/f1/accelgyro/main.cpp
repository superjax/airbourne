/*
 * accelgyro - Reads and Prints Accelerometer and Gyro Readings
 * Copyright (c) 2016 James Jackson
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "airbourne.h"

// This is a wrapper for the printf function that puts the stdout
// stream onto UART1
static void _putc(void *p, char c)
{
  (void)p; // avoid compiler warning about unused variable
  _uart1.put_byte((uint8_t*)&c, 1);
}

// This the memory and callbacks for IMU updates
// The idea is that when the IMU data is ready, it will trigger
// an external interrupt, which will call IMU_interrupt_CB.
// We immediately request a data update, which asynchronously
// transfers the measurement via I2C and when the transfer is completed,
// the IMU calls the data_ready_CB.  We can then immediately call the read_all
// function, which pulls the data into local variables ready for processing
volatile uint8_t status;
vector3 accel;
vector3 gyro;
float temp;
void IMU_interrupt_CB(void)
{
  _mpu6050.request_async_update();
}

void IMU_data_ready_CB(void)
{
  _mpu6050.read_all(&accel, &gyro, &temp);

  static int output_throttle;
  if(output_throttle == 10)
  {
    output_throttle = 0;
    printf("%d.%d, %d.%d, %d.%d, %d.%d, %d.%d, %d.%d, %d.%d\n",
           (int)accel.x,(int)(accel.x*1000)%1000,
           (int)accel.y,(int)(accel.y*1000)%1000,
           (int)accel.z,(int)(accel.z*1000)%1000,

           (int)gyro.x,(int)(gyro.x*1000)%1000,
           (int)gyro.y,(int)(gyro.y*1000)%1000,
           (int)gyro.z,(int)(gyro.z*1000)%1000,

           (int)temp,(int)(temp*1000)%1000);
  }
  output_throttle++;
}

int main(void)
{
  airbourne_init();
  _uart1.init(1, 115200, UART::MODE_DMA_TX_RX);
  init_printf(NULL, _putc);

  _LED0.toggle();
  _i2c.init(2);

  _mpu6050.init(&_i2c, MPU6050::ACCEL_FSR_8G, MPU6050::GYRO_FSR_2000DPS, MPU6050::LPF_42HZ, true);
  _mpu6050.register_transfer_complete_CB(&IMU_data_ready_CB);

  while (1)
  {
    delay_ms(500);
    _LED0.toggle();
    _LED1.toggle();
  }
}

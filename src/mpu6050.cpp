#include <math.h>

#include "mpu6050.h"
#include "hmc5883l.h"
#include "i2c.h"
#include "board.h"

MPU6050* IMUPtr;

// External Callback declarations (wrappers for the actual C-based callbacks)
void data_ready(void);
void transfer_complete(void);

MPU6050::MPU6050() {}

MPU6050::MPU6050(I2C* I2C_Dev, uint8_t accel_FSR, uint8_t gyro_FSR, uint8_t LPF, bool enable_EXTI)
{
    init(I2C_Dev, accel_FSR, gyro_FSR, LPF, enable_EXTI);
}

void MPU6050::init(I2C* I2C_Dev, uint8_t accel_FSR, uint8_t gyro_FSR, uint8_t LPF, bool enable_EXTI)
{
    // Register the I2C device in the class member
    I2C_ = I2C_Dev;

    // Connect the IMUPtr to this instantiation of the class
    IMUPtr = this;

    // The number of G's in a full-scale accel reading
    int acc_range_G = 2 << accel_FSR;

    // Deg/s in a full-scale gyro reading
    int gyro_range_deg_s = 250 << gyro_FSR;

    // Maximum possible reading (From FSR divided by 2^15 (max of int16))
    accel_scale_ = ((float)acc_range_G * 9.80665f) / ((float)0x7FFF);
    gyro_scale_ = ((float)gyro_range_deg_s * 3.14159f/180.0f) / ((float)0x7FFF);

    bool success = true;
    // Reset IMU
    success &= write(MPU6050_RA_PWR_MGMT_1, 0x80);
    delay_ms(100);

    // Configure Gyro
    success &= write(MPU6050_RA_SMPLRT_DIV, 0x00);  // Output full rate
    success &= write(MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_ZGYRO); // use the gyro as the clock source
    delay_ms(10);

    success &= write(MPU6050_RA_CONFIG, LPF); // set the LPF
    write(MPU6050_RA_GYRO_CONFIG, gyro_FSR << 3);  // Set the full-scale range of the gyro

    // Configure Accelerometer
    success &= write(MPU6050_RA_ACCEL_CONFIG, accel_FSR << 3);


    if(enable_EXTI)
    {
        // Configure Data Ready Interrupt
        success &= write(MPU6050_RA_INT_ENABLE, 1 << MPU6050_INTERRUPT_DATA_RDY_BIT);

        // Initialize MPU EXTI
        EXTI_init();
    }
    completeCB_ = NULL;
}

void gpioExtiLineConfig(uint8_t portsrc, uint8_t pinsrc)
{
    uint32_t tmp = 0x00;

    tmp = ((uint32_t)0x0F) << (0x04 * (pinsrc & (uint8_t)0x03));
    AFIO->EXTICR[pinsrc >> 0x02] &= ~tmp;
    AFIO->EXTICR[pinsrc >> 0x02] |= (((uint32_t)portsrc) << (0x04 * (pinsrc & (uint8_t)0x03)));
}

void MPU6050::EXTI_init()
{
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // Configure EXTI
    EXTI_ClearITPendingBit(IMU_EXTERNAL_INTERRUPT_LINE);
    EXTI_InitTypeDef EXTI_InitStrutcure;

    // Initialize GPIO for external Interrupt
    uint32_t tmp = 0x00;
    tmp = ((uint32_t)0x0F) << (0x04 * (IMU_EXTERNAL_INTERRUPT_PINSOURCE & (uint8_t)0x03));
    AFIO->EXTICR[IMU_EXTERNAL_INTERRUPT_PINSOURCE >> 0x02] &= ~tmp;
    AFIO->EXTICR[IMU_EXTERNAL_INTERRUPT_PINSOURCE >> 0x02] |= (((uint32_t)IMU_EXTERNAL_INTERRUPT_PORTSOURCE) << (0x04 * (IMU_EXTERNAL_INTERRUPT_PINSOURCE & (uint8_t)0x03)));

    // Configure EXTI Line13
    EXTI_InitStrutcure.EXTI_Line = IMU_EXTERNAL_INTERRUPT_LINE;
    EXTI_InitStrutcure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStrutcure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStrutcure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStrutcure);

    // Disable AFIO Clock - we don't need it anymore
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, DISABLE);

    // Configure NVIC (Nested Vector Interrupt Controller)
    NVIC_InitTypeDef NVIC_InitStructure;
    // Select NVIC Channel to configure
    NVIC_InitStructure.NVIC_IRQChannel = IMU_EXTI_IRQn;
    // Set priority to lowest
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    // Set subpriority to lowest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // Update NVIC registers
    NVIC_Init(&NVIC_InitStructure);
}

void MPU6050::register_transfer_complete_CB(void (*functionPtr)(void))
{
    completeCB_ = functionPtr;
}

// Blocking Read Functions
void MPU6050::read_accel(vector3 *acc)
{
    uint8_t buf[6];
    read(MPU6050_RA_ACCEL_XOUT_H, buf, 6);

    acc->x = ((float)(int16_t)((buf[0] << 8) | buf[1])) * accel_scale_;
    acc->y = ((float)(int16_t)((buf[2] << 8) | buf[3])) * accel_scale_;
    acc->z = ((float)(int16_t)((buf[4] << 8) | buf[5])) * accel_scale_;
}


void MPU6050::read_gyro(vector3* gyr)
{
    uint8_t buf[6];
    read(MPU6050_RA_GYRO_XOUT_H, buf, 6);

    gyr->x = ((float)(int16_t)((buf[0] << 8) | buf[1])) * gyro_scale_;
    gyr->y = ((float)(int16_t)((buf[2] << 8) | buf[3])) * gyro_scale_;
    gyr->z = ((float)(int16_t)((buf[4] << 8) | buf[5])) * gyro_scale_;
}


void MPU6050::read_temp(float* temp)
{
    uint8_t buf[2];
    read(MPU6050_RA_TEMP_OUT_H, buf, 2);

    *temp = ((float)(int16_t)((buf[0] << 8) | buf[1]))/340.0f + 36.53f;
}


void MPU6050::read_all(vector3* acc, vector3* gyro, float* temp)
{
    uint8_t buf[14];
    read(MPU6050_RA_ACCEL_XOUT_H, buf, 14);

    acc->x = ((float)(int16_t)((buf[0] << 8) | buf[1])) * accel_scale_;
    acc->y = ((float)(int16_t)((buf[2] << 8) | buf[3])) * accel_scale_;
    acc->z = ((float)(int16_t)((buf[4] << 8) | buf[5])) * accel_scale_;

    *temp = ((float)(int16_t)((buf[6] << 8) | buf[7]))/340.0f + 36.53f;

    gyro->x = ((float)(int16_t)((buf[8] << 8) | buf[9])) * gyro_scale_;
    gyro->y = ((float)(int16_t)((buf[10] << 8) | buf[11])) * gyro_scale_;
    gyro->z = ((float)(int16_t)((buf[12] << 8) | buf[13])) * gyro_scale_;
}

// Asynchronous Read Functions <-- This is also called by the EXTI interrupt tied to the IMU
void MPU6050::request_async_update()
{
    time_us_ = micros();

    I2C_->queue_job_IT(I2C::READ | I2C::IT, MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_ACCEL_XOUT_H, data_buffer_, 14, NULL, &transfer_complete);
}

// This converts data in the buffer to the various measurements in SI units
bool MPU6050::async_read_all(vector3 *acc, vector3 *gyro, float *temp, uint64_t *time_us)
{
    bool out = new_data_;

    acc->x = (float)((int16_t)((data_buffer_[0] << 8) | data_buffer_[1])) * accel_scale_;
    acc->y = (float)((int16_t)((data_buffer_[2] << 8) | data_buffer_[3])) * accel_scale_;
    acc->z = (float)((int16_t)((data_buffer_[4] << 8) | data_buffer_[5])) * accel_scale_;

    *temp = (float)((int16_t)((data_buffer_[6] << 8) | data_buffer_[7]))/340.0f + 36.53f;

    gyro->x = (float)((int16_t)((data_buffer_[8] << 8) | data_buffer_[9])) * gyro_scale_;
    gyro->y = (float)((int16_t)((data_buffer_[10] << 8) | data_buffer_[11])) * gyro_scale_;
    gyro->z = (float)((int16_t)((data_buffer_[12] << 8) | data_buffer_[13])) * gyro_scale_;

    *time_us = time_us_;

    new_data_ = false;

    return out;
}

bool MPU6050::new_data()
{
    return new_data_;
}

bool MPU6050::write(uint8_t reg, uint8_t data)
{
    I2C_->write_byte(MPU6050_ADDRESS_AD0_LOW, reg, data);
}

bool MPU6050::read(uint8_t reg, uint8_t *data, uint32_t len)
{
    I2C_->read(MPU6050_ADDRESS_AD0_LOW, reg, len, data);
}

void MPU6050::I2C_complete_CB()
{
    new_data_ = true;
    // Here, we have gotten brand new data over I2C, so it's time to call the registered callback
    if(completeCB_ != NULL)
    {
        completeCB_();
    }
}

//=================================================================
// Wrappers for dealing with interrupts
void transfer_complete(void)
{
    IMUPtr->I2C_complete_CB();
}

// Both the IMU and the Magnetometer use the same interrupt
extern "C" void IMU_EXTI_IRQHandler(void)
{
    // IMU interrupt
    if (EXTI_GetITStatus(IMU_EXTERNAL_INTERRUPT_LINE) != RESET)
    {
        IMUPtr->request_async_update();
    }
    EXTI_ClearITPendingBit(IMU_EXTERNAL_INTERRUPT_LINE);

    // MAG Interrupt
    if(EXTI_GetITStatus(MAG_EXTERNAL_INTERRUPT_LINE) != RESET)
    {
        HMC5883LPtr->update();
    }
    EXTI_ClearITPendingBit(MAG_EXTERNAL_INTERRUPT_LINE);
}
//===============================================================




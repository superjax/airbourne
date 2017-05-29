#include <math.h>

#include "hmc5883l.h"
#include "board.h"

void i2c_complete_callback();

HMC5883L* HMC5883LPtr;

HMC5883L::HMC5883L()
{
    HMC5883LPtr = this;
}


bool HMC5883L::init(I2C* I2CDev)
{
    // Link up I2C object
    i2c = I2CDev;

    // Link up external pointer for IRQ's
    HMC5883LPtr = this;

    // Initialize Gain
    gain_.x = 1.0f/390.0f;
    gain_.y = 1.0f/390.0f;
    gain_.z = 1.0f/390.0f;

    DRDY_.init(MAG_DRDY_GPIO, MAG_DRDY_PIN, GPIO_Mode_IPD);

    // wait for magnetometer to initialize
    while(millis() < 500);

    // Detect Magnetometer
    uint8_t byte = 0;
    if(!i2c->read(HMC58X3_ADDR, HMC58X3_ID1, 1, &byte))
    {
        return false;
    }
    else if( byte != 0x34)
    {
        return false;
    }

    // Configure HMC5833L
    i2c->write_byte(HMC58X3_ADDR, HMC58X3_CRA, HMC58X3_CRA_DO_75 | HMC58X3_CRA_NO_AVG | HMC58X3_CRA_MEAS_MODE_NORMAL ); // 75 Hz Measurement, no bias, no averaging
    i2c->write_byte(HMC58X3_ADDR, HMC58X3_CRB, HMC58X3_CRB_GN_390); // 390 LSB/Gauss
    i2c->write_byte(HMC58X3_ADDR, HMC58X3_MODE, HMC58X3_MODE_CONTINUOUS); // Continuous Measurement Mode

    // Clear any data with the wrong gain
    uint32_t timeout = HMC58X3_TIMEOUT;
    vector3 magADC;
    while(!DRDY_.read() && --timeout > 0); // Wait fo the data ready to fire
    if(timeout == 0)
    {
        return false;
    }
    blocking_read(&magADC);

    timeout = HMC58X3_TIMEOUT;
    while(!DRDY_.read() && --timeout > 0); // Read new data, with the right gain
    if(timeout == 0)
    {
        return false;
    }
    blocking_read(&magADC);

    // Set up the DRDY interrupt
    configure_DRDY_interrupt();

    return true;
}

// The Magnetometer uses the same external interrupt as the IMU.  This makes the code a bit more
// complicated, because the callback for the Magnetometer Interrupt is actually in the imu .cpp file
void HMC5883L::configure_DRDY_interrupt()
{
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // Configure EXTI
    EXTI_ClearITPendingBit(MAG_EXTERNAL_INTERRUPT_LINE);
    EXTI_InitTypeDef EXTI_InitStrutcure;

    // Initialize GPIO for external Interrupt
    uint32_t tmp = 0x00;
    tmp = ((uint32_t)0x0F) << (0x04 * (MAG_EXTERNAL_INTERRUPT_PINSOURCE & (uint8_t)0x03));
    AFIO->EXTICR[MAG_EXTERNAL_INTERRUPT_PINSOURCE >> 0x02] &= ~tmp;
    AFIO->EXTICR[MAG_EXTERNAL_INTERRUPT_PINSOURCE >> 0x02] |= (((uint32_t)MAG_EXTERNAL_INTERRUPT_PORTSOURCE) << (0x04 * (MAG_EXTERNAL_INTERRUPT_PINSOURCE & (uint8_t)0x03)));

    // Configure EXTI Line13
    EXTI_InitStrutcure.EXTI_Line = MAG_EXTERNAL_INTERRUPT_LINE;
    EXTI_InitStrutcure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStrutcure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStrutcure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStrutcure);

    // Disable AFIO Clock - we don't need it anymore
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, DISABLE);

    // Configure NVIC (Nested Vector Interrupt Controller)
    NVIC_InitTypeDef NVIC_InitStructure;
    // Select NVIC Channel to configure
    NVIC_InitStructure.NVIC_IRQChannel = MAG_EXTI_IRQn;
    // Set priority to lowest
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    // Set subpriority to lowest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // Update NVIC registers
    NVIC_Init(&NVIC_InitStructure);
}

void HMC5883L::update()
{
    i2c->queue_job_IT(I2C::READ, HMC58X3_ADDR, HMC58X3_DATA, data_buffer_, 6, NULL, &i2c_complete_callback);
    return;
}

void HMC5883L::read(vector3 *mag)
{
    new_data_ = false;
    raw_data_[0] = (int16_t)((data_buffer_[0] << 8) | data_buffer_[1]);
    raw_data_[1] = (int16_t)((data_buffer_[2] << 8) | data_buffer_[3]);
    raw_data_[2] = (int16_t)((data_buffer_[4] << 8) | data_buffer_[5]);

    mag->x = (float)raw_data_[0] * gain_.x;
    mag->y = (float)raw_data_[1] * gain_.y;
    mag->z = (float)raw_data_[2] * gain_.z;
}

void HMC5883L::blocking_read(vector3 *data)
{
    i2c->read(HMC58X3_ADDR, HMC58X3_DATA, 6, data_buffer_);
    // During calibration, gain is 1.0, so the read returns normal non-calibrated values.
    // After calibration is done, gain is set to calculated gain values.
    raw_data_[0] = (int16_t)((data_buffer_[0] << 8) | data_buffer_[1]);
    raw_data_[1] = (int16_t)((data_buffer_[2] << 8) | data_buffer_[3]);
    raw_data_[2] = (int16_t)((data_buffer_[4] << 8) | data_buffer_[5]);

    data->x = (float)raw_data_[0] * gain_.x;
    data->y = (float)raw_data_[1] * gain_.y;
    data->z = (float)raw_data_[2] * gain_.z;
}

bool HMC5883L::new_data()
{
    return new_data_;
}

void HMC5883L::data_read_CB()
{
    new_data_ = true;
}

void i2c_complete_callback()
{
    HMC5883LPtr->data_read_CB();
}



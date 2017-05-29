#include "ms5611.h"

void ms5611_request_CB();
void ms5611_adc_read_CB();

MS5611* MS5611Ptr;

MS5611::MS5611()
{
    MS5611Ptr = this;
}

bool MS5611::init(I2C* I2CDev)
{
    MS5611Ptr = this;
    i2c = I2CDev;
    bool success = true;

    uint8_t byte;

    i2c->write_byte(0, 0, 0);

    // Wait for the MS5611 to power up
    while(millis() < 10){}

    success &= i2c->read(MS5611_ADDR, MS5611_PROM_RD, 1, &byte);

    if(!success)
        return false;

    reset();

    // initialize the state variable
    current_state_ = READ_PRESSURE;

    // read the PROM
    return read_prom();
    }


void MS5611::update()
{
    uint64_t now = micros();

    // If it's not time to do anything, just wait
    if (next_update_time_us > now)
    {
        return;
    }

    else if (current_state_ == READ_PRESSURE)
    {
        // start a pressure measurement
        i2c->queue_job_IT(I2C::READ, MS5611_ADDR, MS5611_ADC_READ, pressure_buffer_, 3, NULL, &ms5611_adc_read_CB);
        next_update_time_us = now + pressure_interval_us;
    }
    else if (current_state_ == READ_TEMPERATURE)
    {
        // Start a temperature measurement
        i2c->queue_job_IT(I2C::READ, MS5611_ADDR, MS5611_ADC_READ, temp_buffer_, 3, NULL, &ms5611_adc_read_CB);
        next_update_time_us = now + temp_interval_us;
    }
}

void MS5611::read(float *pressure, float *temperature)
{
    convert_to_SI();
    *pressure = pressure_SI_;
    *temperature = temperature_SI_;
}


void MS5611::reset()
{
    i2c->write_byte(MS5611_ADDR, MS5611_RESET, 1);
    delay_us(2800);
}

bool MS5611::read_prom()
{
    // Read the PROM
    for (int i = 0; i < 8; i++)
    {
        uint8_t rxbuf[3] = {0, 0, 0};
        volatile uint8_t status;
        i2c->read(MS5611_ADDR, MS5611_PROM_RD + i * 2, 3, rxbuf);
        PROM_[i] = (uint16_t)((rxbuf[0] << 8) |  rxbuf[1]);
    }

    int32_t i, j;
    uint32_t res = 0;
    uint8_t crc = PROM_[7] & 0xF;
    PROM_[7] &= 0xFF00;

    bool empty = true;

    for (i = 0; i < 16; i++) {
        if (PROM_[i >> 1]) {
            empty = false;
        }
        if (i & 1)
            res ^= ((PROM_[i >> 1]) & 0x00FF);
        else
            res ^= (PROM_[i >> 1] >> 8);
        for (j = 8; j > 0; j--) {
            if (res & 0x8000)
                res ^= 0x1800;
            res <<= 1;
        }
    }
    PROM_[7] |= crc;
    if (!empty && crc == ((res >> 12) & 0xF))
        return true;

    return false;
}

void MS5611::read_CB()
{
    if(current_state_ == READ_PRESSURE)
    {
        // Convert raw bytes into a measurement
        uint32_t pressure = (pressure_buffer_[0] << 16) | (pressure_buffer_[1] << 8) | (pressure_buffer_[2]);
        if(pressure != 0)
        {
            raw_pressure_ = pressure;
        }
        // start a temperature update
        uint8_t temp_command = 1;
        i2c->queue_job_IT(I2C::WRITE, MS5611_ADDR, MS5611_ADC_CONV + MS5611_ADC_D2 + OSR_, &temp_command, 1, NULL, &ms5611_request_CB);
    }
    else if (current_state_ == READ_TEMPERATURE)
    {
        // Convert raw byte ino a measurement
        uint32_t temperature = (temp_buffer_[0] << 16) | (temp_buffer_[1] << 8) | (temp_buffer_[2]);
        if (temperature != 0)
        {
            raw_temp_ = temperature;
        }
        // Start a temperature update
        uint8_t pressure_command = 1;
        i2c->queue_job_IT(I2C::WRITE, MS5611_ADDR, MS5611_ADC_CONV + MS5611_ADC_D1 + OSR_, &pressure_command, 1, NULL, &ms5611_request_CB);
    }
    else
    {
        // This shouldn't ever happen, but if it does
        current_state_ = READ_PRESSURE;
    }
}

void MS5611::measurement_request_CB()
{
    if(current_state_ == READ_PRESSURE)
    {
        next_update_time_us = micros() + temp_interval_us;
        current_state_ = READ_TEMPERATURE;
    }
    else if (current_state_ == READ_TEMPERATURE)
    {
        next_update_time_us = micros() + pressure_interval_us;
        current_state_ = READ_PRESSURE;
    }
}

void MS5611::convert_to_SI()
{
    uint32_t press;
    int64_t temp;
    int64_t delta;
    int64_t dT = (int64_t)raw_temp_ - ((uint64_t)PROM_[5] * 256);
    int64_t off = ((int64_t)PROM_[2] << 16) + (((int64_t)PROM_[4] * dT) >> 7);
    int64_t sens = ((int64_t)PROM_[1] << 15) + (((int64_t)PROM_[3] * dT) >> 8);
    temp = 2000 + ((dT * (int64_t)PROM_[6]) >> 23);

    // temperature measurement is lower than 20degC
    if (temp < 2000)
    {
        delta = temp - 2000;
        delta = 5 * delta * delta;
        off -= delta >> 1;
        sens -= delta >> 2;

        // temperature lower than -15degC
        if (temp < -1500)
        {
            delta = temp + 1500;
            delta = delta * delta;
            off -= 7 * delta;
            sens -= (11 * delta) >> 1;
        }
        temp -= ((dT * dT) >> 31);
    }
    press = ((((int64_t)raw_pressure_ * sens) >> 21) - off) >> 15;

   pressure_SI_ = (float)press;
   temperature_SI_ = (float)(temp/100.0);
}


void ms5611_request_CB()
{
    MS5611Ptr->measurement_request_CB();
}

void ms5611_adc_read_CB()
{
    MS5611Ptr->read_CB();
}


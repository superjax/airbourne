/*
   drv_i2c.c :  I^2C support for STM32F103

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/drv_i2c.c

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

#define I2C_DEVICE (I2CDEV_2)

#include <stdbool.h>
#include <stdio.h>
#include <string.h> // memset
#include <stdlib.h>

#include "stm32f10x_conf.h"
//#include "drv_system.h"         // timers, delays, etc
#include "gpio.h"
#include "board.h"

#include "i2c.h"

#ifndef SOFT_I2C

// I2C2
// SCL  PB10
// SDA  PB11
// I2C1
// SCL  PB6
// SDA  PB7


I2C* I2CDev_1Ptr;
I2C* I2CDev_2Ptr;

#define I2C_DEFAULT_TIMEOUT 100


void I2C::init(uint8_t index)
{
    //    i2cDevice_t hardware = i2cHardwareMap[index];

    NVIC_InitTypeDef nvic;
    I2C_InitTypeDef i2c;

    if (index > NUM_I2C_DEVICES)
        index = NUM_I2C_DEVICES;

    // Link up the C pointers and hardware registers
    if(index == 1)
    {
        SCL_.init(I2C1_GPIO, I2C1_SCL_PIN, GPIO_Mode_AF_OD);
        SDA_.init(I2C1_GPIO, I2C1_SDA_PIN, GPIO_Mode_AF_OD);
        I2Cx_ = I2C1;
        I2Cx_index_ = 1;
        ev_irq_ = I2C1_EV_IRQn;
        er_irq_ = I2C1_ER_IRQn;
        I2CDev_1Ptr = this;
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    }
    if(index == 2)
    {
        SCL_.init(I2C2_GPIO, I2C2_SCL_PIN, GPIO_Mode_AF_OD);
        SDA_.init(I2C2_GPIO, I2C2_SDA_PIN, GPIO_Mode_AF_OD);
        I2Cx_ = I2C2;
        I2Cx_index_ = 2;
        ev_irq_ = I2C2_EV_IRQn;
        er_irq_ = I2C2_ER_IRQn;
        I2CDev_2Ptr = this;
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    }

    // clock out stuff to make sure slaves arent stuck
    // This will also configure GPIO as AF_OD at the end
    unstick();

    // Init I2C peripheral
    I2C_DeInit(I2Cx_);
    I2C_StructInit(&i2c);

    // The EVT and ERR interrupts get enabled when we start an i2c job
    I2C_ITConfig(I2Cx_, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed = 400000;
    I2C_Cmd(I2Cx_, ENABLE);
    I2C_Init(I2Cx_, &i2c);

    // I2C ER Interrupt
    nvic.NVIC_IRQChannel = er_irq_;
    nvic.NVIC_IRQChannelPreemptionPriority = 4;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    // I2C EV Interrupt
    nvic.NVIC_IRQChannel = ev_irq_;
    nvic.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_Init(&nvic);

    busy_ = false;

    // Initialize buffer
    init_buffer();
    async_watchdog_start_time_us_ = micros();
    job_status_ = NULL;
}

void I2C::DMA_Config()
{
    if(operation_ & WRITE)
    {
        // Confiigure the DMA Tx Channel with the buffer address and the buffer size
        DMA_InitStruct_.DMA_MemoryBaseAddr = (uint32_t)data_buffer_;
        DMA_InitStruct_.DMA_DIR = DMA_DIR_PeripheralDST;
        DMA_InitStruct_.DMA_BufferSize = (uint32_t)len_;
        DMA_InitStruct_.DMA_MemoryBaseAddr = (uint32_t)&I2Cx_->DR;
        DMA_Cmd(I2C2_DMA_CHANNEL_TX, DISABLE);
        DMA_Init(I2C2_DMA_CHANNEL_TX, &DMA_InitStruct_);
        DMA_Cmd(I2C2_DMA_CHANNEL_TX, ENABLE);
    }
    else
    {
        DMA_InitStruct_.DMA_MemoryBaseAddr = (uint32_t)data_buffer_;
        DMA_InitStruct_.DMA_DIR = DMA_DIR_PeripheralSRC;
        DMA_InitStruct_.DMA_BufferSize = (uint32_t)len_;
        DMA_InitStruct_.DMA_PeripheralBaseAddr = (uint32_t)&I2Cx_->DR;;
        DMA_Cmd(I2C2_DMA_CHANNEL_RX, DISABLE);
        DMA_Init(I2C2_DMA_CHANNEL_RX, &DMA_InitStruct_);
        DMA_Cmd(I2C2_DMA_CHANNEL_RX, ENABLE);
    }

}

bool I2C::hardware_failure(void)
{
    error_count_++;
    // reinit peripheral + clock out garbage
    init(I2Cx_index_);
    return false;
}

bool I2C::start_I2C_job()
{
    uint32_t timeout = I2C_DEFAULT_TIMEOUT;

    // wait for the bus to become free
    while(busy_)
    {
        // wait for the bus to become free
        if(timeout-- == 0)
            return hardware_failure();
    }

    // Turn the bus back on on
    busy_ = true;

    timeout = I2C_DEFAULT_TIMEOUT;

    // If the bus is inactive
    if (!(I2Cx_->CR2 & I2C_IT_EVT))
    {
        // If there is no start pending
        if (!(I2Cx_->CR1 & I2C_CR1_START))
        {
            // Wait for any stop to finish
            while (I2Cx_->CR1 & I2C_CR1_STOP)
            {
                if (timeout-- == 0)
                    return hardware_failure();
            }
            // Start the new job
            I2C_GenerateSTART(I2Cx_, ENABLE);
        }
        // Turn on the interrupts
        I2C_ITConfig(I2Cx_, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF, ENABLE);
    }
}

bool I2C::perform_I2C_job(uint8_t type, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len, volatile uint8_t *status, void (*CB)())
{

    addr_ = addr << 1;
    reg_ = reg;
    operation_ = type;
    data_buffer_ = buf;
    len_ = len;
    error_ = false;
    complete_CB_ = CB;
    job_status_ = status;
    index_ = 0;
    final_stop_ = false;
    subaddress_sent_ = (reg_ == 0xFF);

    // reset the time
    async_watchdog_start_time_us_ = micros();

    if (!I2Cx_)
        return false;

    //====================================
    // Start the Job
    //====================================
    if(type & IT)
    {
        // Clear the POS bit
        I2Cx_->CR1 &= (uint16_t) ~I2C_CR1_POS;
        // Set the ACK
        I2C_AcknowledgeConfig(I2Cx_, ENABLE);

        // Start the job
        start_I2C_job();
    }

    //===================================
    // Polling
    //===================================
    if(type & POLLING)
    {
        // wait for the interrupt routines to take care of the job
        uint32_t timeout = I2C_DEFAULT_TIMEOUT;
        while(busy_ && --timeout > 0);
        if(timeout == 0)
            return hardware_failure();
        else
            delay_us(1);
    }

    return true;
}

bool I2C::write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    return perform_I2C_job(WRITE | POLLING, addr, reg, data, len, NULL, NULL);
}

bool I2C::write_byte(uint8_t addr, uint8_t reg, uint8_t data)
{
    return perform_I2C_job(WRITE | POLLING, addr, reg, &data, 1, NULL, NULL);
}

bool I2C::read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{

    return perform_I2C_job(READ | POLLING, addr, reg, buf, len, NULL, NULL);
}

bool I2C::read_IT(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf, volatile uint8_t* status, void (*CB)(void))
{
    return perform_I2C_job(READ | IT, addr, reg, buf, len, status, CB);
}

bool I2C::write_IT(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf, volatile uint8_t* status, void (*CB)(void))
{
    return perform_I2C_job(WRITE |IT, addr, reg, buf, len, status, CB);
}

void I2C::error_handler(void)
{
    // Read the I2C1 status register
    volatile uint32_t SR1 = I2Cx_->SR1;

    if (SR1 & 0x0F00)
        error_ = true;

    // If AF, BERR or ARLO, abandon the current job and commence new if there are jobs
    if (SR1 & I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR)
    {
        // read second status register to clear ADDR if it is set
        (void)I2Cx_->SR2;
        // disable the RXNE/TXE interrupt
        I2C_ITConfig(I2Cx_, I2C_IT_BUF, DISABLE);
        if (!(SR1 & I2C_SR1_ARLO) && !(I2Cx_->CR1 & I2C_CR1_STOP))
        {
            // If we are currently trying to send a start
            if (I2Cx_->CR1 & I2C_CR1_START)
            {
                uint32_t timeout = I2C_DEFAULT_TIMEOUT;
                while (I2Cx_->CR1 & I2C_CR1_START && --timeout > 0);
                if(timeout == 0)
                    hardware_failure();
                // Send a stop
                I2C_GenerateSTOP(I2Cx_, ENABLE);
                timeout = I2C_DEFAULT_TIMEOUT;
                while (I2Cx_->CR1 & I2C_CR1_STOP && --timeout > 0);
                if(timeout == 0)
                    hardware_failure();
                // reset and configure the hardware
                init(I2Cx_index_);
            }
            else
            {
                // stop to free up the bus
                I2C_GenerateSTOP(I2Cx_, ENABLE);
                // Interrupts will be enabled by future jobs
                I2C_ITConfig(I2Cx_, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
            }
        }
    }
    // reset all the error bits to clear the interrupt
    I2Cx_->SR1 &= ~0x0F00;

    // Update job status
    if (job_status_ != NULL)
        (*job_status_) = I2C_JOB_ERROR;

    // Call the complete callback
    if (complete_CB_ != NULL)
        complete_CB_();

    // allow new jobs
    busy_ = 0;

    // load new job, if there is one
    job_handler();
}

void I2C::event_handler(void)
{
    if(operation_ & POLLING)
    {
        // don't do anything
        return;
    }

    uint32_t SR1 = I2Cx_->SR1;

    // Just set a start bit
    if(SR1 & I2C_SR1_SB)
    {
        volatile uint32_t temp = I2Cx_->SR2;
        if((operation_ & READ) && subaddress_sent_)
        {
            // Set the POS bit on a two-byte receive
            if(len_ == 2)
            {
                I2Cx_->CR1 |= I2C_CR1_POS;
            }
            // start a read
            I2C_Send7bitAddress(I2Cx_, addr_, I2C_Direction_Receiver);
        }
        else
        {
            // if we need to send a subaddress, or we are writing bits,
            // start a write
            I2C_Send7bitAddress(I2Cx_, addr_, I2C_Direction_Transmitter);
        }
    }

    // Just sent the address
    else if(SR1 & I2C_SR1_ADDR)
    {
        //        __disable_irq();
        // read SR2 to clear the ADDR bit
        volatile uint32_t temp = I2Cx_->SR2;

        if(!subaddress_sent_)
        {
            //            __enable_irq();
            // write the subaddress - This also clears the EV8_1
            I2Cx_->DR = reg_;
        }

        else if(operation_ & WRITE)
        {
            //            __enable_irq();
            // Send the first byte - Clear EV8_1
            I2Cx_->DR = data_buffer_[index_++];
            len_--;

            if(len_ == 0)
            {
                // Set a STOP
                I2Cx_->CR1 |= I2C_CR1_STOP;

                // Disable the TXE interrupt
                I2Cx_->CR2 &= (uint16_t) ~I2C_IT_BUF;
            }
            else
            {
                // Enable TXE interrupt to handle individual bytes
                I2Cx_->CR2 |= I2C_IT_BUF;
            }
        }

        else // READ
        {
            // This is a EV6_1
            if(len_ == 1)
            {
                // Disable ACK
                I2Cx_->CR1 &= (uint16_t) ~I2C_CR1_ACK;
                // Program the STOP
                I2Cx_->CR1 |= I2C_CR1_STOP;
                //                __enable_irq();
            }

            // This is a two-byte receive
            else if(len_ == 2)
            {
                // set NACK
                I2Cx_->CR1 &= (uint16_t) ~I2C_CR1_ACK;
                // Disable TXE to allow buffer to fill
                I2Cx_->CR2 &= (uint16_t) ~I2C_IT_BUF;
                //                __enable_irq();
            }

            // Three byte read
            else if (len_ == 3)
            {
                // Disable the RXNE so we get a BTF in two bytes
                I2Cx_->CR2 &= (uint16_t) ~I2C_IT_BUF;
                //                __enable_irq();
            }

            // Greater than three byte read
            else
            {
                // Make sure RXNE is enabled to handle the individual bytes
                // until we get down to 3
                I2Cx_->CR2 |= I2C_IT_BUF;
                //                __enable_irq();
            }
        }
    }

    // If BTF is set, we have a EV7_2, EV7_3, or an EV8_2
    else if(SR1 & I2C_SR1_BTF)
    {
        // EV8_2
        if(SR1 & I2C_SR1_TXE)
        {
            if(len_ > 0)
            {
                // Write the data to the DR register
                I2Cx_->DR = data_buffer_[index_++];
                // Decrement the number of bytes to be written
                len_--;
            }
            if(len_ == 0)
            {
                // Program the STOP
                I2Cx_->CR1 |= I2C_CR1_STOP;
                // Disable the EVT IT in order to not have another BTF IT
                I2Cx_->CR2 &= (uint16_t) ~I2C_IT_EVT;
            }
        }

        // EV7_3 or EV7_2
        if (SR1 & I2C_SR1_RXNE)
        {
            // EV7_2
            if(len_ == 3) {
                // set NACK
                I2Cx_->CR1 &= (uint16_t) ~I2C_CR1_ACK;
                // Read byte N-2
                data_buffer_[index_++] = I2Cx_->DR;
                len_--;
                // Set the STOP
                I2Cx_->CR1 |= I2C_CR1_STOP;
                // Read byte N-1
                data_buffer_[index_++] = I2Cx_->DR;
                len_--;
                // Enable the RXNE interrupt again
                I2Cx_->CR2 |= I2C_IT_BUF;
            }

            // EV7_3
            else if(len_ == 2)
            {
                //                __disable_irq();
                I2C_GenerateSTOP(I2Cx_, ENABLE);
                // Read the two bytes in the buffer
                data_buffer_[index_++] = I2Cx_->DR;
                len_--;
                //                __enable_irq();
                data_buffer_[index_++] = I2Cx_->DR;
                len_--;
                /* Clear POS bit */
                I2Cx_->CR1  &= (uint16_t) ~I2C_CR1_POS;
            }
            else
            {
                // We shouldn't ever get here, but sometimes we get here if we weren't fast enough
                // handling the RXNE.  We have to handle the EV7 here
                // Read the DR register
                data_buffer_[index_++] = I2Cx_->DR;
                len_--;
                if(len_ == 3)
                {
                    // Disable the RXNE interrupt, so we get a BTF in 2 bytes
                    I2Cx_->CR2 &= (uint16_t) ~I2C_IT_BUF;
                }
                // If there is only one byte left to read, disable ACK and set STOP
                else if (len_ == 1)
                {
                    // Clear ACK
                    I2Cx_->CR1 &= (uint16_t) ~I2C_CR1_ACK;
                    // Program STOP
                    I2Cx_->CR1 |= I2C_CR1_STOP;
                }
            }
        }
        while (I2Cx_->CR1 & I2C_CR1_START);
    }

    // Byte was Received (EV7)
    else if(SR1 & I2C_SR1_RXNE)
    {
        // Read the DR register (also clears the RXNE)
        data_buffer_[index_++] = I2Cx_->DR;
        len_--;
        // If there are three bytes left, then perform a 3-byte read
        if(len_ == 3)
        {
            // Disable the RXNE interrupt, so we get a BTF in 2 bytes
            I2Cx_->CR2 &= (uint16_t) ~I2C_IT_BUF;
        }
        // If there is only one byte left to read, disable ACK and set STOP
        else if (len_ == 1)
        {
            // Clear ACK
            I2Cx_->CR1 &= (uint16_t) ~I2C_CR1_ACK;
            // Program STOP
            I2Cx_->CR1 |= I2C_CR1_STOP;
        }
    }

    // Just sent a byte (EV8) (TXE alone is set)
    else if(SR1 & I2C_SR1_TXE)
    {
        // we just sent the subaddress
        if(!subaddress_sent_)
        {
            // set the subaddress sent flag
            subaddress_sent_ = true;
            if(operation_ & READ)
            {
                // Send a second START
                I2Cx_->CR1 |= I2C_CR1_START;
            }
        }
        // If there is still data to write
        else if(len_ > 0)
        {
            // Write the data to the DR register
            I2Cx_->DR = data_buffer_[index_++];
            // Decrement the number of bytes to be written
            len_--;
        }

        // We just sent the final byte
        if(len_ == 0)
        {
            // Flush the buffer, and wait for an EV8_2 by disabling TXE interrupts
            I2Cx_->CR2 &= (uint16_t) ~I2C_IT_BUF;
        }
    }

    if(len_ == 0)
    {
        // Turn off EVT and ERR IT
        I2Cx_->CR2 &= (uint16_t) ~ I2C_IT_EVT;
        busy_ = false;

        // Set the Job flag
        if (job_status_ != NULL)
        {
            (*job_status_) = I2C_JOB_COMPLETE;
        }

        // Call the complete CB
        if (complete_CB_ != NULL)
        {
            complete_CB_();
        }

        // Perform Next Job
        job_handler();
    }
}




uint16_t I2C::get_error_count(void)
{
    return error_count_;
}

void I2C::unstick(void)
{
    SCL_.set_mode(GPIO_Mode_Out_OD);
    SDA_.set_mode(GPIO_Mode_Out_OD);

    SCL_.write(HIGH);
    SDA_.write(HIGH);

    // wait for any clock stretching to finish
    uint32_t timeout = I2C_DEFAULT_TIMEOUT;
    while(!SCL_.read() && timeout-- == 0);

    for(int i = 0; i < 8; i++)
    {


        // Toggle SCL pin a few times
        SCL_.write(LOW);
        delay_us(1);
        SCL_.write(HIGH);
        delay_us(1);
    }

    // Create a Start Condition
    SDA_.write(LOW);
    delay_us(5);
    SCL_.write(LOW);
    delay_us(5);

    // Create a Stop
    SCL_.write(HIGH);
    delay_us(5);
    SDA_.write(HIGH);

    // Configure Pins to be controlled by the I2C peripheral
    SCL_.set_mode(GPIO_Mode_AF_OD);
    SDA_.set_mode(GPIO_Mode_AF_OD);
}

void I2C::job_handler()
{
    if(buffer_count_ == 0)
    {
        // the queue is empty, stop performing i2c until
        // a new job is enqueued
        return;
    }

    while(busy_)
    {
        // check the watchdog timer
        uint64_t now = micros();
        //        if(labs(now < async_watchdog_start_time_us_))
        //        {
        //            volatile int error = 1;
        //        }
        if(labs(now - async_watchdog_start_time_us_) > 1000)
        {
            hardware_failure();
        }
        // wait for the current job to finish.  This function
        // will get called again when the job is done
        return;
    }

    // Perform the job on the front of the queue
    i2cJob_t* job = job_buffer_ + buffer_tail_;

    // First, change status to BUSY
    if(job->status != NULL)
        (*job->status) = I2C_JOB_BUSY;

    // Increment the tail
    buffer_tail_ = (buffer_tail_ +1) % I2C_BUFFER_SIZE;

    // Decrement the number of jobs on the buffer
    buffer_count_--;

    // perform the job
    perform_I2C_job(job->type, job->addr, job->reg, job->data, job->length, job->status, job->CB);

    return;
}

void I2C::queue_job_IT(uint8_t type, uint8_t addr_, uint8_t reg_, uint8_t *data, uint8_t length, volatile uint8_t* status_, void (*CB)(void))
{
    // Get a pointer to the head
    i2cJob_t* job = job_buffer_ + buffer_head_;

    // Make sure the IT flag is set
    type |= IT;

    // save the data about the job
    job->type = type;
    job->data = data;
    job->addr = addr_;
    job->reg = reg_;
    job->length = length;
    job->next_job = NULL;
    job->status = status_;
    job->CB = CB;

    // change job status
    if(job->status != NULL)
        (*job->status) = I2C_JOB_QUEUED;

    // Increment the buffer size
    buffer_count_++;

    if(buffer_count_ > I2C_BUFFER_SIZE)
    {
        // We have overrun the buffer, and will start dropping the oldest data
        buffer_count_ = I2C_BUFFER_SIZE;
        buffer_tail_ = (buffer_tail_ +1)%I2C_BUFFER_SIZE;
    }

    // Increment the buffer head for next call
    buffer_head_ = (buffer_head_ + 1)%I2C_BUFFER_SIZE;

    if(buffer_count_ == 1)
    {
        // if the buffer queue was empty, restart i2c job handling
        job_handler();
    }

    return;
}

void I2C::init_buffer()
{
    // write zeros to the buffer, and set all the indexes to zero
    memset(job_buffer_, 0, I2C_BUFFER_SIZE*sizeof(i2cJob_t));
    buffer_count_ = 0;
    buffer_head_ = 0;
    buffer_tail_ = 0;
}



//===============================================================================
// C-based IRQ functions (defined in the STD lib somewhere
extern "C" void I2C1_ER_IRQHandler(void)
{
    I2CDev_1Ptr->error_handler();
}

extern "C" void I2C1_EV_IRQHandler(void)
{
    I2CDev_1Ptr->event_handler();
}

extern "C" void I2C2_ER_IRQHandler(void)
{
    I2CDev_2Ptr->error_handler();
}

extern "C" void I2C2_EV_IRQHandler(void)
{
    I2CDev_2Ptr->event_handler();
}




#endif

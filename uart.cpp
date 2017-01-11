/*
 * uart.cpp - A UART Class for use on the naze32
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

#include "uart.h"
#include "led.h"

// Intstantiations of the three pointers to class variables for linking C-based IRQs
// to the class variables
UART* UART1Ptr;
UART* UART2Ptr;
UART* UART3Ptr;

UART::UART(){}

void UART::init(int index, uint32_t baudrate, uint8_t mode)
{
    UART_Hardware_Config_t hardware = UART_Hardware_Configuration_Array[index-1];
    // Copy pre-defined values from the Hardware Configuration Struct into the class members
    UART_Base_addr_ = hardware.UART_Peripheral;
    Tx_DMA_Channel_ = hardware.txDMAChannel;
    Rx_DMA_Channel_ = hardware.rxDMAChannel;
    TxDMAIRQ_ = hardware.TxDMAIRQ;
    RxDMAIRQ_ = hardware.RxDMAIRQ;
    UARTIRQ_ = hardware.UARTIRQ;
    DMA_TX_IT_BIT_ = hardware.DMA_IT_TX_BIT;
    DMA_RX_IT_BIT_ = hardware.DMA_IT_RX_BIT;
    baudrate_ = baudrate;
    receive_CB_ = NULL;

    if(mode == MODE_DMA_RX || mode == MODE_DMA_TX_RX)
        DMA_Rx_ = true;
    if(mode == MODE_DMA_TX || mode == MODE_DMA_TX_RX)
        DMA_Tx_ = true;

    // Link to the extern "C" IRQ functions
    if(index == 1)
        UART1Ptr = this;
    else if(index == 2)
        UART2Ptr = this;
    else if(index == 3)
        UART3Ptr = this;

    // Initialize the Circular Buffers
    rx_buffer_head_ = RX_BUFFER_SIZE; // DMA counts down on receive
    rx_buffer_tail_ = RX_BUFFER_SIZE;
    tx_buffer_head_ = 0;
    tx_buffer_tail_ = 0;

    // Set up the GPIO Pins for UART
    rx_gpio_ = GPIO(hardware.port, hardware.Rxpin, GPIO_Mode_IN_FLOATING);
    tx_gpio_ = GPIO(hardware.port, hardware.Txpin, GPIO_Mode_AF_PP);

    // Configure the USART Peripheral
    USART_Configuration();

    // Configure the DMA and associated Interrupts
    if(DMA_Rx_ || DMA_Tx_)
    {
        DMA_Configuration();
    }

//    if(!DMA_Rx_)
//    {
        USART_ITConfig(UART_Base_addr_, USART_IT_RXNE, ENABLE);
//    }

        //    if(!DMA_Tx_)
        //    {
        //        // TXE will be enabled when we go to send data
        //    }
    // Initialize the Appropriate Interrupts
    NVIC_Configuration();
}



void UART::USART_Configuration()
{
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = baudrate_;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* USART configuration */
    USART_Init(UART_Base_addr_, &USART_InitStructure);

    /* Enable the USART1 */
    USART_Cmd(UART_Base_addr_, ENABLE);
}



void UART::NVIC_Configuration()
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure the Priority Group to 2 bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* Configure the RX Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = UARTIRQ_;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Configure the TX DMA Interrupt */
    if(DMA_Tx_)
    {
        NVIC_InitStructure.NVIC_IRQChannel = TxDMAIRQ_;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    }
    NVIC_Init(&NVIC_InitStructure);


}

void UART::DMA_Configuration()
{
    DMA_InitTypeDef DMA_InitStructure;

    if(DMA_Tx_)
    {
        DMA_DeInit(Tx_DMA_Channel_);
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART_Base_addr_->DR;
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)tx_buffer_;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
        DMA_InitStructure.DMA_BufferSize = RX_BUFFER_SIZE;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
        DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
        DMA_Init(Tx_DMA_Channel_, &DMA_InitStructure);

        USART_DMACmd(UART_Base_addr_, USART_DMAReq_Tx, ENABLE);
        DMA_ITConfig(Tx_DMA_Channel_, DMA_IT_TC, ENABLE);
        //  DMA_Cmd(Tx_DMA_Channel_, ENABLE); // don't enable until we have something to send
    }

    if(DMA_Rx_)
    {
        DMA_DeInit(Rx_DMA_Channel_);

        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART_Base_addr_->DR;
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rx_buffer_;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
        DMA_InitStructure.DMA_BufferSize = RX_BUFFER_SIZE;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
        DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
        DMA_Init(Rx_DMA_Channel_, &DMA_InitStructure);

        USART_DMACmd(UART_Base_addr_, USART_DMAReq_Rx, ENABLE);
        DMA_ITConfig(Rx_DMA_Channel_, DMA_IT_TC, ENABLE);
        DMA_Cmd(Rx_DMA_Channel_, ENABLE);

        // set the buffer pointers to where the DMA is starting (starts at 256 and counts down)
        rx_buffer_tail_ = DMA_GetCurrDataCounter(Rx_DMA_Channel_);
        rx_buffer_head_ = DMA_GetCurrDataCounter(Rx_DMA_Channel_);
    }
}

void UART::register_receive_CB(void (*CB)(uint8_t))
{
    receive_CB_ = CB;
}

void UART::unregister_receive_CB(void)
{
    receive_CB_ = NULL;
}

void UART::IRQ_callback()
{
    if(DMA_Rx_)
    {
        // DMA took care of putting the data on the buffer
        // Just call the callback until we have not more data
        // Update the head position from the DMA
        rx_buffer_head_ =  Rx_DMA_Channel_->CNDTR;
        if(receive_CB_ != NULL)
        {
            while(rx_buffer_head_ != rx_buffer_tail_)
            {
                // read a new byte and decrement the tail
                uint8_t byte = rx_buffer_[RX_BUFFER_SIZE - rx_buffer_tail_];
                receive_CB_(byte);
                if(rx_buffer_tail_-- == 0)
                {
                    // wrap to the top if at the bottom
                    rx_buffer_tail_ = RX_BUFFER_SIZE;
                }
            }
        }
    }
    else
    {
        if(UART_Base_addr_->SR & USART_FLAG_RXNE)
        {
            // if there is a callback, then directly process the data
            if(receive_CB_ != NULL)
            {
                receive_CB_(UART_Base_addr_->DR);
            }
            else
            {

                // Put data on the circular RX buffer
                rx_buffer_[rx_buffer_head_] = UART_Base_addr_->DR;
                rx_buffer_head_ = (rx_buffer_head_ + 1) % RX_BUFFER_SIZE;
            }
        }
    }

    if(DMA_Tx_)
    {
        // clear the DMA Interrupt Flags
        DMA_ClearITPendingBit(DMA_TX_IT_BIT_);
        DMA_Cmd(Tx_DMA_Channel_, DISABLE);

        // If there is more data to be sent
        if(tx_buffer_head_ != tx_buffer_tail_)
        {
            startDMA();
        }

    }
    else
    {
        if(UART_Base_addr_->SR & USART_FLAG_TXE)
        {
            // If there is still data on the buffer
            if(tx_buffer_tail_ != tx_buffer_head_)
            {
                // Put the next byte on the data register of the USART peripheral
                UART_Base_addr_->DR = tx_buffer_[tx_buffer_tail_];
                // Move the tail to the next place on the buffer
                tx_buffer_tail_ = (tx_buffer_tail_ + 1) % TX_BUFFER_SIZE;
            }
            else
            {
                // We are done sending data, turn off the TX (only way to stop TXE interrupts)
                USART_ITConfig(UART_Base_addr_, USART_IT_TXE, DISABLE);
            }
        }
    }
}



void UART::put_byte(uint8_t* ch, uint32_t len)
{
    // Put Data on the tx_buffer
    for(int i = 0; i < len ; i++)
    {
        tx_buffer_[tx_buffer_head_] = ch[i];
        tx_buffer_head_ = (tx_buffer_head_ + 1) % TX_BUFFER_SIZE;
    }

    // Start a DMA Transmission
    if(DMA_Tx_)
    {
        if(!(Tx_DMA_Channel_->CCR & 1)) // If the DMA is disabled
        {
            startDMA();
        }
    }

    // Start an interrupt-driven USART transmission
    else
    {
        USART_ITConfig(UART_Base_addr_, USART_IT_TXE, ENABLE);
    }
}

void UART::startDMA()
{
    // Set the start of the transmission to the oldest data
    Tx_DMA_Channel_->CMAR = (uint32_t)&tx_buffer_[tx_buffer_tail_];
    if(tx_buffer_head_ > tx_buffer_tail_)
    {
        // Set the length of the transmission to the data on the buffer
        // if contiguous, this is easy
        Tx_DMA_Channel_->CNDTR = tx_buffer_head_ - tx_buffer_tail_;
        tx_buffer_tail_ = tx_buffer_head_;
    }
    else
    {
        // We will have to send the data in two groups, first the tail,
        // then the head we will do later
        Tx_DMA_Channel_->CNDTR = TX_BUFFER_SIZE - tx_buffer_tail_;
        tx_buffer_tail_ = 0;
    }
    // Start the Transmission
    DMA_Cmd(Tx_DMA_Channel_, ENABLE);
}

uint8_t UART::read_byte()
{
    uint8_t byte = 0;
    if(DMA_Rx_)
    {
        // pull the next byte off the array
        // (the head counts down, because CNTR counts down)
        if(rx_buffer_head_ != rx_buffer_tail_)
        {
            // read a new byte and decrement the tail
            byte = rx_buffer_[RX_BUFFER_SIZE - rx_buffer_tail_];
            if(--rx_buffer_tail_ == 0)
            {
                // wrap to the top if at the bottom
                rx_buffer_tail_ = RX_BUFFER_SIZE;
            }
        }
    }
    else
    {
        // If there is data to read
        if(rx_buffer_head_ != rx_buffer_tail_)
        {
            // pull the oldest data off the buffer
            byte = rx_buffer_[rx_buffer_tail_];
            // advance the tail
            rx_buffer_tail_ = (rx_buffer_tail_ + 1) % RX_BUFFER_SIZE;
        }
    }
    return byte;
}

bool UART::bytes_waiting()
{
    return rx_buffer_head_ != rx_buffer_tail_;
}

uint32_t UART::num_bytes_waiting()
{
    if(DMA_Rx_)
    {
        // Remember, the DMA CNDTR counts down
        rx_buffer_head_ = Rx_DMA_Channel_->CNDTR;
        if(rx_buffer_head_ < rx_buffer_tail_)
        {
            // Easy, becasue it's contiguous
            return rx_buffer_tail_ - rx_buffer_head_;
        }
        else if(rx_buffer_head_ > rx_buffer_tail_)
        {
            // Add the parts on either end of the buffer
            // I'm pretty sure this is wrong
            return rx_buffer_tail_ + RX_BUFFER_SIZE - rx_buffer_head_;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        if(rx_buffer_head_ == rx_buffer_tail_)
        {
            // no new bytes
            return 0;
        }
        else if(rx_buffer_head_ > rx_buffer_tail_)
        {
            // easy, because it's contiguous
            return rx_buffer_head_ - rx_buffer_tail_;
        }
        else
        {
            // count the part up to the end of the buffer
            uint32_t bytes_to_end = RX_BUFFER_SIZE - rx_buffer_tail_;
            // add it to the placement of the head
            return bytes_to_end + rx_buffer_head_;
        }
    }
}

//=========================================================================
// Handlers for the various IRQ routines
// These are all in C, so we have to link to the C++ objects
extern "C"
{

void USART1_IRQHandler(void)
{
    UART1Ptr->IRQ_callback();
}
void USART2_IRQHandler(void)
{
    UART2Ptr->IRQ_callback();
}
void USART3_IRQHandler(void)
{
    UART3Ptr->IRQ_callback();
}
void DMA1_Channel2_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TE2))
        DMA_ClearITPendingBit(DMA1_IT_TE2);
    if(DMA_GetITStatus(DMA1_IT_HT2))
        DMA_ClearITPendingBit(DMA1_IT_HT2);
    if(DMA_GetFlagStatus(DMA1_IT_TC2))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC2);
        UART3Ptr->IRQ_callback();
    }
}
void DMA1_Channel3_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TE3))
        DMA_ClearITPendingBit(DMA1_IT_TE3);
    if(DMA_GetITStatus(DMA1_IT_HT3))
        DMA_ClearITPendingBit(DMA1_IT_HT3);
    if(DMA_GetFlagStatus(DMA1_IT_TC3))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC3);
        UART3Ptr->IRQ_callback();
    }
}
void DMA1_Channel4_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TE4))
        DMA_ClearITPendingBit(DMA1_IT_TE4);
    if(DMA_GetITStatus(DMA1_IT_HT4))
        DMA_ClearITPendingBit(DMA1_IT_HT4);
    if(DMA_GetFlagStatus(DMA1_IT_TC4))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC4);
        UART1Ptr->IRQ_callback();;
    }
}
void DMA1_Channel5_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TE5))
        DMA_ClearITPendingBit(DMA1_IT_TE5);
    if(DMA_GetITStatus(DMA1_IT_HT5))
        DMA_ClearITPendingBit(DMA1_IT_HT5);
    if(DMA_GetFlagStatus(DMA1_IT_TC5))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC5);
        UART1Ptr->IRQ_callback();
    }
}
void DMA1_Channel6_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TE6))
        DMA_ClearITPendingBit(DMA1_IT_TE6);
    if(DMA_GetITStatus(DMA1_IT_HT6))
        DMA_ClearITPendingBit(DMA1_IT_HT6);
    if(DMA_GetFlagStatus(DMA1_IT_TC6))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC6);
        UART2Ptr->IRQ_callback();
    }
}
void DMA1_Channel7_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TE7))
        DMA_ClearITPendingBit(DMA1_IT_TE7);
    if(DMA_GetITStatus(DMA1_IT_HT7))
        DMA_ClearITPendingBit(DMA1_IT_HT7);
    if(DMA_GetFlagStatus(DMA1_IT_TC7))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC7);
        UART2Ptr->IRQ_callback();
    }
}

} // end C-code

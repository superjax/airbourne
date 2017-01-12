/*
 * board.cpp - Hardware Configuration Methods for Naze32 Hardware
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

#include "board.h"
#include "led.h"
#include "stm32f10x_conf.h"

#define AFIO_MAPR_SWJ_CFG_NO_JTAG_SW            (0x2 << 24)

void enablePeripherals()
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 |
                         RCC_APB1Periph_USART2 | RCC_APB1Periph_USART3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1 |
                         RCC_APB2Periph_USART1, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_ClearFlag();

  // Remap the JTAG ports to be used as LEDs
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_NO_JTAG_SW;
}

void enableInterrupts()
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

static volatile uint32_t us_per_tick = 0;
static volatile uint32_t num_sys_ticks = 0;

void OSCinit()
{
    // Use convenience function -> This sets the Processor
    // Frequency to 80 MHz if possible. Otherwise it sets it
    // to 64 MHz or 72 MHz, depending on how stable the external
    // oscillator is
    SetSysClock(true);
}

void startWallClock()
{    
  RCC_ClocksTypeDef clocks;
  RCC_GetClocksFreq(&clocks);
  us_per_tick = clocks.SYSCLK_Frequency / 1000000;
  SysTick_Config(clocks.HCLK_Frequency / 1000);
}

extern "C" void SysTick_Handler(void)
{
    num_sys_ticks++;
}

uint64_t micros(void)
{
    static uint64_t last_us, last_cycle_cnt, last_ms;
    volatile register uint64_t ms, cycle_cnt;
    do {
        ms = num_sys_ticks;
        cycle_cnt = SysTick->VAL;
    } while (ms != num_sys_ticks);
    uint64_t us = (ms * 1000) + (us_per_tick * 1000 - cycle_cnt) / us_per_tick;

    if(last_us > us)
    {
        volatile int error = 0;
        // we must have missed a SysTick interrupt somehow

    }

    last_us = us;
    last_cycle_cnt = cycle_cnt;
    last_ms = ms;
    return us;
}

uint32_t millis()
{
  return num_sys_ticks;
}

void delay_ms(uint32_t t)
{
  uint32_t end;
  end = millis() + t;
  while(millis() < end)
  {
    ;
  }
}

void delay_us(uint32_t t)
{
  uint64_t end;
  end = micros() + t;
  while(micros() < end);
}

void reboot()
{
    NVIC_SystemReset();
}

void rebootToBootloader()
{
     *((uint32_t *)0x20004FF0) = 0xDEADBEEF;

    reboot();
}

pwm_hardware_struct_t pwm_hardware[6] =
{
	{ GPIOA, GPIO_Pin_8,  TIM1, TIM_Channel_1, TIM1_CC_IRQn },
	{ GPIOA, GPIO_Pin_11, TIM1, TIM_Channel_4, TIM1_CC_IRQn },
	{ GPIOB, GPIO_Pin_6,  TIM4, TIM_Channel_1, TIM4_IRQn },
	{ GPIOB, GPIO_Pin_7,  TIM4, TIM_Channel_2, TIM4_IRQn },
	{ GPIOB, GPIO_Pin_8,  TIM4, TIM_Channel_3, TIM4_IRQn },
	{ GPIOB, GPIO_Pin_9,  TIM4, TIM_Channel_4, TIM4_IRQn },
};
//=============================================
// RC Configuration
pwm_hardware_struct_t rc_hardware[8] =
{
    {GPIOA, GPIO_Pin_0, TIM2, TIM_Channel_1, TIM2_IRQn},
	{GPIOA, GPIO_Pin_1, TIM2, TIM_Channel_2, TIM2_IRQn},
	{GPIOA, GPIO_Pin_2, TIM2, TIM_Channel_3, TIM2_IRQn},
	{GPIOA, GPIO_Pin_3, TIM2, TIM_Channel_4, TIM2_IRQn},
	{GPIOA, GPIO_Pin_6, TIM3, TIM_Channel_1, TIM3_IRQn},
	{GPIOA, GPIO_Pin_7, TIM3, TIM_Channel_2, TIM3_IRQn},
	{GPIOB, GPIO_Pin_0, TIM3, TIM_Channel_3, TIM3_IRQn},
	{GPIOB, GPIO_Pin_1, TIM3, TIM_Channel_4, TIM3_IRQn},
};

UART_Hardware_Config_t UART_Hardware_Configuration_Array[3] =
{
  {USART1, GPIOA, GPIO_Pin_9, GPIO_Pin_10, DMA1_Channel4, DMA1_Channel5,
   DMA1_Channel4_IRQn, DMA1_Channel5_IRQn, USART1_IRQn, DMA1_IT_TC4, DMA1_IT_TC5},
  {USART2, GPIOA, GPIO_Pin_2, GPIO_Pin_3,  DMA1_Channel7, DMA1_Channel6,
   DMA1_Channel7_IRQn, DMA1_Channel6_IRQn, USART2_IRQn, DMA1_IT_TC7, DMA1_IT_TC6},
  {USART3, GPIOB, GPIO_Pin_10, GPIO_Pin_11, DMA1_Channel2, DMA1_Channel3,
   DMA1_Channel2_IRQn, DMA1_Channel3_IRQn, USART3_IRQn, DMA1_IT_TC2, DMA1_IT_TC3},
};

//I2C_Hardware_Config_t I2C_Hardware_Configuration_Array[2] =
//{
//  {I2C1, GPIOB, GPIO_Pin_6, GPIO_Pin_7, I2C1_EV_IRQn, I2C1_ER_IRQn, RCC_APB1Periph_I2C1},
//  {I2C2, GPIOB, GPIO_Pin_10, GPIO_Pin_11, I2C2_EV_IRQn, I2C2_ER_IRQn, RCC_APB1Periph_I2C2}
//};

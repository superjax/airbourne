/*
 * board.h - Hardware Configuration File for naze32 hardware
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

#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include <cstddef>
#include "stm32f10x_conf.h"

//============================================
// LED CONFIGURATION
#define LED0_PORT GPIOB
#define LED0_PIN  GPIO_Pin_3
#define LED1_PORT GPIOB
#define LED1_PIN GPIO_Pin_4

//============================================
// Oscillator Configuration
void OSCinit(void);

//============================================
// Peripheral Configuration
void enablePeripherals(void);
void enableInterrupts(void);

//============================================
// TIMING CONFIGURATION
void startWallClock(void);
uint64_t micros();
uint32_t millis();
void delay_ms(uint32_t t);
void delay_us(uint32_t t);

//===========================================
// System Functions
void reboot();
void rebootToBootloader();

//============================================
// UART CONFIGURATION
typedef struct
{
  USART_TypeDef* UART_Peripheral;
  GPIO_TypeDef* port;
  uint32_t Txpin;
  uint32_t Rxpin;
  DMA_Channel_TypeDef *txDMAChannel;
  DMA_Channel_TypeDef *rxDMAChannel;
  IRQn_Type TxDMAIRQ;
  IRQn_Type RxDMAIRQ;
  IRQn_Type UARTIRQ;
  uint32_t DMA_IT_TX_BIT;
  uint32_t DMA_IT_RX_BIT;
} UART_Hardware_Config_t;

extern UART_Hardware_Config_t UART_Hardware_Configuration_Array[3];

//============================================
// I2C CONFIGURATION
#define NUM_I2C_DEVICES 2
#define I2C_BUFFER_SIZE 64


#define I2C1_GPIO GPIOB
#define I2C1_SCL_PIN GPIO_Pin_6
#define I2C1_SDA_PIN GPIO_Pin_7

#define I2C2_GPIO GPIOB
#define I2C2_SCL_PIN GPIO_Pin_10
#define I2C2_SDA_PIN GPIO_Pin_11

//==============================================
// IMU Configuration
#define IMU_EXTERNAL_INTERRUPT_LINE EXTI_Line13
#define IMU_EXTERNAL_INTERRUPT_PORT GPIOB
#define IMU_EXTERNAL_INTERRUPT_PIN GPIO_Pin_13
#define IMU_EXTERNAL_INTERRUPT_PINSOURCE GPIO_PinSource13
#define IMU_EXTERNAL_INTERRUPT_PORTSOURCE GPIO_PortSourceGPIOB
#define IMU_EXTI_IRQn EXTI15_10_IRQn
#define IMU_EXTI_IRQHandler EXTI15_10_IRQHandler

//=============================================
// Magnetometer Configuration
#define MAG_DRDY_GPIO GPIOB
#define MAG_DRDY_PIN GPIO_Pin_12
#define MAG_EXTERNAL_INTERRUPT_LINE EXTI_Line12
#define MAG_EXTERNAL_INTERRUPT_PINSOURCE GPIO_PinSource12
#define MAG_EXTERNAL_INTERRUPT_PORTSOURCE GPIO_PortSourceGPIOB
#define MAG_EXTI_IRQn EXTI15_10_IRQn

//=============================================
// PWM Configuration
#define NUM_PWM_OUTPUTS 6
#define NUM_RC_INPUTS 8
typedef struct{
  GPIO_TypeDef* GPIO;
  uint16_t pin;
  TIM_TypeDef* TIM;
  uint8_t channel;
  uint8_t IQR_Channel;
} pwm_hardware_struct_t;

extern pwm_hardware_struct_t pwm_hardware[NUM_PWM_OUTPUTS];
extern pwm_hardware_struct_t rc_hardware[NUM_RC_INPUTS];

//============================================
// RC Configuration
#define DSM_UART_PORT 2
#define PPM_CHANNEL 1
#define MAX_US 2000
#define MIN_US 1000

//============================================
// EEPROM CONFIGURATION
#define FLASH_PAGE_COUNT 128
#define FLASH_PAGE_SIZE ((uint16_t)0x0400)
#define CONFIG_SIZE (FLASH_PAGE_SIZE * 3)
#define FLASH_WRITE_ADDR (0x08000000 + (FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - (CONFIG_SIZE / 1024))))

// EEPROM_VERSION should be re-created at compile time by makes with the git hash and timestamp
#ifndef EEPROM_VERSION
#define EEPROM_VERSION 0
#endif






#endif // BOARD_H

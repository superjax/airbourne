/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <stdint.h>
#pragma once

#include "stm32f4xx_conf.h"

#ifndef TARGET_REVO
#define TARGET_REVO
#endif

#ifdef TARGET_REVO
#include "revo_f4.h"
#endif


// Define the hardware configuration structs.  (These are instantiated in the board-specific c-file)
typedef struct
{
  GPIO_TypeDef* GPIO;
  uint16_t pin;
} LED_configuration_t;
extern LED_configuration_t led_config[NUM_LED];

typedef struct
{
  uint8_t serial_type;
  GPIO_TypeDef* GPIO;
  uint16_t rx_pin;
  uint16_t tx_pin;
  USART_TypeDef* USARTx;
  uint32_t rxDMAIrq;
  uint32_t txDMAIrq;
  uint32_t rxDMAPos;
  DMA_Stream_TypeDef *rxDMAStream;
  DMA_Stream_TypeDef *txDMAStream;
  uint32_t rxDMAChannel;
  uint32_t txDMAChannel;
  uint32_t txDMAPeripheralBaseAddr;
  uint32_t rxDMAPeripheralBaseAddr;
} serial_configuration_t;
extern serial_configuration_t serial_config[NUM_SERIAL_CONNECTIONS];


#ifdef __cplusplus
extern "C" {
#endif

void systemInit(void);
void delayMicroseconds(uint32_t us);
void delay(uint32_t ms);

uint32_t micros(void);
uint32_t millis(void);

// bootloader/IAP
void systemReset(void);
void systemResetToBootloader(void);




#ifdef __cplusplus
}
#endif

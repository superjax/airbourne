#include "system.h"

LED_configuration_t led_config[NUM_LED] = {{GPIOB, GPIO_Pin_5}};

// From Serial.h UART = 0, VCP = 1
serial_configuration_t serial_config[NUM_SERIAL_CONNECTIONS] =
{
  {1, GPIOA, GPIO_Pin_11, GPIO_Pin_12}
};

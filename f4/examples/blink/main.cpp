
#include "stm32f4xx.h"
#include "system.h"
#include "gpio.h"

int main()
{
  systemInit();

  GPIO LED1(LED1_GPIO, LED1_PIN, GPIO::OUTPUT);
  GPIO LED2(LED2_GPIO, LED2_PIN, GPIO::OUTPUT);

  while(1)
  {
    delay(200);
    LED1.toggle();
    LED2.toggle();
  }
}

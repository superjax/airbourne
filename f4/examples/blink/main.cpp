#include "system.h"


int main()
{
  systemInit();

  while(1)
  {
    delay(200);
    volatile int debug = 1;
  }
}

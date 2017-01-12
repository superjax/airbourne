#include "airbourne.h"

void airbourne_init()
{
    SystemInit();
    OSCinit();
    enableInterrupts();
    enablePeripherals();
    startWallClock();
    initLED();
}

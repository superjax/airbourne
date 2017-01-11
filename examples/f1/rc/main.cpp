#include "board.h"
#include "rc.h"

int main()
{
    SystemInit();
    OSCinit();
    enableInterrupts();
    enablePeripherals();
    startWallClock();

    RC_DSM rc;

    UART uart1, uart2, uart3;
    uart1.init(1, 115200, UART::MODE_DMA_TX_RX);
    uart2.init(2, 115200, UART::MODE_DMA_TX_RX);
    uart3.init(3, 115200, UART::MODE_DMA_TX_RX);

//    rc.init(RC_DSM::DSM_2048);

    while(1)
    {
        volatile int wait = 0;
    }

}

#include <airbourne.h>

UART serial;

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    serial.put_byte((uint8_t*)&c, 1);
}

int main()
{
    airbourne_init();

    serial.init(1, 115200, UART::MODE_DMA_TX_RX);
    init_printf(NULL, _putc);

    RC* rc;
    RC_DSM rc_dsm;
    RC_PPM rc_ppm;
    uint8_t mode = RC::PPM;

    switch(mode)
    {
    case RC::DSM:
        rc = & rc_dsm;
        break;
    case RC::PPM:
        rc = & rc_ppm;
        break;
    }

    rc->init();

    uint16_t read_rc[8];

    while(1)
    {
        for(int i = 0; i < 8; i++)
        {
            read_rc[i] = rc->readus(i);
            printf("%d\t", read_rc[i]);
        }
        delay_ms(100);
        printf("\n");
    }


}

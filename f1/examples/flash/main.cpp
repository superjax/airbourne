#include "board.h"
#include "eeprom.h"

int main()
{
    SystemInit();
    OSCinit();
    enableInterrupts();
    enablePeripherals();
    startWallClock();

    uint8_t data[10];

    for(int i = 0; i < 10; i++)
    {
        data[i] = 0x50 + i;
    }

    EEPROM eeprom;

    uint8_t read_data[10];
    for (int i = 0; i < 10; i++)
    {
        read_data[i] = 0;
    }
    bool success = false;

    success = eeprom.read(read_data, 10);

    success = eeprom.write(data, 10);

    reboot();

    while(1)
    {
        ;
    }

}

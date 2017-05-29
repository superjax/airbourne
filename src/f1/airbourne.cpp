#include "airbourne.h"

LED _LED0;
LED _LED1;

// UARTs
UART _uart1;

// I2C
I2C _i2c;

// Sensors
MPU6050 _mpu6050;
HMC5883L _hmc5883l;
MS5611 _ms5611;

// Motors <--- we probably need a smarter way to allocate these.
//             I don't think that they take up too much room for now
PWM_Out* _motors[13];
Servo _servos[13];
SimonK_ESC _simonk[13];

// RC
RC_DSM _rc_dsm;
RC_PPM _rc_ppm;
RC* _rc;

EEPROM _eeprom;

void airbourne_init()
{
    SystemInit();
    OSCinit();
    enableInterrupts();
    enablePeripherals();
    startWallClock();

    _LED0 = LED(LED0_PORT, LED0_PIN);
    _LED1 = LED(LED1_PORT, LED1_PIN);
}



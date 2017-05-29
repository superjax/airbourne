#include "servo.h"


void Servo::init(uint8_t pin)
{
    init(pin, 50, MAX_US, MIN_US);
}

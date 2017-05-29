#include "simonk_esc.h"

void SimonK_ESC::init(uint8_t pin)
{
    init(pin, 490, MAX_US, MIN_US);
}

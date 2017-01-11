#ifndef BAROMETER_H
#define BAROMETER_H

#include "i2c.h"

class Barometer
{
public:
    Barometer() {}
    virtual bool init(I2C* I2CDev) = 0;
    virtual void update() = 0;
    virtual void read(float* pressure, float* temperature) = 0;
};

#endif // BAROMETER_H

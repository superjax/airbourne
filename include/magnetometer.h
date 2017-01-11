#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include "lib/Vector/vector3.h"

class Magnetometer
{
public:
    Magnetometer(){}
    virtual bool init(I2C *I2CDev) = 0;
    virtual void read(vector_t *mag) = 0;
    virtual bool new_data() = 0;
    virtual void update() = 0;
};

#endif // MAGNETOMETER_H

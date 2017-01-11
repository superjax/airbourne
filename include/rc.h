#ifndef RC_H
#define RC_H

#include <board.h>
#include <stdint.h>

class RC
{
public:
    typedef enum
    {
        PPM,
        DSM2,
        DSMX
    } mode_t;

    RC() {}

    virtual void init() = 0;

    virtual uint32_t readus(uint8_t channel) = 0;
    virtual float read(uint8_t channel) = 0;
};


#include "rc_dsm.h"
#include "rc_ppm.h"

#endif // RC_H

#ifndef SERIAL_CLASS_H
#define SERIAL_CLASS_H

#include <stdint.h>


class Serial
{
public:

  Serial();
  virtual void put_byte(uint8_t* ch, uint32_t len) = 0;
  virtual uint8_t read_byte() = 0;
  virtual bool bytes_waiting() = 0;

};

#endif // SERIAL CLASS_H

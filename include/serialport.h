#ifndef SERIAL_CLASS_H
#define SERIAL_CLASS_H

#include <stdint.h>


class Serial
{
public:

  Serial();
  void put_byte(uint8_t* ch, uint32_t len){}
  uint8_t read_byte(){}
  bool bytes_waiting(){}

};

#endif // SERIAL CLASS_H

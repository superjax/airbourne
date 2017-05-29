#include "system.h"
# include "Serial.h"


class VCP : Serial
{
public:
  VCP(serial_configuration_t* config);
  void put_byte(uint8_t* ch, uint32_t len);
  uint8_t read_byte();
  bool bytes_waiting();

private:
  uint32_t baud_;
  uint8_t  stop_bits_;
  uint8_t data_bits_;
  uint8_t parity_;

};

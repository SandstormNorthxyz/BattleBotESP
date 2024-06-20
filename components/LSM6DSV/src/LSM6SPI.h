#include <Arduino.h>
#include "driver/spi_master.h"

namespace LSM6DSV{

  void initSPI();
  void init();
  uint8_t readReg(uint8_t reg);
  void writeReg(uint8_t reg, uint8_t val);
  void readRawData();
  uint16_t getJerk();

}
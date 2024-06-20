#include "Arduino.h"

#define GPIO(x) GPIO_NUM_##x

#define LED GPIO(16)
#define BATT_DIV GPIO(5)
#define ESC_PWM GPIO(37)
#define RADIO_RX GPIO(35)
#define RADIO_TX GPIO(36)

#define DRV_ARM GPIO(18)
#define SPI_CLK GPIO(43)
#define SPI_PTX GPIO(1)
#define SPI_CTX GPIO(2)

#define M1_CS GPIO(44)
#define M1_DRVOFF GPIO(15)
#define M1_IN1 GPIO(7)
#define M1_IN2 GPIO(6)
#define M1_ISENSE GPIO(4)
#define M1_FAULT GPIO(47)
#define M1_ENC GPIO(38)

#define M2_CS GPIO(42)
#define M2_DRVOFF GPIO(3)
#define M2_IN1 GPIO(21)
#define M2_IN2 GPIO(17)
#define M2_ISENSE GPIO(8)
#define M2_FAULT GPIO(48)
#define M2_ENC GPIO(41)

#define LSM6_ADDR ((uint8_t) 0x6B)
#define LSM6_SCL GPIO(14)
#define LSM6_SDA GPIO(13)
#define LSM6_SDO GPIO(12)
#define LSM6_CS GPIO(11)
#define LSM6_ODR GPIO(10)
#define LSM6_INT GPIO(9)

#define EEPROM_ADDR ((uint8_t) 0x05)
#define EEPROM_SCL GPIO(39)
#define EEPROM_SDA GPIO(40)

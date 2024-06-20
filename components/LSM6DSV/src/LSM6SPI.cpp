#include "LSM6SPI.h"
#include "PINDEFS.h"
#include "LSM6Registers.h"

namespace LSM6DSV {
  esp_err_t ret;
  spi_device_handle_t LSM6SPI;

  void initSPI() {
    spi_bus_config_t LSM6SPIBusConfig = {
        .mosi_io_num = LSM6_SDA,
        .miso_io_num = LSM6_SDO,
        .sclk_io_num = LSM6_SCL,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    ret = spi_bus_initialize(SPI3_HOST, &LSM6SPIBusConfig, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t LSM6SPIConfig = {
        .command_bits = 1,
        .address_bits = 7,
        .mode = 3,
        .cs_ena_posttrans = 4, //cut down if this starts to become an issue, should be able to go all the way to 0
        .clock_speed_hz = 8 * 1000 * 1000,
        .spics_io_num = LSM6_CS,
        .queue_size = 1,
        .pre_cb = NULL, //or NULL
        .post_cb = NULL, //or NULL
    };
    ret = spi_bus_add_device(SPI3_HOST, &LSM6SPIConfig, &LSM6SPI);
    ESP_ERROR_CHECK(ret);

  }

  void init(){
    LSM6DSV::writeReg(CTRL3_C, 0b01000100);
    LSM6DSV::writeReg(CTRL1_XL, 0x0B);
    LSM6DSV::writeReg(CTRL2_G, 0x0B);
  }

  uint8_t readReg(uint8_t reg) {
    static uint32_t rxBuffer = 0;

    spi_transaction_t LSM6Trans = {
        .flags = 0,
        .cmd = 0x1,
        .addr = reg,
        .length = 8,
        .rxlength = 8,
        .tx_buffer = NULL,
        .rx_buffer = &rxBuffer,
    };

    ret = spi_device_transmit(LSM6SPI, &LSM6Trans);
    ESP_ERROR_CHECK(ret);

    return SPI_SWAP_DATA_RX(rxBuffer, 8);
  }

  void writeReg(uint8_t reg, uint8_t val) {
    static uint32_t txBuffer = 0;

    spi_transaction_t LSM6Trans = {
        .flags = 0,
        .cmd = 0x0,
        .addr = reg,
        .length = 8,
        .rxlength = 0,
        .tx_buffer = &txBuffer,
        .rx_buffer = NULL,
    };

    txBuffer = SPI_SWAP_DATA_TX(val, 8);

    ret = spi_device_transmit(LSM6SPI, &LSM6Trans);
    ESP_ERROR_CHECK(ret);

  }

  void readRawData(){
    uint8_t data[12] = {0};

    spi_transaction_t LSM6TransData = {
        .flags = 0,
        .cmd = 0x1,
        .addr = OUTX_L_G,
        .length = 8 * 12,
        .rxlength = 8 * 12,
        .tx_buffer = NULL,
        .rx_buffer = &data,
    };

    ret = spi_device_transmit(LSM6SPI, &LSM6TransData);
    ESP_ERROR_CHECK(ret);

    int16_t gyroData[3];
    int16_t accelData[3];


    //concatenate high and low bytes of each int16 from bytestream tx char buffer,
    // then swap bit order back to little-endian since SPI transmits as big-endian
    gyroData[0] = SPI_SWAP_DATA_RX(data[0] << 8 | data[1], 16);
    gyroData[1] = SPI_SWAP_DATA_RX(data[2] << 8 | data[3], 16);
    gyroData[2] = SPI_SWAP_DATA_RX(data[4] << 8 | data[5], 16);

    accelData[0] = SPI_SWAP_DATA_RX(data[6] << 8 | data[7], 16);
    accelData[1] = SPI_SWAP_DATA_RX(data[8] << 8 | data[9], 16);
    accelData[2] = SPI_SWAP_DATA_RX(data[10] << 8 | data[11], 16);

    if(Serial.availableForWrite()) {

      Serial.print("\tGyro: ");
      Serial.print(gyroData[0]);
      Serial.print("\t");
      Serial.print(gyroData[1]);
      Serial.print("\t");
      Serial.print(gyroData[2]);

      Serial.print("\t\tAccel: ");
      Serial.print(accelData[0]);
      Serial.print("\t");
      Serial.print(accelData[1]);
      Serial.print("\t");
      Serial.print(accelData[2]);


      Serial.println();
    }
  }

  uint16_t getJerk(){
    static uint16_t prevAccX = 0;
    static uint16_t prevAccY = 0;
    static uint16_t prevAccZ = 0;

    uint8_t data[12] = {0};

    spi_transaction_t LSM6TransData = {
        .flags = 0,
        .cmd = 0x1,
        .addr = OUTX_L_G,
        .length = 8 * 12,
        .rxlength = 8 * 12,
        .tx_buffer = NULL,
        .rx_buffer = &data,
    };

    ret = spi_device_transmit(LSM6SPI, &LSM6TransData);
    ESP_ERROR_CHECK(ret);

    int16_t gyroData[3];
    int16_t accelData[3];


    //concatenate high and low bytes of each int16 from bytestream tx char buffer,
    // then swap bit order back to little-endian since SPI transmits as big-endian
    gyroData[0] = SPI_SWAP_DATA_RX(data[0] << 8 | data[1], 16);
    gyroData[1] = SPI_SWAP_DATA_RX(data[2] << 8 | data[3], 16);
    gyroData[2] = SPI_SWAP_DATA_RX(data[4] << 8 | data[5], 16);

    accelData[0] = SPI_SWAP_DATA_RX(data[6] << 8 | data[7], 16);
    accelData[1] = SPI_SWAP_DATA_RX(data[8] << 8 | data[9], 16);
    accelData[2] = SPI_SWAP_DATA_RX(data[10] << 8 | data[11], 16);

    uint16_t jerkX = accelData[0] - prevAccX;
    uint16_t jerkY = accelData[1] - prevAccY;
    uint16_t jerkZ = accelData[2] - prevAccZ;

    prevAccX = accelData[0];
    prevAccY = accelData[1];
    prevAccZ = accelData[2];

    return sqrt(pow(jerkX, 2) + pow(jerkY, 2) + pow(jerkZ, 2));
  }
}
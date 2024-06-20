// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"

#include <Arduino.h>
#include <Bluepad32.h>
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "hal/ledc_types.h"
//#include "../components/BetterPWM/src/BetterPWM.h"
#include "../components/LSM6DSV/src/LSM6SPI.h"
#include <soc/adc_channel.h>
#include "driver/adc.h"


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

#define MOTOR_TIMER 1

#define M1_CS GPIO(44)
#define M1_DRVOFF GPIO(15)
#define M1_IN1 GPIO(7)
#define M1_IN2 GPIO(6)
#define M1_IN1_CHANNEL 0
#define M1_IN2_CHANNEL 1
#define M1_ISENSE GPIO(4)
#define M1_ISENSE_ADC ADC1_GPIO4_CHANNEL
#define M1_FAULT GPIO(47)
#define M1_ENC GPIO(38)

#define M2_CS GPIO(42)
#define M2_DRVOFF GPIO(3)
#define M2_IN1 GPIO(21)
#define M2_IN2 GPIO(17)
#define M2_IN1_CHANNEL 2
#define M2_IN2_CHANNEL 3
#define M2_ISENSE GPIO(8)
#define M2_ISENSE_ADC ADC1_GPIO8_CHANNEL
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

//
// README FIRST, README FIRST, README FIRST
//
// Bluepad32 has a built-in interactive console.
// By default, it is enabled (hey, this is a great feature!).
// But it is incompatible with Arduino "Serial" class.
//
// Instead of using, "Serial" you can use Bluepad32 "Console" class instead.
// It is somewhat similar to Serial but not exactly the same.
//
// Should you want to still use "Serial", you have to disable the Bluepad32's console
// from "sdkconfig.defaults" with:
//    CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE=n

ControllerPtr myController;
bool commsGood = false;

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    if (myController == nullptr) {
        Console.printf("CALLBACK: Controller is connected\n");
        // Additionally, you can get certain gamepad properties like:
        // Model, VID, PID, BTAddr, flags, etc.
        ControllerProperties properties = ctl->getProperties();
        Console.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName(), properties.vendor_id,
                       properties.product_id);
        myController = ctl;
        commsGood = true;
    }else {
        Console.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {

    if(myController == ctl) {
        Console.printf("CALLBACK: Controller disconnected from index\n");
        myController = nullptr;
        commsGood = false;
    }else {
        Console.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Console.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    if (ctl->a()) {
        static int colorIdx = 0;
        // Some gamepads like DS4 and DualSense support changing the color LED.
        // It is possible to change it by calling:
        switch (colorIdx % 3) {
            case 0:
                // Red
                ctl->setColorLED(255, 0, 0);
                break;
            case 1:
                // Green
                ctl->setColorLED(0, 255, 0);
                break;
            case 2:
                // Blue
                ctl->setColorLED(0, 0, 255);
                break;
        }
        colorIdx++;
    }

    if (ctl->b()) {
        // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
        // support changing the "Player LEDs": those 4 LEDs that usually indicate
        // the "gamepad seat".
        // It is possible to change them by calling:
        ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */,
                            0xFF /* strongMagnitude */);
    }

    if (ctl->x()) {
        // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S, Stadia support rumble.
        // It is possible to set it by calling:
        // Some controllers have two motors: "strong motor", "weak motor".
        // It is possible to control them independently.
        ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */,
                            0x40 /* strongMagnitude */);
    }

    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
//    dumpGamepad(ctl);

    double drive = ctl->axisY() / 512.0;
    double turn = ctl->axisRX() / 512.0;
    turn *= abs(drive) * .5 + .5;
    double m1Pow = constrain(drive + turn, -1, 1);
    double m2Pow = constrain(drive - turn, -1, 1);

//    Console.print("Drive: ");
//    Console.print((drive * 100.0));
//    Console.print("\tTurn: ");
//    Console.print((turn * 100.0));
//    Console.println();

    digitalWrite(M1_IN2, m1Pow > 0);
    digitalWrite(M2_IN2, m2Pow < 0);
    analogWrite(M1_IN1, abs(m1Pow) > 0.1 ? abs(m1Pow) * 255.0 : 0);
    analogWrite(M2_IN1, abs(m2Pow) > 0.1 ? abs(m2Pow) * 255.0 : 0);

    // See ArduinoController.h for all the available functions.
}

void processControllers() {
    if (myController && myController->isConnected() && myController->hasData()) {
        if (myController->isGamepad()) {
            processGamepad(myController);
        } else {
            Console.printf("Unsupported controller\n");
        }
    }

}





esp_err_t ret;
spi_device_handle_t M1SPI;
spi_device_handle_t M2SPI;

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    pinMode(LED, OUTPUT);

    LSM6DSV::initSPI();
    LSM6DSV::init();

    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);

    // Enables the BLE Service in Bluepad32.
    // This service allows clients, like a mobile app, to setup and see the state of Bluepad32.
    // By default, it is disabled.
    BP32.enableBLEService(false);

//    adc1_config_width(ADC_WIDTH_BIT_12);
//    adc1_config_channel_atten(M1_ISENSE_ADC, ADC_ATTEN_DB_2_5);
//    adc1_config_channel_atten(M2_ISENSE_ADC, ADC_ATTEN_DB_2_5);


    pinMode(M2_DRVOFF, OUTPUT);
    pinMode(M1_DRVOFF, OUTPUT);
    digitalWrite(M2_DRVOFF, LOW);
    digitalWrite(M1_DRVOFF, LOW);

    pinMode(DRV_ARM, OUTPUT);
    digitalWrite(DRV_ARM, LOW);
    delay(10);
    digitalWrite(DRV_ARM, HIGH);
    delay(20);

    pinMode(M1_IN1, OUTPUT);
    pinMode(M1_IN2, OUTPUT);
    pinMode(M2_IN1, OUTPUT);
    pinMode(M2_IN2, OUTPUT);
    digitalWrite(M1_IN2, HIGH);
    digitalWrite(M2_IN2, HIGH);
    analogWrite(M1_IN1, 0);
    analogWrite(M2_IN1, 0);

    spi_bus_config_t DRV8243SPIBusConfig = {
        .mosi_io_num = SPI_CTX,
        .miso_io_num = SPI_PTX,
        .sclk_io_num = SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    ret = spi_bus_initialize(SPI2_HOST, &DRV8243SPIBusConfig, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t M1SPIConfig = {
        .command_bits = 0,
        .address_bits = 0,
        .mode = 1,
        .cs_ena_posttrans = 0, //cut down if this starts to become an issue, should be able to go all the way to 0
        .clock_speed_hz = 2 * 1000 * 1000,
        .spics_io_num = M1_CS,
        .queue_size = 1,
        .pre_cb = NULL, //or NULL
        .post_cb = NULL, //or NULL
    };
    ret = spi_bus_add_device(SPI2_HOST, &M1SPIConfig, &M1SPI);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t M2SPIConfig = {
        .command_bits = 0,
        .address_bits = 0,
        .mode = 1,
        .cs_ena_posttrans = 0, //cut down if this starts to become an issue, should be able to go all the way to 0
        .clock_speed_hz = 2 * 1000 * 1000,
        .spics_io_num = M2_CS,
        .queue_size = 1,
        .pre_cb = NULL, //or NULL
        .post_cb = NULL, //or NULL
    };
    ret = spi_bus_add_device(SPI2_HOST, &M2SPIConfig, &M2SPI);
    ESP_ERROR_CHECK(ret);

}


void readFaults(){
    static uint32_t rxBuffer = 0;
    static uint16_t txBuffer = 0;

    spi_transaction_t DRV8243Trans = {
        .flags = 0,
        .cmd = 0,
        .addr = 0,
        .length = 16,
        .rxlength = 16,
        .tx_buffer = &txBuffer,
        .rx_buffer = &rxBuffer,
    };

    txBuffer = SPI_SWAP_DATA_TX((uint16_t) 0b0100000100000000, 16);

    ret = spi_device_transmit(M2SPI, &DRV8243Trans);
    ESP_ERROR_CHECK(ret);

    Serial.println(SPI_SWAP_DATA_RX(rxBuffer, 16), BIN);
}

void clearFaults(){
    static uint32_t rxBuffer = 0;
    static uint16_t txBuffer = 0;

    spi_transaction_t DRV8243Trans = {
        .flags = 0,
        .cmd = 0,
        .addr = 0,
        .length = 16,
        .rxlength = 16,
        .tx_buffer = &txBuffer,
        .rx_buffer = &rxBuffer,
    };

    txBuffer = SPI_SWAP_DATA_TX((uint16_t) 0b0000100010000000, 16);

    ret = spi_device_transmit(M1SPI, &DRV8243Trans);
    ESP_ERROR_CHECK(ret);

    Serial.print("M1: ");
    Serial.print(SPI_SWAP_DATA_RX(rxBuffer, 16), BIN);

    txBuffer = SPI_SWAP_DATA_TX((uint16_t) 0b0000100010000000, 16);

    ret = spi_device_transmit(M2SPI, &DRV8243Trans);
    ESP_ERROR_CHECK(ret);

    Serial.print("\tM2: ");
    Serial.print(SPI_SWAP_DATA_RX(rxBuffer, 16), BIN);
    Serial.println();
}

void setPWMMode(){
    static uint32_t rxBuffer = 0;
    static uint16_t txBuffer = 0;

    spi_transaction_t DRV8243Trans = {
        .flags = 0,
        .cmd = 0,
        .addr = 0,
        .length = 16,
        .rxlength = 16,
        .tx_buffer = &txBuffer,
        .rx_buffer = &rxBuffer,
    };

    txBuffer = SPI_SWAP_DATA_TX((uint16_t) ((0b01000000 | 0x0C) << 8), 16);
    ret = spi_device_transmit(M1SPI, &DRV8243Trans);
    ESP_ERROR_CHECK(ret);
    uint8_t m1Config3 = SPI_SWAP_DATA_RX(rxBuffer, 16) & 0x00FF;

    txBuffer = SPI_SWAP_DATA_TX((uint16_t) ((0b01000000 | 0x0C) << 8), 16);
    ret = spi_device_transmit(M2SPI, &DRV8243Trans);
    ESP_ERROR_CHECK(ret);
    uint8_t m2Config3 = SPI_SWAP_DATA_RX(rxBuffer, 16) & 0x00FF;


    txBuffer = SPI_SWAP_DATA_TX((uint16_t) (((0b00000000 | 0x0C) << 8) | (m1Config3 | 0x03)), 16);
    ret = spi_device_transmit(M1SPI, &DRV8243Trans);
    ESP_ERROR_CHECK(ret);

    txBuffer = SPI_SWAP_DATA_TX((uint16_t) (((0b00000000 | 0x0C) << 8) | (m2Config3 | 0x03) ), 16);
    ret = spi_device_transmit(M2SPI, &DRV8243Trans);
    ESP_ERROR_CHECK(ret);

}

uint64_t serialTimer = millis();
// Arduino loop function. Runs in CPU 1.
void loop() {
//    gpio_set_level(LED, !gpio_get_level(LED));
    gpio_set_level(LED, commsGood);
    gpio_set_level(DRV_ARM, commsGood);

    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    clearFaults();
    readFaults();
//    setPWMMode();

//    double m1ISENSE = adc1_get_raw(M1_ISENSE_ADC);
//    double m2ISENSE = adc1_get_raw(M2_ISENSE_ADC); // / 4096.0
//
//    if(millis() - serialTimer > 20) {
//        serialTimer = millis();
//        Console.print("M1:");
//        Console.print(m1ISENSE);
//        Console.print(",M2:");
//        Console.print(m2ISENSE);
//        Console.println();
//    }

//    double jerk = ((double) LSM6DSV::getJerk()) / ((double) 0xFFFF);
//    myController->playDualRumble(0 /* delayedStartMs */, 10 /* durationMs */, jerk * 255 /* weakMagnitude */,
//                        jerk * 255 /* strongMagnitude */);

    LSM6DSV::readRawData();


    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
//    vTaskDelay(1);
    delay(10);
}

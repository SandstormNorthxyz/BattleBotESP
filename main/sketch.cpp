// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"

#include <Arduino.h>
#include <Bluepad32.h>
#include "../components/BetterPWM/src/BetterPWM.h"
#include "bt/uni_bt_allowlist.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "hal/ledc_types.h"

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
#define M1_FAULT GPIO(47)
#define M1_ENC GPIO(38)

#define M2_CS GPIO(42)
#define M2_DRVOFF GPIO(3)
#define M2_IN1 GPIO(21)
#define M2_IN2 GPIO(17)
#define M2_IN1_CHANNEL 1
#define M2_IN2_CHANNEL 3
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

#define ESC_CHANNEL 2

#define LEDC_TIMER              LEDC_TIMER_3
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (ESC_PWM) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_6
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (50) // Frequency in Hertz. Set frequency at 4 kHz

static const char * blueXboxController = "98:7A:14:07:73:2C";
static const char * blackXboxController = "0C:35:26:6F:6F:49";


double WEAPONSPEED = 0;
double weaponGood = false;
boolean r1Prev = false, l1Prev = false, flipped = false, yPrev = false, aPrev = false, bPrev = false;
int weaponState = 0;

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
        weaponState = 0;
        ledcWrite(M1_IN1_CHANNEL, 0);
        ledcWrite(M2_IN1_CHANNEL, 0);
        ledcWrite(ESC_CHANNEL, 608);
        WEAPONSPEED = 0;
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
    dumpGamepad(ctl);

    if(ctl->y() and !yPrev) weaponState = 1;
    if(ctl->a() and !aPrev) weaponState = 2;
    if(ctl->b() and !bPrev) weaponState = 0;

    yPrev = ctl->y();
    aPrev = ctl->a();
    bPrev = ctl->b();


    if(weaponState == 1) {
        WEAPONSPEED = ((((double)ctl->brake()) / 1023.0) * 57.3) + 122.8;
    }else if(weaponState == 2){
        WEAPONSPEED = -((((double)ctl->brake()) / 1023.0) * 57.3) + 122.8;
    }else{
        WEAPONSPEED = 0;
    }

    //boolean flipped = false;
    if(ctl->r1() and !r1Prev){
        flipped = false;
    }else if(ctl->l1() and !l1Prev){
        flipped = true;
    }
    r1Prev = ctl->r1();
    l1Prev = ctl->l1();
//
//
//    double drive = ctl->axisY() / 512.0;
//    double turn = ctl->axisRX() / 512.0;
//    turn *= 1.0 - (drive * .5);
//    double power = 1.0 - ((ctl->throttle() / 1023.0) * .5);
//    //power *= flipped ? -1.0 : 1.0;
//    turn *= flipped ? -1.0 : 1.0;
//    if(drive < .1 and drive > -.1) drive = 0.0;
//    if(turn < .1 and turn > -.1) turn = 0.0;
//    double m1Pow = constrain(drive - turn, -1, 1) * power;
//    double m2Pow = constrain(drive + turn, -1, 1) * power;

    int turn = constrain(ctl->axisRX(), -510, 510);
    int drive = constrain(ctl->axisY(), -510, 510);
    //turn = turn / (510 - (abs(drive) / 2));
    double power = 1.0 - ((ctl->throttle() / 1023.0) * .5);
    drive *= flipped ? -1 : 1;
    //turn *= flipped ? -1.0 : 1.0;
    if(drive < 30 and drive > -30) drive = 0;
    if(turn < 30 and turn > - 30) turn = 0;
    int m1Pow = -constrain(drive + turn, -510, 510) * power;
    int m2Pow = constrain(drive - turn, -510, 510) * power;

    digitalWrite(M1_IN2, m1Pow > 0);
    digitalWrite(M2_IN2, m2Pow > 0);
    ledcWrite(M1_IN1_CHANNEL, abs(m1Pow) > 60 ? abs(m1Pow) * 16 : 0);
    ledcWrite(M2_IN1_CHANNEL, abs(m2Pow) > 60 ? abs(m2Pow) * 16 : 0);

//    Console.println(abs(m1Pow) > 60 ? abs(m1Pow) * 16 : 0);
//    Console.println(abs(m2Pow) > 60 ? abs(m2Pow) * 16 : 0);
    //    Console.print("Drive: ");
    //    Console.print((drive * 100.0));
    //    Console.print("\tTurn: ");
    //    Console.print((turn * 100.0));
    //    Console.println();


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
//    Serial.begin(115200);
    pinMode(LED, OUTPUT);


    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

//    // "forgetBluetoothKeys()" should be called when the user performs
//    // a "device factory reset", or similar.
//    // Calling "forgetBluetoothKeys" in setup() just as an example.
//    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
//    // But it might also fix some connection / re-connection issues.
//    BP32.forgetBluetoothKeys();

    bd_addr_t blueXboxAddr;
    bd_addr_t blackXboxAddr;

    sscanf_bd_addr(blueXboxController, blueXboxAddr);
    sscanf_bd_addr(blackXboxController, blackXboxAddr);

    uni_bt_allowlist_add_addr(blueXboxAddr);
    //uni_bt_allowlist_add_addr(blackXboxAddr); //TODO reenable if need to use owencontroller
    uni_bt_allowlist_set_enabled(true);


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
    ledcSetup(M1_IN1_CHANNEL, 4000, 13);
    ledcSetup(M2_IN1_CHANNEL, 4000, 13);
    ledcAttachPin(M1_IN1, M1_IN1_CHANNEL);
    ledcAttachPin(M2_IN1, M2_IN1_CHANNEL);
    ledcWrite(M1_IN1_CHANNEL, 0);
    ledcWrite(M2_IN1_CHANNEL, 0);

    ledcSetup(ESC_CHANNEL, 50, 13);
    ledcAttachPin(ESC_PWM, ESC_CHANNEL);
    ledcWrite(ESC_CHANNEL, 0);

//    analogWrite(M1_IN1, 0);
//    analogWrite(M2_IN1, 0);


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


//      ledc_timer_config_t ledc_timer = {
//          .speed_mode       = LEDC_MODE,
//          .duty_resolution  = LEDC_DUTY_RES,
//          .timer_num        = LEDC_TIMER,
//          .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
//          .clk_cfg          = LEDC_AUTO_CLK
//      };
//      ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
//
//      // Prepare and then apply the LEDC PWM channel configuration
//      ledc_channel_config_t ledc_channel = {
//          .gpio_num       = LEDC_OUTPUT_IO,
//
//          .speed_mode     = LEDC_MODE,
//          .channel        = LEDC_CHANNEL,
//          .intr_type      = LEDC_INTR_DISABLE,
//          .timer_sel      = LEDC_TIMER,
//          .duty           = 0, // Set duty to 0%
//          .hpoint         = 0
//      };
//
//      ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

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

// Arduino loop function. Runs in CPU 1.
void loop() {
//    gpio_set_level(LED, !gpio_get_level(LED));
    gpio_set_level(LED, commsGood);
    gpio_set_level(DRV_ARM, commsGood);

    if(commsGood) {
//        Console.println(WEAPONSPEED * 100);
//        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 4096));
//        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        ledcWrite(ESC_CHANNEL, ((uint32_t) WEAPONSPEED) + 608);
    }else{
//        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 1216));
//        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        ledcWrite(ESC_CHANNEL, 608);
    }

    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    clearFaults();
    readFaults();
//    setPWMMode();

    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
//    vTaskDelay(1);
    delay(20);
}

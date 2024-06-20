// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"

#include <Arduino.h>
#include <Bluepad32.h>

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
        // Turn on the 4 LED. Each bit represents one LED.
        static int led = 0;
        led++;
        // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
        // support changing the "Player LEDs": those 4 LEDs that usually indicate
        // the "gamepad seat".
        // It is possible to change them by calling:
        ctl->setPlayerLEDs(led & 0x0f);
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
    dumpGamepad(ctl);

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

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    pinMode(LED, OUTPUT);


    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
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


}

// Arduino loop function. Runs in CPU 1.
void loop() {
//    gpio_set_level(LED, !gpio_get_level(LED));
    gpio_set_level(LED, commsGood);

    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();


    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
    vTaskDelay(1);
//    delay(150);
}

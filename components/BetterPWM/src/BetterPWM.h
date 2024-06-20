#include "driver/ledc.h"
#include <Arduino.h>

#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_12_BIT // Set duty resolution to 12 bits // Set duty to 50%. (2 ** 12) * 50% = 4096
#define LEDC_FREQUENCY          (10000) // Frequency in Hertz. Set frequency at 4 kHz

namespace BetterPWM{
    void setupTimer(ledc_timer_t timer, uint32_t frequency);
    void setupChannel(ledc_channel_t channel, ledc_timer_t timer, int gpio_num);
    void setDutyCycle(ledc_channel_t channel, double duty); //range of 0 to 1
}
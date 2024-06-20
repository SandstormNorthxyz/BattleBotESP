#include "BetterPWM.h"

namespace BetterPWM{
    void setupTimer(ledc_timer_t timer, uint32_t frequency){
        // Prepare and then apply the LEDC PWM timer configuration
        ledc_timer_config_t ledc_timer = {
            .speed_mode       = LEDC_MODE,
            .duty_resolution  = LEDC_DUTY_RES,
            .timer_num        = timer,
            .freq_hz          = frequency,  // Set output frequency at 4 kHz
            .clk_cfg          = LEDC_AUTO_CLK
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    }

    void setupChannel(ledc_channel_t channel, ledc_timer_t timer, int gpio_num){
        // Prepare and then apply the LEDC PWM channel configuration
        ledc_channel_config_t ledc_channel = {
            .gpio_num       = gpio_num,
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_CHANNEL_0,
            .intr_type      = LEDC_INTR_DISABLE,
            .timer_sel      = timer,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }

    void setDutyCycle(ledc_channel_t channel, double duty){
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel, (duty * 4096.0)));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel));
    }
}
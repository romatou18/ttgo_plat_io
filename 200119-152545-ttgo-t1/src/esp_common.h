#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include "esp_adc_cal.h"
#include "esp_sleep.h"

//! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
static void espDelay(int ms)
{   
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,ESP_PD_OPTION_ON);
    esp_light_sleep_start();
}
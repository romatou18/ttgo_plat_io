#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include "esp_adc_cal.h"
#include "esp_sleep.h"
#include "FreeRTOS.h"

//! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
static void espDelay(int ms)
{   
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,ESP_PD_OPTION_ON);
    esp_light_sleep_start();
}


// #include "soc/timer_group_struct.h"
// #include "soc/timer_group_reg.h"
// void feedTheDog(){
//   // feed dog 0
//   TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE; // write enable
//   TIMERG0.wdt_feed=1;                       // feed dog
//   TIMERG0.wdt_wprotect=0;                   // write protect
//   // feed dog 1
//   TIMERG1.wdt_wprotect=TIMG_WDT_WKEY_VALUE; // write enable
//   TIMERG1.wdt_feed=1;                       // feed dog
//   TIMERG1.wdt_wprotect=0;                   // write protect
// }
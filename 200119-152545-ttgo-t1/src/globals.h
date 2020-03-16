#pragma once

#include <stdint.h>

#define TEST_MODE
/**********************************************
 * SCREEN
 **********************************************
 */
#ifndef TFT_DISPOFF
#define TFT_DISPOFF 0x28
#endif

#ifndef TFT_SLPIN
#define TFT_SLPIN   0x10
#endif

#define TFT_MOSI            19
#define TFT_SCLK            18
#define TFT_CS              5
#define TFT_DC              16
#define TFT_RST             23

#define TFT_BL          4  // Display backlight control pin
#define ADC_EN          14
#define ADC_PIN         34

/**********************************************
 * SPI
 **********************************************
 */
#define SD_MOSI 26
#define SD_MISO 25
#define SD_SCKL 32
#define SD_SPEED 4000000U //26 Mhz max on matrixed HSPI otherwise 27mhz

/**********************************************
 * SD CARD
 **********************************************
 */
#define SD_CS 33

/**********************************************
 * BUTTONS
 **********************************************
 */
#define BUTTON_1        35
#define BUTTON_2        0

#define FORMAT_SPIFFS_IF_FAILED true

/**********************************************
 * GPS
 **********************************************
 */
#define GPS_REFRESH_RATE_HZ 10
#define GPS_PPS_INT_ENABLED 1
static constexpr uint32_t GPSBaud = 9600U;
static constexpr int RX_pin = 38, TX_pin = 37;
static constexpr int GPS_PPS_PIN = 39;
static constexpr int GPS_QUEUE_SIZE = 20;


/**********************************************
 * BAROMETER / I2C #1
 **********************************************
 */
static constexpr uint8_t refresh_rate_baro_hz = 15;
static constexpr uint8_t I2CBUS_ID_HP206 = 1;
#define SDA_1 21
#define SCL_1 22
static constexpr uint8_t BARO_QUEUE_SIZE = 30;
#define BARO_TAG "baro"


/**********************************************
 * MPU9250 / I2C #1
 **********************************************
 */

// #define MPU_HIDEAKITAI false
// #define MPU_BOLDER true
// #define MPU_SPARKFUN trues

static constexpr float NELSON_MAGNETIC_DECLINATION = 20.49;
#define MPU9250_I2C_ADR 0x68
// #define I2CBUS_ID_MPU9250 0
#define SDA_2 17
#define SCL_2 2
#define IMU_CS 13
#define IMU_INT_PIN 12 
#define WAKE_ON_MOTION_INTERRUPT_PIN 12

#define IMU_QUEUE_SIZE 400

typedef enum
{
  GPS_T,
  BARO_T,
  IMU_T
} EventType_t;

typedef struct 
{
  void *update_p;
  EventType_t type;
} QueueUpdate;

/***************************
 * TASKS DELAYS
 ***************************/
#define GPS_TASK_DELAY 1U
#define SCREEN_TASK_DELAY 10U

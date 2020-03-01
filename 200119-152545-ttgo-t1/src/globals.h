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
constexpr uint32_t GPSBaud = 9600U;
constexpr int RX_pin = 38, TX_pin = 37;


/**********************************************
 * BAROMETER / I2C #1
 **********************************************
 */
#define I2CBUS_ID_HP206 1
#define SDA_1 21
#define SCL_1 22
#define BARO_QUEUE_SIZE 30
#define BARO_TAG "baro"


/**********************************************
 * MPU9250 / I2C #1
 **********************************************
 */

// #define MPU_HIDEAKITAI false
// #define MPU_BOLDER true
#define MPU_SPARKFUN true

static constexpr float NELSON_MAGNETIC_DECLINATION = 20.49;
#define MPU9250_I2C_ADR 0x68
// #define I2CBUS_ID_MPU9250 0
// #define SDA_2 17
// #define SCL_2 2
#define IMU_CS 13
#define IMU_INT_PIN 12 
#define WAKE_ON_MOTION_INTERRUPT_PIN 12


enum {
    X = 0,
    Y = 1,
    Z = 2
} COORD;
#define IMU_QUEUE_SIZE 400

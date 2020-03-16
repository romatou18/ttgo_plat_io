#include "esp_common.h"
#include "utils.h"


#include <TFT_eSPI.h>
#include <SPI.h>
#include "WiFi.h"

#include <Button2.h>
#include "esp_adc_cal.h"
#include "bmp.h"

// Baro HP206C on SPI
#include <HP20x_dev.h>
#include <KalmanFilter.h>
#include <Wire.h>

#include "mpu.hpp"
#include "mpu_icm20948.hpp"

#include "baro.h"

#include "gps_manager.h"
#include "sd_file.h"
#include "globals.h"

unsigned char hp206_available = 0;

#define TAG_MAIN "main"

/* Instance */
Grove::KalmanFilter t_filter;    //temperature filter
Grove::KalmanFilter p_filter;    //pressure filter
Grove::KalmanFilter a_filter;    //altitude filter

SPIClass spi_SD;
// SPIClass spi_IMU;
// set up variables using the SD utility library functions:
SDMGT sd_mgt(LOG_FILE_PATH);

TaskHandle_t *TaskGPS1;
TaskHandle_t TaskBaro2;
TaskHandle_t Task3;
TaskHandle_t TaskEstimator4;

SemaphoreHandle_t g_xHP206_reading_mutex;
SemaphoreHandle_t g_i2c_mutex;
SemaphoreHandle_t g_GPS_reading_mutex;

// static EventGroupHandle_t gps_event_group;
static QueueHandle_t g_gpsQueue  = xQueueCreate( GPS_QUEUE_SIZE, sizeof( GPSupdate * ) );
static QueueHandle_t g_baroQueue = xQueueCreate( BARO_QUEUE_SIZE, sizeof( baro_reading_t * ) );

// QueueHandle_t g_imu_queue;

static std::array<GPSupdate, GPS_QUEUE_SIZE> g_gpsUpdateList;
static std::array<baro_reading_t, BARO_QUEUE_SIZE> g_baroUpdateList;

float temperature_accumulator, temp_filtered_accumulator;
float pressure_accumulator, pressure_filtered_accumulator;
float alti_accumulator, alti_filter_accumulator;

baro_reading_t g_baro_latest;
volatile bool g_pressure_reading_lock = false;





Grove::HP20x_dev HP20x(I2CBUS_ID_HP206);


uint32_t timePreviousUs;
uint32_t timeNowUs;
float imuTimeDeltaSecs;
float baroTimeDeltaSecs;

float gpsTimeDeltaSecs;
volatile uint8_t GPS_interrupt_count = 0;
portMUX_TYPE GPS_mux = portMUX_INITIALIZER_UNLOCKED;

						// orientation/motion vars
float q[4];           // [w, x, y, z]         quaternion container
int   aa[3];         // [x, y, z]            accel sensor measurements
int   aaReal[3];     // [x, y, z]            gravity-free accel sensor measurements
int   aaWorld[3];    // [x, y, z]            world-frame accel sensor measurements
float gravity[3];    // [x, y, z]            gravity vector
 

#if MPU_BOLDER
MPU9250 IMU(spi_IMU, IMU_CS);
  // MPU9250 IMU(Wire1, MPU9250_I2C_ADR);
#endif

#if MPU_SPARKFUN
  // MPU9250 mpu;
  MPU9250_DMP imu;
#endif

portMUX_TYPE mpu_latest_mutex = portMUX_INITIALIZER_UNLOCKED;
// std::atomic_flag g_imu_reading_lock = ATOMIC_FLAG_INIT;
imu_raw_t* g_imu_latest;
std::array<imu_raw_t, IMU_QUEUE_SIZE> g_imuUpdateList;


// bias matrix used later on the altitude estimator
float g_gyroBias[3] = {0.0 , 0.0, 0.0};
float g_accBias[3] = {0.0 , 0.0, 0.0};


// configuring GPIOS to Serial tx RX
//SoftwareSerial gps_serial;
//HardwareSerial Serial2(2);

// The TinyGPS++ object
TinyGPSPlus gps;

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library
Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);

char wifi_list_buf[512];
int vref = 1100;
int btnCick = false;


void setup_sd_card()
{
  SDMGT::reader_spi_setup(sd_mgt);

  #ifdef TEST_MODE
  // SDMGT::test_sd_card();
  #endif
}


void writeLogWithPressure(unsigned long millis, uint8_t month, uint8_t day, uint8_t h, uint8_t min, uint8_t s, uint8_t cs, 
float p, float pf, float a, float af, double lat, double lng)
{
  char line[50];
  snprintf(line, sizeof(line), "%lu,%d-%d-%d:%d:%d.%d,%.6f,%.6f,%.3f,%.3f,%.3f,%.3f\n",
  millis,
  month, day, h,min, s,cs,
  lat, lng, p, pf,a, af);
  sd_mgt.logToFile(line);
}

void writeLog(unsigned long millis, uint8_t month, uint8_t day, uint8_t h, uint8_t min, uint8_t s, uint8_t cs, double lat, double lng)
{
  char line[50];
  snprintf(line, sizeof(line), "%lu,%d-%d-%d:%d:%d.%d,%.3f,%.3f,,,,\n",
  millis,
  month, day, h,min, s,cs, lat,lng);
  sd_mgt.logToFile(line);
}

void updateScreen(float lat, float lon, baro_reading_t* baro_latest)
{
    // Serial.print("updateScreen() running on core ");
    // Serial.println(xPortGetCoreID());
    static uint64_t timeStamp = 0;
    if (millis() - timeStamp > 1000) 
    {
      timeStamp = millis();       
      // String info = current_gps_pos + " " + current_gps_time;
      tft.setTextSize(2);
      tft.setRotation(1);
      tft.setCursor(0,0);
      tft.fillScreen(TFT_BLACK);
      tft.setTextDatum(TL_DATUM);

      int16_t wp = tft.width() * 0.1;
      int16_t yb = tft.height()* 0.1;
      float vh = 0.0;
      float vinc = 0.15;
    
      String latStr(lat, 7);
      String lngStr(lon, 7);

      tft.drawString(latStr , wp, yb );
      vh += vinc;
      tft.drawString(lngStr , wp, (int)(yb + tft.height()*vh));
      vh += vinc;
      // Serial.println("showGPS " +  String(gps.location.lng(), 6));

      if(g_pressure_reading_lock == false)
      {
        String p_info = String(g_baro_latest.p, 2) +" hpa   ";
        // sprintf(info, "%f hpa   ", p);
        tft.drawString(p_info, wp, yb + tft.height()* vh);
        vh += vinc;
        // sprintf(info, "%f fhpa  ", pf);
        String pf_info = String(g_baro_latest.pf, 2) +" hpaf   ";
        tft.drawString(pf_info, wp, (int)(yb + tft.height()* vh));
        vh += vinc;
        
        // sprintf(info, "%f m    ", a);
        String a_info = String(g_baro_latest.a, 2) +" m   ";
        tft.drawString(a_info, wp, (int)(yb + tft.height()* vh));
        vh += vinc;
        // sprintf(info, "%f fm    ", af);
        String af_info = String(g_baro_latest.af, 2) +" mf   ";
        tft.drawString(af_info, wp, (int)(yb + tft.height()* vh));
        vh += vinc;
      }
    }
}

void setup_soft_serial_gps()
{
  Serial.print("setup_soft_serial_gps() running on core ");
  Serial.println(xPortGetCoreID());
  // Open serial communications and wait for port to open:
//  gps_serial.begin(GPSBaud, SWSERIAL_8N1, RXPin, TXPin, false, 95, 11); // RX, TX GPS
  Serial2.begin(GPSBaud, SERIAL_8N1, RX_pin, TX_pin);    //Baud rate, parity mode, RX, TX

  /////////////////////////////////////
  // Interrupt
  ////////////////////////////////////
  
#ifdef GPS_PPS_INT_ENABLED
  pinMode(GPS_PPS_PIN, INPUT);
  attachInterrupt(GPS_PPS_PIN, GPS_PPS_interrupt_handlerISR, FALLING);
#endif

///////////////////////////
/// GPS mutex
g_GPS_reading_mutex = xSemaphoreCreateMutex();

  Serial.println(" GPS serial init success");
  espDelay(1000); 
}

void task_loop_baro_HP206C(void * param)
{
  // Serial.print("loop_baro_HP206C() running on core ");
  // Serial.println(xPortGetCoreID());
  static long Temper = 0;
  static long Pressure = 0;
  static long Altitude = 0;
  static int cnt = 1;
  static uint64_t queue_pos = 0;
  static uint64_t lastComplete = 0;

  // for(;;) 
  // {
  if( (millis() - lastComplete) >  1000 / cnt * refresh_rate_baro_hz)
  {
    if(OK_HP20X_DEV == hp206_available )
    { 
      temperature_accumulator = 0.0;
      temp_filtered_accumulator = 0.0;
      alti_accumulator = 0.0;
      alti_filter_accumulator = 0.0;
      pressure_accumulator = 0.0;
      pressure_filtered_accumulator = 0.0;

      for (int i = 0; i < cnt; i++)
      {
        Temper = 0;
        Pressure = 0;
        Altitude = 0;
        /* code */
        //Serial.println("------------------\n");
        xSemaphoreTake( g_i2c_mutex, portMAX_DELAY );
        Temper = HP20x.ReadTemperature();
        //Serial.println("Temper:");
        temperature_accumulator += Temper/100.0 / (float)cnt;
        //Serial.print(t);	  
        //Serial.println("C.");
        //Serial.println("Filter:");

        temp_filtered_accumulator += t_filter.Filter(Temper/100.0) / (float)cnt;
        //Serial.print(tf);
        //Serial.println("C.");
    
        Pressure = HP20x.ReadPressure();
        //Serial.println("Pressure:");
        pressure_accumulator += Pressure/100.0  / (float)cnt;
      
        pressure_filtered_accumulator += p_filter.Filter(Pressure/100.0)  / (float)cnt;
    
        Altitude = HP20x.ReadAltitude();
        xSemaphoreGive( g_i2c_mutex );
        //Serial.println("Altitude:");
        alti_accumulator += Altitude/100.0  / (float)cnt;
        alti_filter_accumulator += a_filter.Filter(Altitude/100.0)  / (float)cnt;

        // 
      }

      xSemaphoreTake( g_xHP206_reading_mutex, portMAX_DELAY );
      g_pressure_reading_lock = true;
      
      baro_reading_t* reading_p  = &g_baroUpdateList[queue_pos++ % BARO_QUEUE_SIZE];

      reading_p->a = alti_accumulator;
      reading_p->af = alti_filter_accumulator;
      reading_p->p = pressure_accumulator;
      reading_p->pf = pressure_filtered_accumulator;
      reading_p->t = temperature_accumulator;
      reading_p->tf = temp_filtered_accumulator;
      g_baro_latest = *reading_p;

      xQueueSend( g_baroQueue, ( void * ) &reading_p, ( TickType_t ) 0 );
      g_pressure_reading_lock = false;
      xSemaphoreGive( g_xHP206_reading_mutex );

      uint64_t diff = millis() - lastComplete;
      Serial.printf("\n \t\t\t\t\t Last Baro : %u ms ago\n", diff);
      lastComplete = millis();
      // delay((1000 / refresh_rate_baro_hz) - 0.1 * (1000 / refresh_rate_baro_hz ));

      // Serial.print( reading_p->p);
      // Serial.println("hPa.");
      // Serial.println("Filter:");

      // Serial.print( reading_p->pf);
      // Serial.println("hPa");

      // Serial.print( reading_p->a);
      // Serial.println("m. af ");
      // Serial.println("Filter:");
      // Serial.print( reading_p->af);
      // Serial.println("m.");
      // Serial.println("------------------\n");

      // delay(1000 / (refresh_rate_baro_hz * cnt));
    } // available
  } // millis
  // vTaskDelete( NULL );
}

void setup_baro_HP206C() 
{
  g_xHP206_reading_mutex = xSemaphoreCreateMutex();

   /* Power up,delay 150ms,until voltage is stable */
  espDelay(150);
  /* Reset HP20x_dev */
  HP20x.begin(SDA_1, SCL_1);
  espDelay(100);

  /* Determine HP20x_dev is available or not */
  hp206_available = HP20x.isAvailable();
  if(OK_HP20X_DEV == hp206_available)
  {
    Serial.println("HP20x_dev is available.\n");    
  }
  else
  {
    Serial.println("HP20x_dev isn't available.\n");
  }

  
  // const esp_timer_create_args_t periodic_timer_args = { &task_loop_baro_HP206C, NULL, ESP_TIMER_TASK, "periodic" };
  // esp_timer_handle_t periodic_timer;
  // esp_timer_create(&periodic_timer_args, &periodic_timer);
  /* The timer has been created but is not running yet */

  /* Start the timers */
  // esp_timer_start_periodic(periodic_timer, 0.06 * 1000 * 1000);
}


// void setup_SPI_IMU()
// {
//   spi_IMU.begin(SD_SCKL, SD_MISO, SD_MOSI, IMU_CS); //CLK,MISO,MOIS,SS
//   Serial.println(F("SPI IMU Init ok."));
//   espDelay(100);
// }

void setup_I2C_IMU()
{
  Wire.begin(SDA_1, SCL_1);
}


void setup_altitude_estimator()
{
  // setup_SPI_IMU();
  #ifdef MPU_9250
  setup_mpu9265(&imu);
  #endif
}

void task_altitude_estimator_loop(void * param)
{
  // AltitudeEstimator g_altitude = AltitudeEstimator(0.0005, 	// sigma Accel
  //                                              0.0005, 	// sigma Gyro https://curve.carleton.ca/system/files/etd/68e57cb9-b9f6-42a2-abf2-0449fcfbe499/etd_pdf/ea730d7d6d7a27dd2979483bf2799689/gessesse-multisensorattitudeandheadingreferencesystem.pdf
  //                                              0.018,   // sigma Baro
  //                                              0.5, 	// ca
  //                                              0.1);	// accelThreshold;
  Serial.print("altitude_estimator_loop() running on core ");
  Serial.println(xPortGetCoreID());
  static float accelData[3];
  static float gyroData[3];


  // static uint32_t prev_ms = millis();

  for(;;){
    // if ((millis() - prev_ms) > 16)
    // {

#if MPU_HIDEATAKI
      imuRead(gyroData, accelData);
      // prev_ms = millis();

      Serial.print("x-roll ");
      Serial.print(mpu.getRoll()); Serial.print(" ");

      // Serial.print("pitch (y-right (east))    : ");
      Serial.print("y-pitch ");
      Serial.print(mpu.getPitch()); Serial.print(" ");

      // Serial.print("yaw   (z-down (down))     : ");
      Serial.print("z-yaw ");
      Serial.print(mpu.getYaw()); Serial.print(" ");
      Serial.println(" ");
#elif MPU_BOLDER
      getIMUBolderInterrupt();
      delay(100);
#endif

      // g_altitude.estimate(accelData, gyroData, baroHeight, timestamp);
      // Serial.print(baroHeight);
      // Serial.print(",");
      // Serial.print(g_altitude.getAltitude());
      // Serial.print(",");
      // Serial.print(g_altitude.getVerticalVelocity());
      // Serial.print(",");
      // Serial.println(g_altitude.getVerticalAcceleration());
      vTaskDelay(100);
    // }
  }
  vTaskDelete( NULL );
}

void initTime() {
	timeNowUs = timePreviousUs = micros();
}

void updateTime() {
	timeNowUs = micros();
	imuTimeDeltaSecs = ((timeNowUs - timePreviousUs) / 1000000.0f);
	timePreviousUs = timeNowUs;
}


void write_log(float* lat, float* lng)
{
  static uint64_t lastComplete = 0;

  if( (millis() - lastComplete) >  50)
  {
    // Serial.print("write_log() running on core ");
    // Serial.println(xPortGetCoreID());

    configASSERT(lat);
    configASSERT(lng);
    static GPSupdate* rxGPS_p;
    static baro_reading_t* baro_rx_p;

    configASSERT(g_baroQueue);
    configASSERT(g_gpsQueue);

    if( g_gpsQueue && g_baroQueue)
    {
      ESP_LOGI("GPS queue ok");
      // Receive a message on the created queue.  Block for 1000 ticks if a
      // message is not immediately available.
      BaseType_t baro = xQueueReceive( g_baroQueue, &( baro_rx_p ), ( TickType_t ) 0 );
      BaseType_t gps = xQueueReceive( g_gpsQueue, &( rxGPS_p ), ( TickType_t ) 0 );
      if(gps)
      {
        Serial.println("gps logging received");
        // pcRxedMessage now points to the struct AMessage variable posted
        // by vATask.
        if(rxGPS_p != nullptr)
        {
          *lat = rxGPS_p->current_gps_lat;
          *lng = rxGPS_p->current_gps_lng;
          if(g_pressure_reading_lock == false)
          {
            writeLogWithPressure(millis(), rxGPS_p->month, rxGPS_p->day, rxGPS_p->h, rxGPS_p->min,
            rxGPS_p->s, rxGPS_p->cs,
            rxGPS_p->current_gps_lat, rxGPS_p->current_gps_lng,
              g_baro_latest.p, g_baro_latest.pf, g_baro_latest.a, g_baro_latest.af);
          }
          else
          {
            writeLog(millis(),rxGPS_p->month, rxGPS_p->day, rxGPS_p->h, rxGPS_p->min,
              rxGPS_p->s, rxGPS_p->cs,
              rxGPS_p->current_gps_lat, rxGPS_p->current_gps_lng);
          }

          uint64_t diff = millis() - lastComplete;
          Serial.printf("\n \t\t\t\t\t LOG : %u ms ago\n", diff);
          lastComplete = millis();
        }
        else
        {
        ESP_LOGI("xQueueReceive nullptr");
        }
        
      }
      else
      {
      ESP_LOGI("xQueueReceive nothing");
      }
      
    }
  }
}



void Task_GPS_loop(void * pvParameters)
{
  Serial.print(F("Task_GPS_loop() running on core "));
  Serial.print(xPortGetCoreID());
  Serial.print(F(" Task delay"));
  Serial.println(GPS_TASK_DELAY);

  static GPSupdate* gps_update_ptr;
  static uint64_t idx = 0;


  // Create a queue capable of containing 10 pointers to AMessage structures.
  // These should be passed by pointer as they contain a lot of data.
  static uint64_t lastTime = 0;
  for(;;)
  {   
    //   Serial.print("updateGPSInfo() running on core ");
    //   Serial.println(xPortGetCoreID());

      /* Block indefinitely (without a timeout, so no need to check the function's
        return value) to wait for a notification.  Here the RTOS task notification
        is being used as a binary semaphore, so the notification value is cleared
        to zero on exit.  NOTE!  Real applications should not block indefinitely,
        but instead time out occasionally in order to handle error conditions
        that may prevent the interrupt from sending any more notifications. */
      portENTER_CRITICAL(&GPS_mux);
      GPS_interrupt_count = 0;
      portEXIT_CRITICAL(&GPS_mux);

      if (Serial2.available() > 0)
      {
        // Serial.println("Serial2.available()");
        if (gps.encode(Serial2.read()))
        {
          if (gps.location.isValid() && gps.location.isUpdated())
          {
            Serial.print(gps.location.lat(), 6);
            Serial.print(F(","));
            Serial.print(gps.location.lng(), 6);
            gps_update_ptr = &g_gpsUpdateList[idx++ % GPS_QUEUE_SIZE];
            gps_update_ptr->current_gps_lat = gps.location.lat();
            // s += F(",");
            gps_update_ptr->current_gps_lng = gps.location.lng();

            if(gps.date.isValid())
            {
              Serial.print(gps.date.month());
              Serial.print(F("/"));
              Serial.print(gps.date.day());
              Serial.print(F("/"));
              Serial.print(gps.date.year());

              gps_update_ptr->day = gps.date.day();
              gps_update_ptr->month = gps.date.month();
            }

            Serial.print(F(" "));
            if(gps.time.isValid())
            {
              // if (gps.time.hour() < 10) Serial.print(F("0"));
              // Serial.print(gps.time.hour());
              // Serial.print(F(":"));
              // if (gps.time.minute() < 10) Serial.print(F("0"));
              // Serial.print(gps.time.minute());
              // Serial.print(F(":"));
              // if (gps.time.second() < 10) Serial.print(F("0"));
              // Serial.print(gps.time.second());
              // Serial.print(F("."));
              // if (gps.time.centisecond() < 10) Serial.print(F("0"));
              // Serial.print(gps.time.centisecond());
              // Serial.println(F(""));

              gps_update_ptr->h = gps.time.hour();
              gps_update_ptr->min = gps.time.minute();
              gps_update_ptr->s = gps.time.second();
              gps_update_ptr->cs = gps.time.centisecond();
              gps_update_ptr->timestamp = millis();

              uint64_t diff = millis() - lastTime;
              Serial.printf("\n\t\t\t\t\t\t\t\tLast GPS : %u ms ago\n\n\n", diff);
              lastTime = millis(); 
               
              // Send a pointer to a struct AMessage object.  Don't block if the
              // queue is already full.
              configASSERT(g_gpsQueue);
              configASSERT(gps_update_ptr);
              configASSERT(&gps_update_ptr);
              xQueueSend( g_gpsQueue, ( void * ) &gps_update_ptr, ( TickType_t ) 0 );
              // YIELD here since we have a ready the next one should be in almost a second since GPS running at 1HZ update rate
              delay(1000 / GPS_REFRESH_RATE_HZ);
            }
            else
            {
              Serial.print(F("INVALID TIME/DATE"));
            }
            // s += String(gps.location.lng(), 6);
          }
          else
          {
            // Serial.print(F("INVALID FIX"));
          }
          // printGPSInfo();
        } else {
          // Serial.println("NOT GPS location is updated!");
        }
      } else {
          // Serial.println("NOT Serial2.available()!");
      }
      
     
    // } // MILLIS

    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
      Serial.println(F("No GPS detected: check wiring."));
      // showTFTMessage("No GPS detected: check wiring.");
      delay(1000 / GPS_REFRESH_RATE_HZ);
    }
    // vTaskDelay(2);
  } // for
  vTaskDelete(NULL);
}

void task_update_screen(void* param)
{
  static uint64_t lastTime = 0;

  Serial.print("task_update_screen() running on core ");
  Serial.print(xPortGetCoreID());
  Serial.print(F(" Task delay"));
  Serial.println(SCREEN_TASK_DELAY);

  static float local_lat = 0;
  static float local_lng = 0; // used for screen update
  uint cycle = 0;

 
  for(;;)
  {
    
    switch(cycle)
    {
      case 0:
        // getIMUBolderInterrupt();
        // delay(0.1 * 1000); 
        // portYIELD();
        cycle++;
      break;

      case 1:
        write_log(&local_lat, &local_lng);
        cycle++;
      break;

      case 2:
        task_loop_baro_HP206C(0);
        cycle++;
      break;

      case 3:
        updateScreen(local_lat, local_lng, &g_baro_latest);
        cycle++;
      break;

      case 4:
        cycle = 0;
      break;
    }

    uint64_t diff = millis() - lastTime; 
    // Serial.printf("\nLast task_update_screen : %u ms ago\n", diff);
    lastTime = millis(); 
    vTaskDelay(2);
  }
  vTaskDelete( NULL );
}


void showVoltage()
{
    static uint64_t timeStamp = 0;
    if (millis() - timeStamp > 1000) {
        timeStamp = millis();
        uint16_t v = analogRead(ADC_PIN);
        float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
        String voltage = "Voltage :" + String(battery_voltage) + "V";
        Serial.println(voltage);
        tft.fillScreen(TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(voltage,  tft.width() / 2, tft.height() / 2 );
    }
}

void button_init()
{
    btn1.setLongClickHandler([](Button2 & b) {
        btnCick = false;
        int r = digitalRead(TFT_BL);
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("Press again to wake up",  tft.width() / 2, tft.height() / 2 );
        espDelay(6000);
        digitalWrite(TFT_BL, !r);

        tft.writecommand(TFT_DISPOFF);
        tft.writecommand(TFT_SLPIN);
        esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
        esp_deep_sleep_start();
    });
    btn1.setPressedHandler([](Button2 & b) {
        Serial.println("Detect Voltage..");
        btnCick = true;
    });

    btn2.setPressedHandler([](Button2 & b) {
        btnCick = false;
        Serial.println("btn press wifi scan");
        // wifi_scan();
        //showBaro();
    });
}

void button_loop()
{
    btn1.loop();
    btn2.loop();
}

void wifi_scan()
{
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(1);

    tft.drawString("Scan Network", tft.width() / 2, tft.height() / 2);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    int16_t n = WiFi.scanNetworks();
    tft.fillScreen(TFT_BLACK);
    if (n == 0) {
        tft.drawString("no networks found", tft.width() / 2, tft.height() / 2);
    } else {
        tft.setTextDatum(TL_DATUM);
        tft.setCursor(0, 0);
        Serial.printf("Found %d net\n", n);
        for (int i = 0; i < n; ++i) {
            sprintf(wifi_list_buf,
                    "[%d]:%s(%d)",
                    i + 1,
                    WiFi.SSID(i).c_str(),
                    WiFi.RSSI(i));
            tft.println(wifi_list_buf);
        }
    }
    WiFi.mode(WIFI_OFF);
}


void mpu_imu_task(void* param) 
{
  MPU_20948::imu_task(param);
}


void setup()
{
  Serial.begin(115200);
  ESP_LOGD(TAG_MAIN, F("setup()"));

  // g_gpsQueue = xQueueCreate( GPS_QUEUE_SIZE, sizeof( GPSupdate * ) );
  if( g_gpsQueue == 0 )
  {
    ESP_LOGE(GPS, "failed to created gps queue");
  }
  else
  {
    ESP_LOGI(GPS, "GPS queue created ok");
  }
  
  // g_baroQueue = xQueueCreate( BARO_QUEUE_SIZE, sizeof( baro_reading_t * ) );
  if( g_baroQueue == 0 )
  {
    ESP_LOGE(BARO_TAG, "failed to created barometer queue");
  }
  else
  {
    ESP_LOGI(BARO_TAG, "barometer queue created ok");
  }

  espDelay(500); 

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(0, 0);
  tft.setTextDatum(MC_DATUM);

  tft.setTextSize(1);

  if (TFT_BL > 0) { // TFT_BL has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
        pinMode(TFT_BL, OUTPUT); // Set backlight pin to output mode
        digitalWrite(TFT_BL, TFT_BACKLIGHT_ON); // Turn backlight on. TFT_BACKLIGHT_ON has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
  }

  tft.setSwapBytes(true);
  tft.pushImage(0, 0,  240, 135, kitetrack242r);
  espDelay(5000);

  tft.setRotation(0);
  button_init();

  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
  //Check type of calibration value used to characterize ADC
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
      Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
      vref = adc_chars.vref;
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
      Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
  } else {
      Serial.println("Default Vref: 1100mV");
  }

  g_i2c_mutex = xSemaphoreCreateMutex();
  setup_sd_card();
  setup_altitude_estimator();

  setup_soft_serial_gps();
  setup_baro_HP206C(); 
  setup_I2C_IMU();

  delay(5000); 

 

  
  xTaskCreatePinnedToCore(
                      Task_GPS_loop,   /* Task function. */
                      "Task1GPS",     /* name of task. */
                      5000,       /* Stack size of task */
                      NULL,        /* parameter of the task */
                      2,           /* priority of the task */
                      TaskGPS1,      /* Task handle to keep track of created task */
                      0);          /* pin task to core 0 */     

  // xTaskCreatePinnedToCore(
  //                     task_loop_baro_HP206C,   /* Task function. */
  //                     "Task2Baro",     /* name of task. */
  //                     5000,       /* Stack size of task */
  //                     NULL,        /* parameter of the task */
  //                     2,           /* priority of the task */
  //                     &TaskBaro2,      /* Task handle to keep track of created task */
  //                     1);          /* pin task to core 1 */

  xTaskCreatePinnedToCore(
                    task_update_screen,   /* Task function. */
                    "Task3Screen",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    3,           /* priority of the task */
                    &Task3,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */


  // xTaskCreatePinnedToCore(
  //                 mpu_imu_task,   /* Task function. */
  //                 "Task4IMU",     /* name of task. */
  //                 10000,       /* Stack size of task */
  //                 NULL,        /* parameter of the task */
  //                 2,           /* priority of the task */
  //                 &TaskEstimator4  ,   /* Task handle to keep track of created task */
  //                 1);          /* pin task to core 1 */
}

void loop()
{
    
    // i2c_scanner();
    // loop_baro_HP206C();
    // getCurrentGPSInfo();


    // if (btnCick) {
      // showVoltage();
     
      // showBaro();
//        showTFTMessage(current_gps_info);
//        Serial.println(current_gps_info);
      // This sketch displays information every time a new sentence is correctly encoded.
     
    // }
    // button_loop();
   
}

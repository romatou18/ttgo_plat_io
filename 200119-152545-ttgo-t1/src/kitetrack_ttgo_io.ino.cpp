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
#include "mpu.h"

#include "gps_manager.h"
#include "sd_file.h"
#include "globals.h"

unsigned char hp206_available = 0;



/* Instance */
Grove::KalmanFilter t_filter;    //temperature filter
Grove::KalmanFilter p_filter;    //pressure filter
Grove::KalmanFilter a_filter;    //altitude filter

SPIClass spi_SD;
// set up variables using the SD utility library functions:
SDMGT sd_mgt(LOG_FILE_PATH);

TaskHandle_t TaskGPS1;
TaskHandle_t TaskBaro2;
TaskHandle_t Task3;
TaskHandle_t TaskEstimator4;


// static EventGroupHandle_t gps_event_group;
QueueHandle_t g_gpsQueue;
std::array<GPSupdate, GPS_QUEUE_SIZE> g_gpsUpdateList;


float temperature_accumulator, temp_filtered_accumulator;
float pressure_accumulator, pressure_filtered_accumulator;
float alti_accumulator, alti_filter_accumulator;

typedef struct {
  float t;
  float tf;
  float p;
  float pf;
  float a;
  float af;
} baro_reading_t;

baro_reading_t g_baro_latest;

volatile bool g_pressure_reading_lock = false;
SemaphoreHandle_t g_xHP206_reading_mutex;
Grove::HP20x_dev HP20x(I2CBUS_ID_HP206);
std::array<baro_reading_t, BARO_QUEUE_SIZE> g_baroUpdateList;
QueueHandle_t g_baroQueue;

// static struct {
//   long Temper;

//   long Pressure;

//   long Altitude;
// } bar_d;


#if BOLDER
MPU9250 IMU(Wire1, MPU9250_I2C_ADR);
#else
MPU9250 mpu;
#endif


AltitudeEstimator g_altitude = AltitudeEstimator(0.0005, 	// sigma Accel
                                               0.0005, 	// sigma Gyro https://curve.carleton.ca/system/files/etd/68e57cb9-b9f6-42a2-abf2-0449fcfbe499/etd_pdf/ea730d7d6d7a27dd2979483bf2799689/gessesse-multisensorattitudeandheadingreferencesystem.pdf
                                               0.018,   // sigma Baro
                                               0.5, 	// ca
                                               0.1);	// accelThreshold;

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

char buff[512];
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

void updateScreen(float lat, float lon)
{
    static uint64_t timeStamp = 0;
    if (millis() - timeStamp > 100) 
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
  Serial2.begin(GPSBaud,SERIAL_8N1, RX_pin, TX_pin);    //Baud rate, parity mode, RX, TX

  Serial.println(" GPS serial init success");
  espDelay(1000); 
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
}

void setup_altitude_estimator()
{

}

void altitude_estimator_loop(void * param)
{
  float baroHeight = g_baro_latest.af;
  float accelData[3];
  float gyroData[3];
  static uint32_t prev_ms = millis();

  for(;;){
    if ((millis() - prev_ms) > 16)
    {
      uint32_t timestamp = micros();
      imuRead(gyroData, accelData);
      prev_ms = millis();

      Serial.print("x-roll ");
      Serial.print(mpu.getRoll()); Serial.print(" ");

      // Serial.print("pitch (y-right (east))    : ");
      Serial.print("y-pitch ");
      Serial.print(mpu.getPitch()); Serial.print(" ");

      // Serial.print("yaw   (z-down (down))     : ");
      Serial.print("z-yaw ");
      Serial.print(mpu.getYaw()); Serial.print(" ");
      Serial.println(" ");


      // g_altitude.estimate(accelData, gyroData, baroHeight, timestamp);
      // Serial.print(baroHeight);
      // Serial.print(",");
      // Serial.print(g_altitude.getAltitude());
      // Serial.print(",");
      // Serial.print(g_altitude.getVerticalVelocity());
      // Serial.print(",");
      // Serial.println(g_altitude.getVerticalAcceleration());
    }
  }

  vTaskDelete( NULL );
}

void loop_baro_HP206C(void* param)
{
  Serial.print("loop_baro_HP206C() running on core ");
  Serial.println(xPortGetCoreID());
  static long Temper = 0;
  static long Pressure = 0;
  static long Altitude = 0;
  static uint8_t refresh_rate_baro_hz = 15;
  static int cnt = 7;
  static uint64_t queue_pos = 0;
    // spiffs_setup();

  for(;;)
  {
    
    if(OK_HP20X_DEV == hp206_available)
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
        //Serial.println("Altitude:");
        alti_accumulator += Altitude/100.0  / (float)cnt;
        alti_filter_accumulator += a_filter.Filter(Altitude/100.0)  / (float)cnt;
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

      Serial.print( reading_p->p);
      Serial.println("hPa.");
      Serial.println("Filter:");

      Serial.print( reading_p->pf);
      Serial.println("hPa");

      Serial.print( reading_p->a);
      Serial.println("m. af ");
      Serial.println("Filter:");
      Serial.print( reading_p->af);
      Serial.println("m.");
      Serial.println("------------------\n");
      espDelay(1000 / refresh_rate_baro_hz);
    }
  }
  vTaskDelete( NULL );
}


void task_update_screen(void* param)
{
  Serial.print("task_update_screen() running on core ");
  Serial.println(xPortGetCoreID());
  GPSupdate* rxGPS_p;
  baro_reading_t* baro_rx_p;

  for(;;)
  {
    if( g_gpsQueue != 0 )
    {
      ESP_LOGI("GPS queue ok");
      // Receive a message on the created queue.  Block for 1000 ticks if a
      // message is not immediately available.
      BaseType_t baro = xQueueReceive( g_baroQueue, &( baro_rx_p ), ( TickType_t ) 60 );
      BaseType_t gps = xQueueReceive( g_gpsQueue, &( rxGPS_p ), ( TickType_t ) 1000 );
      if(gps)
      {
        ESP_LOGI("gps received");
        // pcRxedMessage now points to the struct AMessage variable posted
        // by vATask.
        if(rxGPS_p != nullptr)
        {
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
          updateScreen(rxGPS_p->current_gps_lat, rxGPS_p->current_gps_lng);
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
            sprintf(buff,
                    "[%d]:%s(%d)",
                    i + 1,
                    WiFi.SSID(i).c_str(),
                    WiFi.RSSI(i));
            tft.println(buff);
        }
    }
    WiFi.mode(WIFI_OFF);
}


void setup()
{
  Serial.begin(115200);
  Serial.println("Start");

  g_gpsQueue = xQueueCreate( GPS_QUEUE_SIZE, sizeof( GPSupdate * ) );
  if( g_gpsQueue == 0 )
  {
    ESP_LOGE(GPS, "failed to created gps queue");
  }
  else
  {
    ESP_LOGI(GPS, "GPS queue created ok");
  }

  g_baroQueue = xQueueCreate( BARO_QUEUE_SIZE, sizeof( baro_reading_t * ) );
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

  setup_sd_card();
  setup_soft_serial_gps();
  setup_baro_HP206C(); 
  setup_altitude_estimator();


  // if (!SPIFFS.begin(true)) 
  // {
  //   Serial.println("An Error has occurred while mounting SPIFFS");
  //   return;
  // }

    
  espDelay(1000); 
     //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    getCurrentGPSInfo,   /* Task function. */
                    "Task1GPS",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TaskGPS1,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 0 */     

  xTaskCreatePinnedToCore(
    loop_baro_HP206C,   /* Task function. */
    "Task2Baro",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    2,           /* priority of the task */
    &TaskBaro2,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */

  xTaskCreatePinnedToCore(
                    task_update_screen,   /* Task function. */
                    "Task3Screen",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task3,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */


//   xTaskCreatePinnedToCore(
//                   altitude_estimator_loop,   /* Task function. */
//                   "Task4IMU",     /* name of task. */
//                   10000,       /* Stack size of task */
//                   NULL,        /* parameter of the task */
//                   2,           /* priority of the task */
//                   &TaskEstimator4,      /* Task handle to keep track of created task */
//                   1);          /* pin task to core 1 */
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

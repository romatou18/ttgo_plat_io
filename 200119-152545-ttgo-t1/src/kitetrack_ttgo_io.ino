#include "Arduino.h"

#include <TFT_eSPI.h>
#include <SPI.h>
#include "WiFi.h"

#include <Button2.h>
#include "esp_adc_cal.h"
#include "bmp.h"

// GPS on Serial2
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Streaming.h>

// Baro HP206C on SPI
#include <HP20x_dev.h>
#include <KalmanFilter.h>
#include <Wire.h>
unsigned char ret = 0;

/* Instance */
KalmanFilter t_filter;    //temperature filter
KalmanFilter p_filter;    //pressure filter
KalmanFilter a_filter;    //altitude filter

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
#define BUTTON_1        35
#define BUTTON_2        0

TaskHandle_t Task1;
TaskHandle_t Task2;

static const int RXPin = 38, TXPin = 37;
static const uint32_t GPSBaud = 9600;
static volatile float current_gps_lat =0.0;
static volatile float current_gps_lng =0.0;

// static struct {
//   long Temper;
//   float t;
//   float tf;

//   long Pressure;
//   float p;
//   float pf;

//   long Altitude;
//   float a;
//   float af;
// } bar_d;

float t, tf;
float p, pf;
float a, af;

HP20x_dev HP20x(((uint8_t)1));

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

void i2c_scanner() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
    //exit(EXIT_FAILURE);
  }
  else {
    Serial.println("done\n");
  }
  espDelay(5000);          
}

void setup_baro_HP206C() 
{
   /* Power up,delay 150ms,until voltage is stable */
  espDelay(150);
  /* Reset HP20x_dev */
  HP20x.begin(21, 22);
  espDelay(100);

  /* Determine HP20x_dev is available or not */
  ret = HP20x.isAvailable();
  if(OK_HP20X_DEV == ret)
  {
    Serial.println("HP20x_dev is available.\n");    
  }
  else
  {
    Serial.println("HP20x_dev isn't available.\n");
  }
}


// void update_baro_HP206C()
// {

//   current_pressure_inf = String("no-baro");
//   current_altitude_inf = String("no-alti");
//   if(OK_HP20X_DEV == ret)
//   { 
//     bar_d.Temper = HP20x.ReadTemperature();
//     bar_d.t = bar_d.Temper / 100.0; 
//     bar_d.tf = t_filter.Filter(bar_d.t);
//     current_temp_inf = String(bar_d.Temper + "C, " + String(bar_d.tf) + "f_C");

//     bar_d.Pressure = HP20x.ReadPressure();
//     bar_d.p = bar_d.Pressure / 100.0;
//     bar_d.pf = p_filter.Filter(bar_d.p);
//     current_pressure_inf = String(bar_d.Pressure + "hPa, " + String(bar_d.pf) + "f_hPa");

//     bar_d.Altitude = HP20x.ReadAltitude();
//     bar_d.a = bar_d.Altitude / 100.0;
//     bar_d.af = a_filter.Filter(bar_d.a);
//     current_altitude_inf = String(bar_d.Altitude + "m, " + String(bar_d.af) + "f_m");

//     espDelay(200);
//   }
// }

void loop_baro_HP206C(void* param)
{
    char display[40];
    static long Temper = 0;
    static long Pressure = 0;
    static long Altitude = 0;
  for(;;){
    if(OK_HP20X_DEV == ret)
    { 
      int cnt = 25;
      t = 0.0;
      tf = 0.0;
      a = 0.0;
      af = 0.0;
      p = 0.0;
      pf = 0.0;

      for (int i = 0; i < cnt; i++)
      {
        Temper = 0;
        Pressure = 0;
        Altitude = 0;
        /* code */
        //Serial.println("------------------\n");
        Temper = HP20x.ReadTemperature();
        //Serial.println("Temper:");
        t += Temper/100.0 / (float)cnt;
        //Serial.print(t);	  
        //Serial.println("C.");
        //Serial.println("Filter:");

        tf += t_filter.Filter(Temper/100.0) / (float)cnt;
        //Serial.print(tf);
        //Serial.println("C.");
    
        Pressure = HP20x.ReadPressure();
        //Serial.println("Pressure:");
        p += Pressure/100.0  / (float)cnt;
       
        pf += p_filter.Filter(Pressure/100.0)  / (float)cnt;
     
        
        Altitude = HP20x.ReadAltitude();
        //Serial.println("Altitude:");
        a += Altitude/100.0  / (float)cnt;
        af += a_filter.Filter(Altitude/100.0)  / (float)cnt;
       
        }
      }

      Serial.print(p);
      Serial.println("hPa.");
      Serial.println("Filter:");

      Serial.print(pf);
      Serial.println("hPa");

      Serial.print(a);
      Serial.println("m. af ");
      Serial.println("Filter:");
      Serial.print(af);
      Serial.println("m.");
      Serial.println("------------------\n");
      espDelay(200);
  }
	 
}

void setup_soft_serial_gps()
{
  Serial.print("setup_soft_serial_gps() running on core ");
  Serial.println(xPortGetCoreID());
  // Open serial communications and wait for port to open:
//  gps_serial.begin(GPSBaud, SWSERIAL_8N1, RXPin, TXPin, false, 95, 11); // RX, TX GPS
  Serial2.begin(GPSBaud,SERIAL_8N1, RXPin, TXPin);    //Baud rate, parity mode, RX, TX

  Serial.println(" GPS serial init success");
  espDelay(1000); 
}

//! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
void espDelay(int ms)
{   
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,ESP_PD_OPTION_ON);
    esp_light_sleep_start();
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

void showGPS()
{
    static uint64_t timeStamp = 0;
    if (millis() - timeStamp > 100) {
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
      
        tft.drawString( String(gps.location.lat(), 6), wp, yb );
        vh += vinc;
        tft.drawString( String(gps.location.lng(), 6),wp, (int)(yb + tft.height()*vh));
        vh += vinc;
        Serial.println("showGPS " +  String(gps.location.lng(), 6));

      String info = String(p, 2) +" hpa   ";
      // sprintf(info, "%f hpa   ", p);
      tft.drawString(info, wp, yb + tft.height()* vh);
       vh += vinc;
      // sprintf(info, "%f fhpa  ", pf);
      info = String(pf, 2) +" hpaf   ";
      tft.drawString(info, wp, (int)(yb + tft.height()* vh));
      vh += vinc;
      
      // sprintf(info, "%f m    ", a);
      info = String(a, 2) +" m   ";
      tft.drawString(info, wp, (int)(yb + tft.height()* vh));
       vh += vinc;
      // sprintf(info, "%f fm    ", af);
      info = String(af, 2) +" mf   ";
      tft.drawString(info, wp, (int)(yb + tft.height()* vh));
       vh += vinc;
        
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

void printGPSInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

void getCurrentGPSInfo(void * pvParameters)
{
  Serial.print("updateGPSInfo() running on core ");
  Serial.println(xPortGetCoreID());
  for(;;){
    if (Serial2.available() > 0)
    {
      if (gps.encode(Serial2.read()))
      {
        if (gps.location.isValid())
        {
          current_gps_lat = gps.location.lat();
          // s += F(",");
          current_gps_lng = gps.location.lat();
          // s += String(gps.location.lng(), 6);
        }
        else
        {
          // s += F("No-GPS-signal");
          current_gps_lat =0.0;
          // s += F(",");
          current_gps_lng = 0.0;
        }
        printGPSInfo();
      }
    }
      
    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
      Serial.println(F("No GPS detected: check wiring."));
  //    showTFTMessage("No GPS detected: check wiring.");
      espDelay(500);
    }
  }

  showGPS();
  espDelay(50);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Start");


  espDelay(500); 
  setup_soft_serial_gps();
  setup_baro_HP206C(); 

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

     //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    getCurrentGPSInfo,   /* Task function. */
                    "Task1GPS",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 0 */                  
  espDelay(500); 

  xTaskCreatePinnedToCore(
                    loop_baro_HP206C,   /* Task function. */
                    "Task2Baro",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    0,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 1 */

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

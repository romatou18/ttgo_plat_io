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

static const int RXPin = 38, TXPin = 37;
static const uint32_t GPSBaud = 9600;
static String current_gps_time;
static String current_gps_pos;
static String current_pressure_inf;
static String current_temp_inf;
static String current_altitude_inf;

static struct {
  long Temper;
  float t;
  float tf;

  long Pressure;
  float p;
  float pf;

  long Altitude;
  float a;
  float af;
} bar_d;

HP20x_dev HP20x(((uint8_t)2));

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


void update_baro_HP206C()
{

  current_pressure_inf = String("no-baro");
  current_altitude_inf = String("no-alti");
  if(OK_HP20X_DEV == ret)
  { 
    bar_d.Temper = HP20x.ReadTemperature();
    bar_d.t = bar_d.Temper / 100.0; 
    bar_d.tf = t_filter.Filter(bar_d.t);
    current_temp_inf = String(bar_d.Temper + "C, " + String(bar_d.tf) + "f_C\n");

    bar_d.Pressure = HP20x.ReadPressure();
    bar_d.p = bar_d.Pressure / 100.0;
    bar_d.pf = p_filter.Filter(bar_d.p);
    current_pressure_inf = String(bar_d.Pressure + "hPa, " + String(bar_d.pf) + "f_hPa\n");

    bar_d.Altitude = HP20x.ReadAltitude();
    bar_d.a = bar_d.Altitude / 100.0;
    bar_d.af = a_filter.Filter(bar_d.a);
    current_altitude_inf = String(bar_d.Altitude + "m, " + String(bar_d.af) + "f_m\n");

    espDelay(200);
  }
}
void loop_baro_HP206C()
{
    char display[40];
    if(OK_HP20X_DEV == ret)
    { 
	  Serial.println("------------------\n");
	  long Temper = HP20x.ReadTemperature();
	  Serial.println("Temper:");
	  float t = Temper/100.0;
	  Serial.print(t);	  
	  Serial.println("C.\n");
	  Serial.println("Filter:");
	  Serial.print(t_filter.Filter(t));
	  Serial.println("C.\n");
 
      long Pressure = HP20x.ReadPressure();
	  Serial.println("Pressure:");
	  t = Pressure/100.0;
	  Serial.print(t);
	  Serial.println("hPa.\n");
	  Serial.println("Filter:");
	  Serial.print(p_filter.Filter(t));
	  Serial.println("hPa\n");
	  
	  long Altitude = HP20x.ReadAltitude();
	  Serial.println("Altitude:");
	  t = Altitude/100.0;
	  Serial.print(t);
	  Serial.println("m.\n");
	  Serial.println("Filter:");
	  Serial.print(a_filter.Filter(t));
	  Serial.println("m.\n");
	  Serial.println("------------------\n");
    espDelay(200);
    }
}

void setup_soft_serial_gps()
{
  // Open serial communications and wait for port to open:
//  gps_serial.begin(GPSBaud, SWSERIAL_8N1, RXPin, TXPin, false, 95, 11); // RX, TX GPS
  Serial2.begin(GPSBaud,SERIAL_8N1, RXPin, TXPin);    //Baud rate, parity mode, RX, TX

  Serial.println(" GPS serial init success");
  delayMicroseconds(2000); 
}

//! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
void espDelay(int ms)
{   
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,ESP_PD_OPTION_ON);
    esp_light_sleep_start();
}

void updateGPSInfo()
{
  String s = String("");
  if (gps.location.isValid())
  {
    s += String(gps.location.lat(), 6);
    s += F(",");
    s += String(gps.location.lng(), 6);
  }
  else
  {
    s += F("No-FIX");
  }

  current_gps_pos = s;

  String t = "";
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) t += F("0");
    t += gps.time.hour();
    t += F(":");
    if (gps.time.minute() < 10) t += F("0");
    t += gps.time.minute();
    t += F(":");
    if (gps.time.second() < 10) t += F("0");
    t += gps.time.second();
    t += F(".");
    if (gps.time.centisecond() < 10) 
    {
      t += F("0");
    }
    t += gps.time.centisecond();
  }
  else
  {
    t += F("no-time");
  }

  current_gps_time = t;
}

//void showStrings()
//{
//    static uint64_t timeStamp = 0;
//    if (millis() - timeStamp > 1000) {
//        timeStamp = millis();
//        // read GPS value here + baro
//        Serial.println(msg);
//        tft.fillScreen(TFT_BLACK);
//        tft.setTextDatum(MC_DATUM);
//
//        int offset = 0;
//        tft.drawString(gps,  tft.width() / 2, tft.height() + offset++ / 2 );
//        tft.drawString(altitude,  tft.width() / 2, tft.height() + offset++ / 2 );
//        tft.drawString(pressure,  tft.width() / 2, tft.height() + offset++ / 2 );
//    }
//}

void showTFTMessage(String msg)
{
    static uint64_t timeStamp = 0;
    if (millis() - timeStamp > 1000) {
        timeStamp = millis();
        Serial.println(msg);
        tft.fillScreen(TFT_BLACK);
        tft.setTextDatum(MC_DATUM);

        int offset = 0;
        tft.drawString(msg,  tft.width() / 2, tft.height() + offset++ / 2 );
    }
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
    if (millis() - timeStamp > 1000) {
        timeStamp = millis();
        String info = current_gps_pos + " " + current_gps_time;
        Serial.println("showGPS " + info);
        tft.fillScreen(TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(current_gps_pos,  tft.width() / 2, tft.height() / 2 );
        tft.drawString(current_gps_time,  tft.width() / 2, tft.height()+ 10 / 2 );
    }
}


void showBaro()
{
  static uint64_t timeStamp = 0;
  if (millis() - timeStamp > 1000) {
      timeStamp = millis();
      String info = current_pressure_inf + " " + current_altitude_inf;
      Serial.println( "showBaro " + info);
      tft.fillScreen(TFT_BLACK);
      tft.setTextDatum(MC_DATUM);
      tft.drawString(current_pressure_inf,  tft.width() / 2, tft.height() / 2 );
      tft.drawString(current_altitude_inf,  tft.width() / 2, tft.height() + 3 / 2 );
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
        wifi_scan();
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

bool getCurrentGPSInfo()
{
  if (Serial2.available() > 0)
  {
    if (gps.encode(Serial2.read()))
    {
      updateGPSInfo();
      printGPSInfo();
      return true;
    }
  }
    
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
//    showTFTMessage("No GPS detected: check wiring.");
    espDelay(2000);
    return false;
  }
}
void setup()
{
    Serial.begin(115200);
    Serial.println("Start");
    setup_soft_serial_gps();
    setup_baro_HP206C(); 

    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(0, 0);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(1);

    current_gps_pos = String("no-gps");
    current_gps_time = String("no-gps");
    current_pressure_inf = String("no-baro");
    current_altitude_inf = String("no-alti");

    if (TFT_BL > 0) { // TFT_BL has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
         pinMode(TFT_BL, OUTPUT); // Set backlight pin to output mode
         digitalWrite(TFT_BL, TFT_BACKLIGHT_ON); // Turn backlight on. TFT_BACKLIGHT_ON has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
    }

    tft.setSwapBytes(true);
    tft.pushImage(0, 0,  240, 135, ttgo);
    espDelay(5000);

    tft.setRotation(0);
    int i = 5;
    while (i--) {
        tft.fillScreen(TFT_RED);
        espDelay(200);
        tft.fillScreen(TFT_BLUE);
        espDelay(200);
        tft.fillScreen(TFT_GREEN);
        espDelay(200);
    }
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

}



void loop()
{
    if (btnCick) {
        //showVoltage();
        // showGPS();
        showBaro();
//        showTFTMessage(current_gps_info);
//        Serial.println(current_gps_info);
        // This sketch displays information every time a new sentence is correctly encoded.
     
    }
    // getCurrentGPSInfo();
    i2c_scanner();
    loop_baro_HP206C();
    button_loop();
   
}

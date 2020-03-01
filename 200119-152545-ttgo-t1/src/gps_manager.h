#pragma once

// GPS on Serial2
#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Streaming.h>



typedef struct
{
  volatile float current_gps_lat = 0.0;
  volatile float current_gps_lng = 0.0;
  volatile uint8_t day, month;
  volatile uint8_t h, s, cs, min;
} GPSupdate;


extern TinyGPSPlus gps;
extern const uint32_t GPSBaud;
extern float current_gps_lat;
extern float current_gps_lng;

extern QueueHandle_t g_gpsQueue;
#define GPS_QUEUE_SIZE 2
extern std::array<GPSupdate, GPS_QUEUE_SIZE> g_gpsUpdateList;



#define GPS "gps"


void printGPSInfo()
{
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
  static GPSupdate* gps_update_ptr;
  static uint64_t idx = 0;

  Serial.print("updateGPSInfo() running on core ");
  Serial.println(xPortGetCoreID());

  // Create a queue capable of containing 10 pointers to AMessage structures.
  // These should be passed by pointer as they contain a lot of data.
  // static uint64_t timeStamp = 0;
  for(;;)
  {
    // Serial.print("updateGPSInfo() running on core ");
    // Serial.println(xPortGetCoreID());
    // if (millis() - timeStamp > 500) 
    // {
    //   timeStamp = millis();  
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
              Serial.println(F(""));

              gps_update_ptr->h = gps.time.hour();
              gps_update_ptr->min = gps.time.minute();
              gps_update_ptr->s = gps.time.second();
              gps_update_ptr->cs = gps.time.centisecond();
              // Send a pointer to a struct AMessage object.  Don't block if the
              // queue is already full.
              xQueueSend( g_gpsQueue, ( void * ) &gps_update_ptr, ( TickType_t ) 0 );
              vTaskDelay(400);
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
        
      if (millis() > 5000 && gps.charsProcessed() < 10)
      {
        Serial.println(F("No GPS detected: check wiring."));
    //    showTFTMessage("No GPS detected: check wiring.");
        espDelay(500);
      }
    // }
    
  }
  vTaskDelete(NULL);
}
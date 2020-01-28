#pragma once

// GPS on Serial2
#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Streaming.h>

extern TinyGPSPlus gps;
extern const uint32_t GPSBaud;
extern float current_gps_lat;
extern float current_gps_lng;

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
  espDelay(50);
}
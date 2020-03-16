#pragma once

// GPS on Serial2
#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Streaming.h>


typedef struct
{
  float current_gps_lat = 0.0;
  float current_gps_lng = 0.0;
  uint8_t day, month;
  uint8_t h, s, cs, min;
  uint64_t timestamp = 0;
} GPSupdate;

extern TinyGPSPlus gps;
extern const uint32_t GPSBaud;
extern float current_gps_lat;
extern float current_gps_lng;
// extern SemaphoreHandle_t g_GPS_reading_mutex;
extern volatile uint8_t GPS_interrupt_count;
extern portMUX_TYPE GPS_mux;

// extern std::array<GPSupdate, GPS_QUEUE_SIZE> g_gpsUpdateList;



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


//ISR for PPS interrupt
void IRAM_ATTR GPS_PPS_interrupt_handlerISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  portENTER_CRITICAL_ISR(&GPS_mux);
  GPS_interrupt_count++;
  portEXIT_CRITICAL_ISR(&GPS_mux);
  /* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE.
  The macro used to do this is dependent on the port and may be called
  portEND_SWITCHING_ISR. */
  // portYIELD_FROM_ISR();
}  

/* An interrupt handler.  The interrupt handler does not perform any processing,
instead it unblocks a high priority task in which the event that generated the
interrupt is processed.  If the priority of the task is high enough then the
interrupt will return directly to the task (so it will interrupt one task but
return to a different task), so the processing will occur contiguously in time -
just as if all the processing had been done in the interrupt handler itself. */
void GPS_PPS_interrupt_handlerISR_BAK( void )
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Clear the interrupt. */

    /* xHigherPriorityTaskWoken must be initialised to pdFALSE.  If calling
    vTaskNotifyGiveFromISR() unblocks the handling task, and the priority of
    the handling task is higher than the priority of the currently running task,
    then xHigherPriorityTaskWoken will automatically get set to pdTRUE. */

    /* Unblock the handling task so the task can perform any processing necessitated
    by the interrupt.  xHandlingTask is the task's handle, which was obtained
    when the task was created. */

//     if(TaskGPS1) {
//       vTaskNotifyGiveFromISR( &TaskGPS1, &xHigherPriorityTaskWoken );

//       /* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE.
//       The macro used to do this is dependent on the port and may be called
//       portEND_SWITCHING_ISR. */
//       portYIELD_FROM_ISR();
//     }
}


#include <Arduino.h>

#include "MPU9250.h"
#include <Wire.h>

MPU9250 mpu;
 
void setup()
{
    Serial.begin(9600);

    Wire.begin(17, 2);

    delay(2000);
    mpu.setup(Wire);
}

void loop()
{
    static uint32_t prev_ms = millis();
    if ((millis() - prev_ms) > 200)
    {
        mpu.update();
        // mpu.print();

        // Serial.print("roll  (x-forward (north)) : ");
        Serial.print("x-roll ");
        Serial.print(mpu.getRoll()); Serial.print(" ");

        // Serial.print("pitch (y-right (east))    : ");
        Serial.print("y-pitch ");
        Serial.print(mpu.getPitch()); Serial.print(" ");

        // Serial.print("yaw   (z-down (down))     : ");
        Serial.print("z-yaw ");
        Serial.print(mpu.getYaw()); Serial.print(" ");
        Serial.println(" ");

        prev_ms = millis();
    }
}

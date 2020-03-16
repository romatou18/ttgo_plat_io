#include "Arduino.h"
#include "MPU9250.h"
#include "Wire.h"
TwoWire Wire1 = TwoWire(1);

MPU9250 mpu;
 Wire1.begin(13, 12);
void setup()
{
    Serial.begin(115200);

    Wire.begin();

    delay(2000);
    mpu.setup(Wire1);
}

void loop()
{
    static uint32_t prev_ms = millis();
    if ((millis() - prev_ms) > 16)
    {
        mpu.update();
        mpu.print();

        Serial.print("roll  (x-forward (north)) : ");
        Serial.println(mpu.getRoll());
        Serial.print("pitch (y-right (east))    : ");
        Serial.println(mpu.getPitch());
        Serial.print("yaw   (z-down (down))     : ");
        Serial.println(mpu.getYaw());

        prev_ms = millis();
    }
}
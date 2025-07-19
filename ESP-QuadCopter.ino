#include "MPU6050.h"

void setup() {
    Serial.begin(115200);
    Wire.begin();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    MPU6050 gyroscope(0x68, 400000);
    gyroscope.initialize();
}

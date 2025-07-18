#include "MPU6050.h"

void setup() {
    Serial.begin(115200);
    Wire.begin();
    vTaskDelay(Sensor::I2C_INIT_DELAY_MS);

    MPU6050 gyroscope(0x68, 400000);
    mpu.initialize();
}

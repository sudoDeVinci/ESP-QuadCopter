#include "MPU6050.hpp"
#include <Arduino.h>


// Create custom TwoWire instances for different I2C buses
TwoWire I2C_1 = TwoWire(0);  // I2C bus 0
TwoWire I2C_2 = TwoWire(1);  // I2C bus 1

MPU_XYZ reading;
// Use custom TwoWire instance
MPU6050 gyroscope(0x68, 400000, DLPF_256HZ, LSB_65P5, false, I2C_1);

void setup() {
    Serial.begin(115200);
    
    // Initialize I2C bus 1 with pins 41 (SDA) and 42 (SCL)
    I2C_1.begin(41, 42);
    
    // If you want to use a second I2C bus, initialize it too:
    // I2C_2.begin(other_sda_pin, other_scl_pin);
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Initialize the gyroscope after TwoWire is set up
    gyroscope.initialize(DLPF_256HZ, LSB_65P5);
}

void loop() {
  reading = gyroscope.readGyro();
  Serial.println(String(reading[0]));
  vTaskDelay(10000 / portTICK_PERIOD_MS);
}

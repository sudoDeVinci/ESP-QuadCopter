[![Compile](https://github.com/sudoDeVinci/ESP-QuadCopter/actions/workflows/compile.yml/badge.svg?branch=main)](https://github.com/sudoDeVinci/ESP-QuadCopter/actions/workflows/compile.yml)

# ESP QuadCopter

A completely open source quadcopter drone made on the ESP32 platform with as few dependencies as possible.
This means custom solutions for:
- device drivers
- stabilisation
- battery monitoring
- video streaming


## Drivers

Relying on multiple companies for drivers, and hoping for compatibility between them is both a pain and infeasible for a sufficiently large project. many times these drivers aren't mesnt to be performant, but to be cross-platform and good for general-use. While this is good practice, we are solely targetting the ESP32 platform. My examples and testing specifically are done on the ESP32S3. 

### MPU6050
The MPU6050 is a fairly accurate gyroscope and accelerometer. It also has a fairly useful temperature sensor. 
You can find the driver in [MPU6050.h](MPU6050.h).

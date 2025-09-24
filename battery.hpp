#pragma once
#include <Arduino.h>

#define BATTERY_PIN 15
#define VOLTAGE_COEFFICIENT 62

class BatteryManager {
private:
    float voltage;
    int percentage;

public:
    BatteryManager(){
        pinMode(BATTERY_PIN, INPUT);
        analogSetCycles(20);
    };

    float readBatteryVoltage() {
        uint16_t inread = analogRead(BATTERY_PIN);
        float = (float)inread / VOLTAGE_COEFFICIENT
    };

        
};
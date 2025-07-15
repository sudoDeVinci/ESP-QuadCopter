#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>
#include <unordered_map>

#define MPU6050_ADDRESS 0x68


enum DLPF_CFG {
DLPF_256HZ = 0x00,
DLPF_188HZ = 0x01,
DLPF_98HZ  = 0x02,
DLPF_42HZ  = 0x03,
DLPF_20HZ  = 0x04,
DLPF_10HZ  = 0x05,
DLPF_5HZ   = 0x06
};

struct MPU6050 {

    uint16_t address;
    uint16_t clk;
    DLPF_CFG filter;

    MPU6050(
        uint16_t address = MPU6050_ADDRESS,
        uint16_t clk = 400000,
        DLPF_CF filter = DLPF_256HZ
    ) : address(address), clk(clk), filter(filter) {
        Wire.setClock(clk);
        Wire.begin();
    }

    void setLPF()

};

#endif // MPU6050_H
#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>
#include <array>
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

enum LSB_SENSITIVITY {
    LSB_131P0 = 0x00,
    LSB_65P5 = 0x08,
    LSB_32P8 = 0x10,
    LSB_16P4 = 0x18
};

std::unordered_map<LSB_SENSITIVITY, float> LSB_MAP = {
    {LSB_SENSITIVITY::LSB_131P0, 131},
    {LSB_SENSITIVITY::LSB_65P5, 65.5},
    {LSB_SENSITIVITY::LSB_32P8, 32.8},
    {LSB_SENSITIVITY::LSB_16P4, 16.4}
};

using MPU_XYZ = std::array<float, 3>;

struct MPU6050 {

    uint16_t address;
    uint16_t clk;
    DLPF_CFG filter;
    LSB_SENSITIVITY sensitivity;
    MPU_XYZ lastGyro;

    /**
     * Writes a single byte to the specified register.
     * @param reg The register address to write to.
     * @param value The byte value to write.
     */
    void writeToReg(uint8_t reg, uint8_t value) const {
        Wire.beginTransmission(address);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }

    /**
     * Writes a single byte to the specified register without any values.
     * This is useful for operations that only require a register address.
     * @param reg The register address to write to.
     */
    void writeToReg(uint8_t reg) const {
        Wire.beginTransmission(address);
        Wire.write(reg);
        Wire.endTransmission();
    }

    MPU6050(
        uint16_t address = MPU6050_ADDRESS,
        uint16_t clk = 400000,
        DLPF_CFG filter = DLPF_256HZ,
        LSB_SENSITIVITY sensitivity = LSB_65P5
    ) : address(address), clk(clk), filter(filter), sensitivity(sensitivity) {
        Wire.setClock(clk);
        Wire.begin();
        delay(250);

        // Turn on the device with no extra configs
        powerOn();

        // set up low pass filter
        setLPF(filter);

        // set up sensitivity
        setSensitivity(sensitivity);
    }

    void powerOn(void) const {
        writeToReg(0x68, 0x00);
    }

    void setLPF(DLPF_CFG newFilter = DLPF_CFG::DLPF_10HZ) {
        this->filter = newFilter;
        writeToReg(0x1A, static_cast<uint8_t>(newFilter));
    }

    void setSensitivity(LSB_SENSITIVITY newSensitivity = LSB_SENSITIVITY::LSB_65P5) {
        this->sensitivity = newSensitivity;
        writeToReg(0x1B, static_cast<uint8_t>(newSensitivity));

    }

    MPU_XYZ readGyro() {
        MPU_XYZ buffer;

        float scaleFactor = LSB_MAP[this->sensitivity];

        uint8_t reg = 0x43;
        writeToReg(reg);

        Wire.requestFrom(address, 6);
        for (size_t i = 0; i < 3; ++i) {
            if (Wire.available() >= 2) {
                int16_t rawValue = (Wire.read() << 8) | Wire.read();
                buffer[i] = rawValue / scaleFactor;
            } else {
                buffer[i] = 0;
            }
        }

        this->lastGyro = buffer;
        return buffer;
    }

};

#endif // MPU6050_H

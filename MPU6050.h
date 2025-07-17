#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>
#include <unordered_map>
#include <array>

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
    LSB_131P0 = 0x00;
    LSB_65P5 = 0x08;
    LSB_32P8 = 0x10;
    LSB_16P4 = 0x18;
}

using MPU_XYZ = std::array<int16_t, 3>;

struct MPU6050 {

    uint16_t address;
    uint16_t clk;
    DLPF_CFG filter;
    LSB_SENSITIVITY sensitivity;

    /**
     * Writes a single byte to the specified register.
     * @param reg The register address to write to.
     * @param values The byte values to write.
     * @param size The number of bytes to write.
     */
    void writeToRegister(uint8_t* values, size_t size) {
        Wire.beginTransmission(address);
        Wire.write(reg);
        for (size_t i = 0; i < size; ++i) {
            Wire.write(values[i]);
        }
        Wire.endTransmission();
    }

    MPU6050(
        uint16_t address = MPU6050_ADDRESS,
        uint16_t clk = 400000,
        DLPF_CF filter = DLPF_256HZ,
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
        uint8_t values[] = {0x68, 0x00};
        writeToRegister(values, sizeof(values));
    }

    void setLPF(DLPF_CFG newFilter = DLFP_CFG::DLPF_10HZ) {
        this->filter = newFilter;
        uint8_t values[] = { 0x1A, newFilter };
        writeToRegister(values, sizeof(values));
    }

    void setSensitivity(LSB_SENSITIVITY newSensitivity = LSB_SENSITIVITY::LSB_65P5) {
        this->sensitivity = newSensitivity;
        uint8_t values[] = { 0x1B, newSensitivity };
        writeToRegister(values, sizeof(values));

    }

    MPU_XYZ readGyro() {
        MPU_XYZ buffer;

        uint8_t reg = 0x43;
        writeToRegister(&reg, 1);

        Wire.requestFrom(address, 6);
        for (size_t i = 0; i < 3; ++i) {
            if (Wire.available()) buffer[i] = Wire.read() << 8 | Wire.read();
            else buffer[i] = 0;
        }

        return buffer;
    }

};

#endif // MPU6050_H

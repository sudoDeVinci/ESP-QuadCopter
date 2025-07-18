#ifndef SENSOR_H
#define SENSOR_H

#include <Wire.h>
#include <array>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <mutex>

using UniqueTimedMutex = std::unique_lock<std::timed_mutex>;

/**
 * Base class for sensors.
 * Provides type-agnostic common functionality for sensor operations.
 */
struct Sensor {

private:
    mutable std::timed_mutex i2cMutex;
    static constexpr std::chrono::milliseconds I2C_TIMEOUT_MS{100};
    static constexpr TickType_t I2C_DELAY_MS = 5 / portTICK_PERIOD_MS;

public:

    /**The address of the sensor in hex.*/
    uint16_t address;
    /**The clock speed of the sensor in Hz.*/
    uint16_t clk;

    Sensor(uint16_t address, uint16_t clk)
        : address(address), clk(clk) {}

    /**
     * Writes a single byte to the specified register.
     * @param reg The register address to write to.
     * @param value The byte value to write.
     */
    void writeToReg(uint8_t reg, uint8_t value) const {
        bool aquired = false;
        {
            UniqueTimedMutex lock(i2cMutex, std::defer_lock);
            if (lock.try_lock_for(I2C_TIMEOUT_MS)) {
                aquired = true;
                Wire.beginTransmission(address);
                Wire.write(reg);
                Wire.write(value);
                Wire.endTransmission();
            } else {
                aquired = false;
                // TODO: Some logging - will handle later after base functionality is working
            }
        }
        vTaskDelay(I2C_DELAY_MS);
    }

    /**
     * Writes a single byte to the specified register without any values.
     * This is useful for operations that only require a register address.
     * @param reg The register address to write to.
     */
    void writeToReg(uint8_t reg) const {
        bool aquired = false;
        {
            UniqueTimedMutex lock(i2cMutex, std::defer_lock);
            if (lock.try_lock_for(I2C_TIMEOUT_MS)) {
                aquired = true;
                Wire.beginTransmission(address);
                Wire.write(reg);
                Wire.endTransmission();
            } else {
                aquired = false;
                // TODO: Some logging - will handle later after base functionality is working
            }
        }
        vTaskDelay(I2C_DELAY_MS);
    }

    /**
     * Find the mean of an array of numerical type T.
     * @param arr The array of type T with size N.
     * @return The mean value of the array.
     */
    template <typename T, size_t N>
    float mean(const std::array<T, N> &arr) const {
        long sum = 0;
        for (size_t i = 0; i < N; ++i) {
            sum += arr[i];
        }
        return sum / (float)N;
    }

    /**
     * Find the mean of an vector of numerical type T.
     * @param arr The vector of type T with size N.
     * @return The mean value of the array.
     */
    template <typename T>
    float mean(const std::vector<T> &vec) const {
        long sum = 0;
        for (const T& num : vec) {
            sum += num;
        }

        return sum / (float)vec.size();
    }

    /**
     * Find the standard deviation of an array of numerical type T.
     * @param arr The array of type T with size N.
     * @return The standard deviation of the array.
     */
    template <typename T, size_t N>
    float stddev(const std::array<T, N> &arr) const {
        float meanValue = mean(arr, N);
        long sum = 0;
        for (size_t i = 0; i < N; ++i) {
            sum += (arr[i] - meanValue) * (arr[i] - meanValue);
        }
        return sqrt(sum / (float)N);
    }

    /**
     * Find the quartiles of an array of numerical type T.
     * @param arr The array of type T with size N.
     * @return An array containing the first quartile, median, and third quartile.
     */
    template <typename T, size_t N>
    std::array<float, 3> quartiles(std::array<T, N> &arr) const {
        std::sort(arr.begin(), arr.end());
        float q1 = arr[N / 4];
        float q3 = arr[(3 * N) / 4];
        float median = arr[N / 2];
        
        std::array<float, 3> quartiles = {q1, median, q3};
        return quartiles;
    }

    /**
     * Remove outliers from an array of numerical type T using the IQR method.
     * @param arr The array of type T.
     * @param q1 The first quartile.
     * @param q3 The third quartile.
     * @return A vector containing the filtered values without outliers.
     */
    template <typename T, size_t N>
    std::vector<T> removeOutliers(
        const std::array<T, N> &arr,
        float q1,
        float q3
    ) const {
        float iqr = q3 - q1;
        float lower_bound = q1 - 1.5 * iqr;
        float upper_bound = q3 + 1.5 * iqr;

        std::vector<T> filtered;
        filtered.reserve(N);
        for (size_t i = 0; i < N; ++i) {
            if (arr[i] >= lower_bound && arr[i] <= upper_bound) {
                filtered.push_back(arr[i]);
            }
        }
        return filtered;
    }
};


#endif
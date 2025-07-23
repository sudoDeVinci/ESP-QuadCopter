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
 * Base class for I2C sensors.
 * Designed to be used with the ESP32 platform and the Arduino framework.
 * It includes methods for writing to registers, calculating statistics, and handling outliers.
 * Meant to be thread-safe with a timed mutex for I2C communication.
 */
struct Sensor {

    protected:
        mutable std::timed_mutex i2cMutex;
        static constexpr std::chrono::milliseconds I2C_TIMEOUT_MS{100};
        static constexpr TickType_t I2C_DELAY_MS = 5 / portTICK_PERIOD_MS;
        static constexpr TickType_t I2C_INIT_DELAY_MS = 250 / portTICK_PERIOD_MS;

    public:
        /**The SDA pin for the I2C bus.*/
        uint8_t SDA;
        /**The SCL pin for the I2C bus.*/
        uint8_t SCL;
        /**The address of the sensor in hex.*/
        uint16_t address;
        /**The clock speed of the sensor in Hz.*/
        uint32_t clk;
        /**Reference to the TwoWire instance to use for I2C communication.*/
        TwoWire& wire;

        Sensor(
            uint8_t sda,
            uint8_t scl,
            uint16_t address,
            uint32_t clk,
            TwoWire& wireInstance = Wire
        )
            : SDA(sda), SCL(scl), address(address), clk(clk), wire(wireInstance) {}

        bool begin(void) {
            if not (wire.setClock(clk)) return false;
            if not (wire.begin(sda, scl)) return false;
            vTaskDelay(I2C_INIT_DELAY_MS);
            return true;
        }

        /**
         * Writes a single byte to the specified register.
         * @param reg The register address to write to.
         * @param value The byte value to write.
         */
        void writeToReg(uint8_t reg, uint8_t value) const {
            UniqueTimedMutex lock(i2cMutex, std::defer_lock);
            if (lock.try_lock_for(I2C_TIMEOUT_MS)) {
                wire.beginTransmission(address);
                wire.write(reg);
                wire.write(value);
                wire.endTransmission();
            } else {
                // TODO: Some logging - will handle later after base functionality is working
            }
            vTaskDelay(I2C_DELAY_MS);
        }

        /**
         * Writes a single byte to the specified register without any values.
         * This is useful for operations that only require a register address.
         * @param reg The register address to write to.
         */
        void writeToReg(uint8_t reg) const {
            UniqueTimedMutex lock(i2cMutex, std::defer_lock);
            if (lock.try_lock_for(I2C_TIMEOUT_MS)) {
                wire.beginTransmission(address);
                wire.write(reg);
                wire.endTransmission();
            } else {
                // TODO: Some logging - will handle later after base functionality is working
            }
            vTaskDelay(I2C_DELAY_MS);
        }
        
        /**
         * Writes a command to the sensor.
         * The command is a 16-bit value split into two bytes.
         * @param cmd The command to write, represented as a 16-bit unsigned integer.
         */
        void writeCommand(uint16_t cmd) const {
            std::array<uint8_t, 2> cmdBytes = {
                static_cast<uint8_t>(cmd >> 8), // High byte
                static_cast<uint8_t>(cmd & 0xFF) // Low byte
            };

            UniqueTimedMutex lock(i2cMutex, std::defer_lock);
            if (lock.try_lock_for(I2C_TIMEOUT_MS)) {
                wire.beginTransmission(this->address);
                wire.write(cmdBytes.data(), cmdBytes.size());
                wire.endTransmission();
            } else {
                // TODO: Some logging - will handle later after base functionality is working
            }

            vTaskDelay(I2C_DELAY_MS);
        }

        /**
         * Performs a CRC8 calculation on the supplied values.
         * @param data  Pointer to the data to use when calculating the CRC8.
         * @param len   The number of bytes in 'data'.
         * @return The computed CRC8 value.
         */
        uint8_t crc8(const uint8_t *data, int len) const {
            const uint8_t POLYNOMIAL(0x31);
            uint8_t crc(0xFF);

            for (int j = len; j; --j) {
                crc ^= *data++;
                for (int i = 8; i; --i) {
                    crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
                }
            }
            return crc;
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
         * @tparam T The type of the elements in the array.
         * @tparam N The size of the array.
         * @param arr The array of type T with size N.
         * @param meanValue The mean value to use for the calculation. If NaN, it will be calculated from the array.
         * @return The standard deviation of the array.
         */
        template <typename T, size_t N>
        float stddev(const std::array<T, N> &arr, const float meanValue = NAN) const {
            float mval = std::isnan(meanValue) ? mean(arr) : meanValue;
            if (N == 0) return 0.0f;

            float sum = 0.0f;
            for (size_t i = 0; i < N; ++i) {
                sum += (arr[i] - mval) * (arr[i] - mval);
            }
            return sqrt(sum / (float)N);
        }

        /**
         * Find the standard deviation of a vector of numerical type T.
         * NOTE: If the meanValue is NaN, it will be calculated from the vector.
         * @tparam T The type of the elements in the vector.
         * @param vec The vector of type T.
         * @param meanValue The mean value to use for the calculation. If NaN, it will be calculated from the array.
         * @return The standard deviation of the vector.
         */
        template <typename T>
        float stddev(const std::vector<T> &vec, const float meanValue = NAN) const {
            float mval = std::isnan(meanValue) ? mean(vec) : meanValue;
            if (vec.empty()) return 0.0f;

            float sum = 0.0f;
            for (const T& num : vec) {
                sum += (num - mval) * (num - mval);
            }
            return sqrt(sum / (float)vec.size());
        }

        /**
         * Find the quartiles of an array of numerical type T.
         * NOTE: This function mutates the input array by sorting it.
         * @tparam T The type of the elements in the array.
         * @tparam N The size of the array.
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
         * Find the quartiles of an array of numerical type T.
         * NOTE: This function mutates the input array by sorting it.
         * @tparam T The type of the elements in the array.
         * @param vec The array of type T with size N.
         * @return An array containing the first quartile, median, and third quartile.
         */
        template <typename T>
        std::array<float, 3> quartiles(std::vector<T> &vec) const {
            std::sort(vec.begin(), vec.end());
            size_t N = vec.size();
            float q1 = vec[N / 4];
            float q3 = vec[(3 * N) / 4];
            float median = vec[N / 2];
            
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

        /**
         * Remove outliers from a vector of numerical type T using the IQR method.
         * @param vec The vector of type T.
         * @param q1 The first quartile.
         * @param q3 The third quartile.
         * @return A vector containing the filtered values without outliers.
         */
        template <typename T>
        std::vector<T> removeOutliers(
            const std::vector<T> &vec,
            float q1,
            float q3
        ) const {
            float iqr = q3 - q1;
            float lower_bound = q1 - 1.5 * iqr;
            float upper_bound = q3 + 1.5 * iqr;

            std::vector<T> filtered;
            filtered.reserve(vec.size());
            for (const T& value : vec) {
                if (value >= lower_bound && value <= upper_bound) {
                    filtered.push_back(value);
                }
            }
            return filtered;
        }
};


#endif
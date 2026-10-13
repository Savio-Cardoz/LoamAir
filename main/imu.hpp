#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"

namespace loamair {

// Raw 6-axis sample straight from the MPU6050 (plus die temperature).
struct ImuSample {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t temp;
};

class Imu {
public:
    Imu() = default;
    ~Imu();

    Imu(const Imu &) = delete;
    Imu &operator=(const Imu &) = delete;

    // Brings up the I2C bus, verifies WHO_AM_I, and configures the sensor.
    esp_err_t init();

    // Reads one 6-axis + temperature burst.
    esp_err_t read(ImuSample &out) const;

    // Full-scale sensitivity for the configured ranges.
    static constexpr float kAccelLsbPerG = 2048.0f;   // +/-16 g
    static constexpr float kGyroLsbPerDps = 16.4f;    // +/-2000 dps

private:
    esp_err_t write_reg(uint8_t reg, uint8_t value) const;
    esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t len) const;

    i2c_master_bus_handle_t bus_{nullptr};
    i2c_master_dev_handle_t dev_{nullptr};
};

}  // namespace loamair

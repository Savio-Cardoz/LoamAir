#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/i2c_master.h"

namespace loamair {

// I2C bus wiring for the GY-87 module (MPU6050 section only).
inline constexpr i2c_port_num_t kI2cPort = I2C_NUM_0;
inline constexpr gpio_num_t kI2cSda = GPIO_NUM_21;
inline constexpr gpio_num_t kI2cScl = GPIO_NUM_22;
inline constexpr uint32_t kI2cFreqHz = 400000;
inline constexpr uint8_t kMpu6050Addr = 0x68;

// Sampling.
inline constexpr uint32_t kSampleRateHz = 100;
inline constexpr uint32_t kSamplePeriodMs = 1000 / kSampleRateHz;

// Ring buffer sizing. At boot the largest safe internal-heap block is used for
// telemetry, leaving kHeapReserveBytes headroom for Wi-Fi + HTTP during transfer.
inline constexpr size_t kHeapReserveBytes = 80 * 1024;
inline constexpr size_t kMinBufferBytes = 16 * 1024;
inline constexpr size_t kMaxBufferBytes = 128 * 1024;

}  // namespace loamair

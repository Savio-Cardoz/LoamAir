#include "imu.hpp"

#include "config.hpp"
#include "esp_check.h"
#include "esp_log.h"

namespace loamair {
namespace {

constexpr char TAG[] = "imu";

constexpr uint8_t kRegWhoAmI = 0x75;
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegSmplrtDiv = 0x19;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegAccelXoutH = 0x3B;

constexpr uint8_t kWhoAmIExpected = 0x68;
constexpr uint8_t kPwrMgmt1Wake = 0x00;       // wake up, use internal 8 MHz oscillator
constexpr uint8_t kDlpfCfg44Hz = 0x03;        // accel 44 Hz / gyro 42 Hz digital low-pass
constexpr uint8_t kGyroFsSel2000dps = 0x18;   // FS_SEL = 3
constexpr uint8_t kAccelFsSel16g = 0x18;      // AFS_SEL = 3

constexpr int kIoTimeoutMs = 100;

int16_t be16(const uint8_t *p)
{
    return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

}  // namespace

Imu::~Imu()
{
    if (dev_ != nullptr) {
        i2c_master_bus_rm_device(dev_);
    }
    if (bus_ != nullptr) {
        i2c_del_master_bus(bus_);
    }
}

esp_err_t Imu::write_reg(uint8_t reg, uint8_t value) const
{
    const uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(dev_, buf, sizeof(buf), kIoTimeoutMs);
}

esp_err_t Imu::read_regs(uint8_t reg, uint8_t *buf, size_t len) const
{
    return i2c_master_transmit_receive(dev_, &reg, 1, buf, len, kIoTimeoutMs);
}

esp_err_t Imu::init()
{
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = kI2cPort;
    bus_cfg.sda_io_num = kI2cSda;
    bus_cfg.scl_io_num = kI2cScl;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &bus_), TAG, "I2C bus init failed");

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = kMpu6050Addr;
    dev_cfg.scl_speed_hz = kI2cFreqHz;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus_, &dev_cfg, &dev_), TAG,
                        "I2C device add failed");

    uint8_t who = 0;
    ESP_RETURN_ON_ERROR(read_regs(kRegWhoAmI, &who, 1), TAG, "WHO_AM_I read failed");
    if (who != kWhoAmIExpected) {
        ESP_LOGE(TAG, "unexpected WHO_AM_I=0x%02X (expected 0x%02X)", who, kWhoAmIExpected);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_RETURN_ON_ERROR(write_reg(kRegPwrMgmt1, kPwrMgmt1Wake), TAG, "wake failed");
    ESP_RETURN_ON_ERROR(write_reg(kRegConfig, kDlpfCfg44Hz), TAG, "DLPF config failed");
    ESP_RETURN_ON_ERROR(write_reg(kRegGyroConfig, kGyroFsSel2000dps), TAG,
                        "gyro range config failed");
    ESP_RETURN_ON_ERROR(write_reg(kRegAccelConfig, kAccelFsSel16g), TAG,
                        "accel range config failed");

    const uint8_t div = static_cast<uint8_t>(1000 / kSampleRateHz - 1);
    ESP_RETURN_ON_ERROR(write_reg(kRegSmplrtDiv, div), TAG, "sample rate config failed");

    ESP_LOGI(TAG, "MPU6050 ready (WHO_AM_I=0x%02X) @ %u Hz, SDA=GPIO%d SCL=GPIO%d", who,
             static_cast<unsigned>(kSampleRateHz), static_cast<int>(kI2cSda),
             static_cast<int>(kI2cScl));
    return ESP_OK;
}

esp_err_t Imu::read(ImuSample &out) const
{
    uint8_t raw[14];
    ESP_RETURN_ON_ERROR(read_regs(kRegAccelXoutH, raw, sizeof(raw)), TAG, "sample read failed");

    out.ax = be16(&raw[0]);
    out.ay = be16(&raw[2]);
    out.az = be16(&raw[4]);
    out.temp = be16(&raw[6]);
    out.gx = be16(&raw[8]);
    out.gy = be16(&raw[10]);
    out.gz = be16(&raw[12]);
    return ESP_OK;
}

}  // namespace loamair

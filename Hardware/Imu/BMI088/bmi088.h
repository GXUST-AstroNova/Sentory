#pragma once

#include <cstdint>

#include "imu.hpp"
#include "main.h"   // GPIO_TypeDef / ACC_CS_Pin / GYRO_CS_Pin
#include "spi.h"    // SPI_HandleTypeDef

class Bmi088 final : public Imu {
public:
    /** 加速度计量程码（直接写进 ACC_RANGE 寄存器） */
    enum class AccelRange : uint8_t {
        G3  = 0x00,   ///< ±3g   @ 10920 LSB/g
        G6  = 0x01,   ///< ±6g   @  5460 LSB/g
        G12 = 0x02,   ///< ±12g  @  2730 LSB/g
        G24 = 0x03,   ///< ±24g  @  1365 LSB/g
    };

    /** 陀螺仪量程码（直接写进 GYRO_RANGE 寄存器；注意码值顺序与加速度计相反） */
    enum class GyroRange : uint8_t {
        Dps2000 = 0x00,   ///< ±2000°/s @ 16.384 LSB/(°/s)
        Dps1000 = 0x01,   ///< ±1000°/s @ 32.768
        Dps500  = 0x02,   ///< ±500°/s  @ 65.536
        Dps250  = 0x03,   ///< ±250°/s  @ 131.072
        Dps125  = 0x04,   ///< ±125°/s  @ 262.144
    };

    struct Config {
        AccelRange accel_range = AccelRange::G6;      ///< 默认 ±6g（5460 LSB/g）
        GyroRange  gyro_range  = GyroRange::Dps2000;  ///< 默认 ±2000°/s（抗高转速削顶）

        uint8_t accel_odr = 0x08;   ///< ACC_CONF[3:0]，0x08 = 100 Hz
        ///
        /// GYRO_BANDWIDTH(0x10)[2:0]（手册 p.39 §5.5.6）。bit7 是只读位、恒为 1，没有模式含义：
        ///   0x00 = 2000 Hz ODR / 532 Hz 滤波    0x01 = 2000 Hz / 230 Hz
        ///   0x02 = 1000 Hz ODR / 116 Hz 滤波    0x03 =  400 Hz /  47 Hz
        ///   0x04 =  200 Hz /  23 Hz             0x05 =  100 Hz /  12 Hz
        ///   0x06 =  200 Hz /  64 Hz             0x07 =  100 Hz /  32 Hz
        /// 取 0x02：与 start() 的 1 kHz 节拍匹配（0x01 是 2000 Hz ODR，噪声更大）。
        uint8_t gyro_odr  = 0x02;

        GPIO_TypeDef *accel_cs_port = ACC_CS_GPIO_Port;
        uint16_t      accel_cs_pin  = ACC_CS_Pin;
        GPIO_TypeDef *gyro_cs_port  = GYRO_CS_GPIO_Port;
        uint16_t      gyro_cs_pin   = GYRO_CS_Pin;

        uint32_t spi_timeout_ms = 10U;

        // Timeout per DMA transaction, measured in HAL milliseconds.
        uint32_t xfer_timeout_ms = 5U;
    };

    explicit Bmi088(SPI_HandleTypeDef *hspi) noexcept;
    Bmi088(SPI_HandleTypeDef *hspi, const Config &cfg) noexcept;
    Bmi088(const Bmi088 &) = delete;
    Bmi088 &operator=(const Bmi088 &) = delete;
    ~Bmi088() override;

    bool init() override;
    StartResult start() override;
    ProcessResult process() override;

private:
    friend void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *);
    friend void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *);
    void onComplete() noexcept;
    void onError() noexcept;
    bool dmaReady() const noexcept;

    void selectAccel() noexcept;
    void deselectAccel() noexcept;
    void selectGyro() noexcept;
    void deselectGyro() noexcept;
    int8_t writeAccel(uint8_t reg, uint8_t value) noexcept;
    int8_t writeGyro(uint8_t reg, uint8_t value) noexcept;
    int8_t readAccel(uint8_t reg, uint8_t *out, uint16_t len) noexcept;
    int8_t readGyro(uint8_t reg, uint8_t *out, uint16_t len) noexcept;
    int8_t readAccelId() noexcept;
    int8_t readGyroId() noexcept;
    int8_t discardAccelFirstRead() noexcept;
    void readAccelErrorReg() noexcept;
    bool startReadAccel() noexcept;
    bool startReadGyro() noexcept;
    void parseAccel(ImuFeedback &sample) const noexcept;
    void parseGyro(ImuFeedback &sample) const noexcept;

    static constexpr uint16_t kMaxTransfer = 8U;
    enum class Xfer : uint8_t { Idle, Accel, Gyro, Complete, Recovering };

    SPI_HandleTypeDef *hspi_;
    Config cfg_;
    uint8_t accel_chip_id_{}, gyro_chip_id_{}, accel_err_reg_{};
    float accel_lsb_per_g_{}, gyro_lsb_per_dps_{};
    volatile Xfer xfer_ = Xfer::Idle;
    volatile uint32_t xfer_start_tick_{};
    uint32_t sample_time_ms_{};
};

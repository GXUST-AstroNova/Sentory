//
// Bosch BMI088（SPI 模式）实现
//
// 寄存器编号与常量已对照 Paparazzi bmi088_regs.h 与 Bosch 官方 bmi08_defs.h 核实。
//

#include "bmi088.h"
#include "dma_buffer.hpp"
#include "irq_guard.hpp"

#include <numbers>

namespace {

// ===========================================================================
// DMA 收发缓冲 —— 必须落在 DMA 可达的 D2 SRAM
//
// STM32H7 的通用 DMA 访问不到 DTCM(0x20000000)，而本工程栈和 .bss 都在 DTCMRAM
// （见 STM32H723XG_FLASH.ld 的 _estack）。所以这两个缓冲**绝不能**是栈上局部数组，
// 也不能是普通成员变量 —— 必须用 RAM_D2_SECTION 落到 0x30000000。
// 段定义见链接脚本的 .RAM_D2 (NOLOAD)。
//
// 放在文件作用域而不是类成员：GCC 不允许给非静态数据成员加 section 属性。
// 用匿名命名空间得到内部链接，不会与其它翻译单元冲突。
// init() 为 SPI2 的活动实例独占这些 DMA 缓冲，析构时释放。
// ===========================================================================
constexpr uint16_t kAccTotal = 20U;
constexpr uint16_t kGyroTotal = 7U;
RAM_D2_SECTION uint8_t g_tx_acc[kAccTotal]{};    ///< 加速度计 MOSI（只有 [0] 有效）
RAM_D2_SECTION uint8_t g_rx_acc[kAccTotal]{};    ///< 加速度计 MISO，DMA 目标
RAM_D2_SECTION uint8_t g_tx_gyro[kGyroTotal]{};  ///< 陀螺仪 MOSI（只有 [0] 有效）
RAM_D2_SECTION uint8_t g_rx_gyro[kGyroTotal]{};  ///< 陀螺仪 MISO，DMA 目标

// SPI2 上只有这一颗 BMI088（工程里也只有一个全局 Bmi088 实例），
// 中断里用这个指针把完成事件转回驱动实例。
Bmi088 *volatile g_active_imu = nullptr;

} // namespace

// ---------------------------------------------------------------------------
// SPI2 DMA 完成/错误回调：只做链式推进（加速度计 -> 陀螺仪），解析与提交在 process()
//
// HAL 的完成路径（核对过 stm32h7xx_hal_spi.c）：
//   SPI_DMATransmitReceiveCplt() 置 EOT 中断 -> SPI_IRQHandler() 在 EOT 分支里
//   先 SPI_CloseTransfer(hspi) 再置 HAL_SPI_STATE_READY 再回调。
// 所以回调触发时总线已关闭、末位已移出，此时释放片选、立刻发起下一笔 DMA 都是安全的。
// ---------------------------------------------------------------------------
extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if ((g_active_imu != nullptr) && (hspi == g_active_imu->hspi_)) {
        g_active_imu->onComplete();
    }
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if (g_active_imu && hspi == g_active_imu->hspi_ && hspi->ErrorCode != HAL_SPI_ERROR_NONE) {
        g_active_imu->onError();
    }
}

namespace {

// -------------------- 加速度计寄存器 --------------------
constexpr uint8_t kAccChipId    = 0x00U;
constexpr uint8_t kAccData      = 0x12U;   // 0x12..0x17：X/Y/Z 各 2 字节，小端
constexpr uint8_t kAccTemp      = 0x22U;   // 0x22..0x23
constexpr uint8_t kAccConf      = 0x40U;   // [7:4] bwp, [3:0] odr
constexpr uint8_t kAccRange     = 0x41U;
constexpr uint8_t kAccPwrConf   = 0x7CU;   // 0x00 = active, 0x03 = suspend
constexpr uint8_t kAccPwrCtrl   = 0x7DU;   // 0x04 = accel on
constexpr uint8_t kAccSoftReset = 0x7EU;
constexpr uint8_t kAccErrReg    = 0x02U;   // ACC_ERR_REG（手册 p.26 §5.3.2）

// -------------------- 陀螺仪寄存器 --------------------
constexpr uint8_t kGyroChipId    = 0x00U;
constexpr uint8_t kGyroData      = 0x02U;   // 0x02..0x07：X/Y/Z 各 2 字节，小端
constexpr uint8_t kGyroRange     = 0x0FU;
constexpr uint8_t kGyroBandwidth = 0x10U;   // bit7 = 0 -> 正常模式；[2:0] odr/滤波
constexpr uint8_t kGyroLpm1      = 0x11U;   // 0x00 = normal mode
constexpr uint8_t kGyroSoftReset = 0x14U;

// -------------------- 常量 --------------------
constexpr uint8_t kAccChipIdValue  = 0x1EU;   // BMI088 加速度计复位值
constexpr uint8_t kGyroChipIdValue = 0x0FU;   // BMI088 陀螺仪复位值
constexpr uint8_t kSoftResetCmd    = 0xB6U;

// 复位等待（取值同 Bosch 官方驱动的 BMI08_ACCEL_SOFTRESET_DELAY_MS / BMI08_GYRO_SOFTRESET_DELAY）
constexpr uint32_t kAccSoftResetDelayMs  = 1U;
/** 上电后等 ts_up：手册 p.14 §4.1「POR / 软复位后加速度计最长需要 1 ms 启动时间」 */
constexpr uint32_t kPorBootDelayMs       = 1U;
/** 切 SPI 用的 CSB 电平保持时间（远大于手册 p.46 Table 12 的 t_CSB_setup/hold = 40ns） */
constexpr uint32_t kCsToggleDelayMs      = 1U;
/**
 * 两次写访问之间必须留出的最小空闲时间（手册 p.45 §6 与 p.46 Table 12）：
 *   正常模式 ≥ 2 µs，suspend 模式 ≥ 1000 µs。
 * “In order to allow for the correct internal synchronisation of data written to the
 *  BMI088, a wait time of at least 2 µs (normal mode) or 1000 µs (suspend mode)
 *  must be followed.”
 * HAL_Delay 最小粒度 1 ms，远大于 2 µs 但绝对安全；只在 init() 里用几次，代价可忽略。
 */
constexpr uint32_t kWriteIdleDelayMs     = 1U;
constexpr uint32_t kGyroSoftResetDelayMs = 30U;
constexpr uint32_t kAccPowerUpDelayMs    = 5U;

// ACC_CONF：0xA0 = normal BWP（[7:4]），低 4 位是 ODR
constexpr uint8_t kAccConfBwpNormal = 0xA0U;

// GYRO_BANDWIDTH(0x10)：低 3 位是 ODR/滤波档位（手册 p.39 §5.5.6）。
// bit7 是【只读位、恒为 1】，手册脚注明确它没有功能、可以忽略 —— 不是“正常模式”位。
// 因此写入时只写低 3 位即可。
constexpr uint8_t kGyroBwMask    = 0x07U;
constexpr uint8_t kGyroBwResetHi = 0x80U;   ///< 只读位，写入无效果；写 1 仅为与复位值一致

// 加速度计灵敏度表，下标 = ACC_RANGE 码值
constexpr float kAccelLsbPerG[4] = {10920.0f, 5460.0f, 2730.0f, 1365.0f};

// 陀螺仪灵敏度表，下标 = GYRO_RANGE 码值（注意与加速度计的码值顺序相反）
constexpr float kGyroLsbPerDps[5] = {16.384f, 32.768f, 65.536f, 131.072f, 262.144f};

// 温度（手册 p.28 §5.3.7）：11 位 2 的补码，0.125 °C/LSB，零点偏移 +23 °C
constexpr float kTempLsbPerDegC = 0.125f;
constexpr float kTempOffsetDegC = 23.0f;
/** TEMP_MSB 等于该值时手册标注为 Invalid（数据无效） */
constexpr uint8_t kTempMsbInvalid = 0x80U;

/** 小端拼一个 int16 */
inline int16_t toInt16Le(const uint8_t *p) noexcept {
    return static_cast<int16_t>(static_cast<uint16_t>(p[1]) << 8 | static_cast<uint16_t>(p[0]));
}

} // namespace

// -------------------- 构造 / 析构 --------------------

Bmi088::Bmi088(SPI_HandleTypeDef *hspi) noexcept
    : Bmi088(hspi, Config{}) {
}

Bmi088::Bmi088(SPI_HandleTypeDef *hspi, const Config &cfg) noexcept
    : hspi_(hspi), cfg_(cfg) {
    // 构造函数里刻意不碰 GPIO：全局对象可能在 MX_GPIO_Init() 之前被构造。
    // 片选状态统一由 init() 第一步接管。
}

Bmi088::~Bmi088() {
    if (g_active_imu == this) {
        g_active_imu = nullptr;
        if (xfer_ != Xfer::Idle && xfer_ != Xfer::Complete) {
            (void)HAL_SPI_Abort(hspi_);
        }
        deselectAccel();
        deselectGyro();
    }
}

// -------------------- 片选 --------------------

void Bmi088::selectAccel() noexcept {
    HAL_GPIO_WritePin(cfg_.accel_cs_port, cfg_.accel_cs_pin, GPIO_PIN_RESET);
}

void Bmi088::deselectAccel() noexcept {
    HAL_GPIO_WritePin(cfg_.accel_cs_port, cfg_.accel_cs_pin, GPIO_PIN_SET);
}

void Bmi088::selectGyro() noexcept {
    HAL_GPIO_WritePin(cfg_.gyro_cs_port, cfg_.gyro_cs_pin, GPIO_PIN_RESET);
}

void Bmi088::deselectGyro() noexcept {
    HAL_GPIO_WritePin(cfg_.gyro_cs_port, cfg_.gyro_cs_pin, GPIO_PIN_SET);
}

// -------------------- 低层 SPI 事务 --------------------

/**
 * 写寄存器：CS 低 -> [reg & 0x7F, value] -> CS 高
 * 加速度计与陀螺仪写法一致，都没有 dummy 字节。
 */
int8_t Bmi088::writeAccel(uint8_t reg, uint8_t value) noexcept {
    uint8_t buf[2];
    buf[0] = static_cast<uint8_t>(reg & 0x7FU);
    buf[1] = value;

    selectAccel();
    const HAL_StatusTypeDef st =
        HAL_SPI_Transmit(hspi_, buf, 2U, cfg_.spi_timeout_ms);
    deselectAccel();

    return (st == HAL_OK) ? 0 : -1;
}

int8_t Bmi088::writeGyro(uint8_t reg, uint8_t value) noexcept {
    uint8_t buf[2];
    buf[0] = static_cast<uint8_t>(reg & 0x7FU);
    buf[1] = value;

    selectGyro();
    const HAL_StatusTypeDef st =
        HAL_SPI_Transmit(hspi_, buf, 2U, cfg_.spi_timeout_ms);
    deselectGyro();

    return (st == HAL_OK) ? 0 : -1;
}

/**
 * 读加速度计：CS 低 -> [reg | 0x80, dummy, data...] -> CS 高
 * 加速度计在 SPI 下地址之后会多回一个 dummy 字节，数据从第 3 个字节开始。
 */
int8_t Bmi088::readAccel(uint8_t reg, uint8_t *out, uint16_t len) noexcept {
    const uint16_t total = static_cast<uint16_t>(1U + 1U + len);   // 地址 + dummy + 数据
    if (hspi_ == nullptr || out == nullptr || total > kMaxTransfer) {
        return -1;
    }

    uint8_t tx[kMaxTransfer];
    uint8_t rx[kMaxTransfer];
    tx[0] = static_cast<uint8_t>(reg | 0x80U);
    for (uint16_t i = 1U; i < total; ++i) {
        tx[i] = 0U;
        rx[i] = 0U;
    }
    rx[0] = 0U;

    selectAccel();
    const HAL_StatusTypeDef st =
        HAL_SPI_TransmitReceive(hspi_, tx, rx, total, cfg_.spi_timeout_ms);
    deselectAccel();

    if (st != HAL_OK) {
        return -1;
    }
    for (uint16_t i = 0U; i < len; ++i) {
        out[i] = rx[2U + i];   // 跳过 1 字节地址 + 1 字节 dummy
    }
    return 0;
}

/**
 * 读陀螺仪：CS 低 -> [reg | 0x80, data...] -> CS 高（无 dummy 字节）
 */
int8_t Bmi088::readGyro(uint8_t reg, uint8_t *out, uint16_t len) noexcept {
    const uint16_t total = static_cast<uint16_t>(1U + len);   // 地址 + 数据
    if (hspi_ == nullptr || out == nullptr || total > kMaxTransfer) {
        return -1;
    }

    uint8_t tx[kMaxTransfer];
    uint8_t rx[kMaxTransfer];
    tx[0] = static_cast<uint8_t>(reg | 0x80U);
    for (uint16_t i = 1U; i < total; ++i) {
        tx[i] = 0U;
        rx[i] = 0U;
    }
    rx[0] = 0U;

    selectGyro();
    const HAL_StatusTypeDef st =
        HAL_SPI_TransmitReceive(hspi_, tx, rx, total, cfg_.spi_timeout_ms);
    deselectGyro();

    if (st != HAL_OK) {
        return -1;
    }
    for (uint16_t i = 0U; i < len; ++i) {
        out[i] = rx[1U + i];   // 跳过 1 字节地址
    }
    return 0;
}

int8_t Bmi088::readAccelId() noexcept {
    uint8_t id = 0U;
    if (readAccel(kAccChipId, &id, 1U) != 0) {
        return -1;
    }
    accel_chip_id_ = id;
    return 0;
}

// 手册 p.45 §6.1：加速度计切到 SPI 之后的第一次读内容不可预测，必须丢掉。
// 这里刻意不复用 readAccelId()，避免把那个无效值写进 accel_chip_id_ 造成误解。
int8_t Bmi088::discardAccelFirstRead() noexcept {
    uint8_t dummy = 0U;
    return readAccel(kAccChipId, &dummy, 1U);
}

void Bmi088::readAccelErrorReg() noexcept {
    uint8_t v = 0U;
    if (readAccel(kAccErrReg, &v, 1U) == 0) {
        accel_err_reg_ = v;
    }
}

int8_t Bmi088::readGyroId() noexcept {
    uint8_t id = 0U;
    if (readGyro(kGyroChipId, &id, 1U) != 0) {
        return -1;
    }
    gyro_chip_id_ = id;
    return 0;
}

// -------------------- 基类接口 --------------------

bool Bmi088::init() {
    if (g_active_imu == this) return true;
    if (!dmaReady() || hspi_->Instance != SPI2 || g_active_imu != nullptr ||
        !cfg_.accel_cs_port || !cfg_.gyro_cs_port) {
        return false;
    }

    const uint8_t acc_range = static_cast<uint8_t>(cfg_.accel_range);
    const uint8_t gyr_range = static_cast<uint8_t>(cfg_.gyro_range);
    if (acc_range > 3U || gyr_range > 4U) {
        return false;
    }

    // 1) 两个片选都释放，等上电稳定（手册 p.14 §4.1：POR 后加速度计需要 ts_up ≤ 1 ms）
    deselectAccel();
    deselectGyro();
    HAL_Delay(kPorBootDelayMs);

    // 2) 把加速度计切到 SPI（手册 p.13 §3、p.45 §6.1）
    //    加速度计上电后处于 I2C 模式，必须检测到 CSB1 的【上升沿】才切到 SPI。
    //    这里先做一次纯上升沿，让随后的软复位命令能被正确接收。
    //    （注意：旧注释写的「低电平锁到 SPI」与手册相反，已按 p.45 §6.1 更正。）
    selectAccel();
    HAL_Delay(kCsToggleDelayMs);
    deselectAccel();
    HAL_Delay(kCsToggleDelayMs);

    // 3) 加速度计软复位（此时器件已在 SPI 模式，命令才会被识别）
    if (writeAccel(kAccSoftReset, kSoftResetCmd) != 0) {
        return false;
    }
    HAL_Delay(kAccSoftResetDelayMs);

    // 4) 【关键】丢弃切到 SPI 之后的第一次读（手册 p.45 §6.1）
    //    原文：「...could perform a dummy SPI read operation, e.g. of register
    //          ACC_CHIP_ID (the obtained value will be invalid).」
    //    软复位会重新初始化接口（p.14 §4.1），复位事务末尾的 CSB1 上升沿又切了一次
    //    SPI，所以这里必须再丢一次 —— 少了这一步，第 5 步读到的就是这个无效值。
    if (discardAccelFirstRead() != 0) {
        return false;
    }

    // 5) 加速度计 chip id 自检（必须是「第二次读」，即丢弃 dummy 之后的那次）
    if (readAccelId() != 0 || accel_chip_id_ != kAccChipIdValue) {
        return false;
    }

    // 6) 顺带取一次错误寄存器（手册 p.26 §5.3.2），纯诊断，不影响成功判定
    readAccelErrorReg();

    // 7) 加速度计上电 + 量程/ODR
    if (writeAccel(kAccPwrCtrl, 0x04U) != 0) {   // accel on
        return false;
    }
    HAL_Delay(kAccPowerUpDelayMs);

    if (writeAccel(kAccPwrConf, 0x00U) != 0) {   // active（非 suspend）
        return false;
    }
    // 手册 p.45 §6 / p.46 Table 12：两次写访问之间至少留 2 µs（正常模式）/1000 µs（suspend）
    HAL_Delay(kWriteIdleDelayMs);
    if (writeAccel(kAccConf,
                   static_cast<uint8_t>(kAccConfBwpNormal |
                                        (cfg_.accel_odr & 0x0FU))) != 0) {
        return false;
    }
    HAL_Delay(kWriteIdleDelayMs);
    if (writeAccel(kAccRange, static_cast<uint8_t>(cfg_.accel_range)) != 0) {
        return false;
    }

    // 8) 陀螺仪软复位 + chip id 自检（陀螺没有 dummy 读问题，见手册 p.45 §6.1）
    if (writeGyro(kGyroSoftReset, kSoftResetCmd) != 0) {
        return false;
    }
    HAL_Delay(kGyroSoftResetDelayMs);

    if (readGyroId() != 0 || gyro_chip_id_ != kGyroChipIdValue) {
        return false;
    }

    // 9) 陀螺仪量程/带宽/功耗模式
    if (writeGyro(kGyroRange, static_cast<uint8_t>(cfg_.gyro_range)) != 0) {
        return false;
    }
    HAL_Delay(kWriteIdleDelayMs);   // 手册 p.46 Table 12：t_IDLE_wacc ≥ 2 µs
    if (writeGyro(kGyroBandwidth,
                  static_cast<uint8_t>(kGyroBwResetHi |               // 只读位，写 1 与复位值一致
                                       (cfg_.gyro_odr & kGyroBwMask))) != 0) {
        return false;
    }
    HAL_Delay(kWriteIdleDelayMs);
    if (writeGyro(kGyroLpm1, 0x00U) != 0) {      // normal mode（手册 p.40 §5.5.7：0x00=normal）
        return false;
    }

    // 10) 量程码 -> 灵敏度
    accel_lsb_per_g_ = kAccelLsbPerG[acc_range];
    gyro_lsb_per_dps_ = kGyroLsbPerDps[gyr_range];

    g_active_imu = this;
    return true;
}

// -------------------- 诊断 --------------------

bool Bmi088::dmaReady() const noexcept {
    return (hspi_ != nullptr) && (hspi_->hdmatx != nullptr) && (hspi_->hdmarx != nullptr);
}

// Each transaction holds its own CS through the SPI end-of-transfer interrupt.
bool Bmi088::startReadAccel() noexcept {
    g_tx_acc[0] = kAccData | 0x80U;
    for (uint16_t i = 1; i < kAccTotal; ++i) g_tx_acc[i] = 0;
    sample_time_ms_ = HAL_GetTick();
    xfer_start_tick_ = sample_time_ms_;
    xfer_ = Xfer::Accel;
    selectAccel();
    return HAL_SPI_TransmitReceive_DMA(hspi_, g_tx_acc, g_rx_acc, kAccTotal) == HAL_OK;
}

bool Bmi088::startReadGyro() noexcept {
    g_tx_gyro[0] = kGyroData | 0x80U;
    for (uint16_t i = 1; i < kGyroTotal; ++i) g_tx_gyro[i] = 0;
    xfer_start_tick_ = HAL_GetTick();
    xfer_ = Xfer::Gyro;
    selectGyro();
    return HAL_SPI_TransmitReceive_DMA(hspi_, g_tx_gyro, g_rx_gyro, kGyroTotal) == HAL_OK;
}

void Bmi088::onComplete() noexcept {
    // HAL closes the transaction and sets READY before dispatching EOT.
    if (hspi_->State != HAL_SPI_STATE_READY) return;
    if (HAL_GetTick() - xfer_start_tick_ > cfg_.xfer_timeout_ms) {
        onError();
        return;
    }
    if (xfer_ == Xfer::Accel) {
        deselectAccel();
        if (!startReadGyro()) onError();
    } else if (xfer_ == Xfer::Gyro) {
        deselectGyro();
        xfer_ = Xfer::Complete;
    }
}

void Bmi088::onError() noexcept {
    if (xfer_ == Xfer::Accel || xfer_ == Xfer::Gyro) {
        deselectAccel();
        deselectGyro();
        xfer_ = Xfer::Recovering;
    }
}

void Bmi088::parseAccel(ImuFeedback &sample) const noexcept {
    // g_rx_acc 布局：0=地址回读, 1=dummy, 2..7=ACC_X/Y/Z(0x12..0x17), 8..17=保留, 18..19=TEMP(0x22..0x23)
    for (uint8_t i = 0U; i < 3U; ++i) {
        sample.accel_m_s2[i] =
            static_cast<float>(toInt16Le(&g_rx_acc[2U + 2U * i])) * (9.80665f / accel_lsb_per_g_);
    }

    // ---- 温度（手册 p.28 §5.3.7）----
    //   Temp_uint11 = TEMP_MSB*8 + TEMP_LSB/32   （TEMP_MSB 是高 8 位，TEMP_LSB 的高 3 位是低 3 位）
    //   if (Temp_uint11 > 1023) Temp_int11 = Temp_uint11 - 2048;  else Temp_int11 = Temp_uint11;
    //   Temperature = Temp_int11 * 0.125 + 23
    //   注意：11 位是【2 的补码】，必须做符号扩展；TEMP_MSB == 0x80 表示数据无效。
    //   与旧实现逐行等价，只是数据来源从局部 raw[] 换成 DMA 缓冲的下标 18/19。
    const uint16_t temp_u11 =
        static_cast<uint16_t>((static_cast<uint16_t>(g_rx_acc[18]) << 3) |   // TEMP_MSB(0x22)
                              (static_cast<uint16_t>(g_rx_acc[19]) >> 5));    // TEMP_LSB(0x23)
    const int16_t temp_i11 = (temp_u11 > 1023U)
                                 ? static_cast<int16_t>(temp_u11 - 2048U)
                                 : static_cast<int16_t>(temp_u11);
    sample.temperature_c =
        static_cast<float>(temp_i11) * kTempLsbPerDegC + kTempOffsetDegC;
}

void Bmi088::parseGyro(ImuFeedback &sample) const noexcept {
    // g_rx_gyro 布局：0=地址回读, 1..6=GYRO_X/Y/Z(0x02..0x07)
    for (uint8_t i = 0U; i < 3U; ++i) {
        sample.gyro_rad_s[i] =
            static_cast<float>(toInt16Le(&g_rx_gyro[1U + 2U * i])) *
            (std::numbers::pi_v<float> / (180.0f * gyro_lsb_per_dps_));
    }
}

// -------------------- 基类接口：异步采样 --------------------

StartResult Bmi088::start() {
    if (g_active_imu != this || !dmaReady()) return StartResult::Error;
    if (xfer_ != Xfer::Idle) return StartResult::Skipped;
    if (startReadAccel()) return StartResult::Started;
    onError();
    return StartResult::Error;
}

ProcessResult Bmi088::process() {
    if (g_active_imu != this) return ProcessResult::Error;
    {
        IrqGuard guard;
        if ((xfer_ == Xfer::Accel || xfer_ == Xfer::Gyro) &&
            HAL_GetTick() - xfer_start_tick_ > cfg_.xfer_timeout_ms) {
            xfer_ = Xfer::Recovering;
        }
        if (xfer_ != Xfer::Recovering && xfer_ != Xfer::Complete) {
            return ProcessResult::Pending;
        }
    }

    if (xfer_ == Xfer::Recovering) {
        // Keep starts and completion callbacks gated until HAL stops DMA and clears EOT.
        const auto status = HAL_SPI_Abort(hspi_);
        deselectAccel();
        deselectGyro();
        // HAL may have closed SPI on error before aborting its second DMA stream.
        for (auto *stream : {hspi_->hdmarx, hspi_->hdmatx}) {
            if (HAL_DMA_GetState(stream) == HAL_DMA_STATE_BUSY) {
                (void)HAL_DMA_Abort(stream);
            }
        }
        if (status == HAL_OK &&
            HAL_DMA_GetState(hspi_->hdmarx) == HAL_DMA_STATE_READY &&
            HAL_DMA_GetState(hspi_->hdmatx) == HAL_DMA_STATE_READY) {
            xfer_ = Xfer::Idle;
        }
        return ProcessResult::Error;
    }

    if (g_rx_acc[18] == kTempMsbInvalid) {
        xfer_ = Xfer::Idle;
        return ProcessResult::Error;
    }
    ImuFeedback sample{};
    sample.sample_time_ms = sample_time_ms_;
    parseAccel(sample);
    parseGyro(sample);
    publish(sample);
    xfer_ = Xfer::Idle;
    return ProcessResult::Updated;
}

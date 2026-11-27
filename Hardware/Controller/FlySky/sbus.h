//
// 富斯 FS-iA6B (iA6B) 的 SBUS 接收解码
//
// 上游：FS-i6 遥控器 + iA6B 接收机（已刷 10 通道 SBUS 固件）。
// 协议：Futaba SBUS —— 100000 baud / 8 数据位 / 偶校验 / 2 停止位 / 反相 TTL。
//       本机 SBUS 已由板上硬件反相器转成正相，接 UART5_RX (PB12)。
//
// ---------------------------------------------------------------------------
// 硬件与 CubeMX 侧（参数全部由 MX_UART5_Init() 生成，本类不改配置）
// ---------------------------------------------------------------------------
//   UART5: PB12 = UART5_RX (AF8)，PC12 = UART5_TX
//          BaudRate = 100000，WordLength = UART_WORDLENGTH_9B（9B + 偶校验 = 8 数据位），
//          Parity = UART_PARITY_EVEN，StopBits = UART_STOPBITS_2
//   DMA  : DMA1_Stream0 <- DMAMUX UART5_RX，外设->内存，BYTE/BYTE，
//          Mode = NORMAL，FIFOMode = DISABLE，Priority = VERY_HIGH
//   NVIC : UART5_IRQn 抢占优先级 5；DMA1_Stream0_IRQn 抢占优先级 5
//
// ---------------------------------------------------------------------------
// 接收方式（做法与 RM25 application.c:412-435 的 DR16 一致）
// ---------------------------------------------------------------------------
//   HAL_UARTEx_ReceiveToIdle_DMA(huart5, s_rx_buf_, 32)
//   一帧收完后在 RxEventCallback 里：
//     1) HT（半传输）事件直接 return —— 此时 DMA 还在跑，重挂会 BUSY
//     2) 先重新挂载接收，再解析（顺序不能反，否则可能丢下一帧）
//     3) 校验帧头/帧尾后解 16 个 11-bit 通道
//
// ---------------------------------------------------------------------------
// 故障恢复（与 Hardware/bsp/uart_bsp 的分工）
// ---------------------------------------------------------------------------
//   HAL 的 UART 回调只有一个全局入口（HAL_UART_ErrorCallback /
//   HAL_UARTEx_RxEventCallback），这两个强符号统一由 uart_bsp.cpp 定义并按
//   huart 分发。本类在构造时把自己注册进 uart_bsp 的端口表：
//       Register({huart_, this, rx_trampoline, error_trampoline,
//                 stale_trampoline, recover_trampoline});
//   析构时 Unregister(this)。因此：
//     - 中断里的 OnError 只做分类计数（PE/NE/FE/ORE/DMA）与 countError()，
//       绝不在中断里 arm()；恢复动作交给 App 的故障定时器按策略调度。
//     - App 通过 uart_bsp::Get(id) 拿到 is_stale / recover 函数指针，或直接
//       调用本类的 Is_Stale() / Recover_Soft() / Recover_Hard()。
//
// ---------------------------------------------------------------------------
// 帧格式（25 字节）
// ---------------------------------------------------------------------------
//   byte0      : 0x0F 帧头
//   byte1..22  : 16 通道 x 11 bit，LSB 在前（176 bit = 22 字节）
//   byte23     : bit0 = ch17，bit1 = ch18，bit2 = 丢帧，bit3 = 失控
//   byte24     : 0x00 帧尾
//   通道值 172 = -100%，992 = 中位，1811 = +100%（标准量程）
//
// snapshot().channels 保存一帧的 16 个原始槽位，前 10 路为此接收机的实时通道。
// channel(0) 读取第一路的归一化值，snapshot().failsafe 读取帧内失控标志。

#ifndef DM_02_BSP_SBUS_H
#define DM_02_BSP_SBUS_H

#include <cstdint>

#include "main.h"              // UART_HandleTypeDef
#include "base_rc_receiver.h"  // RcReceiver

class Sbus final : public RcReceiver {
public:
    explicit Sbus(UART_HandleTypeDef *huart) noexcept;
    Sbus(const Sbus&) = delete;
    Sbus& operator=(const Sbus&) = delete;
    ~Sbus() noexcept override;
    bool init() noexcept override;

private:
    static void rx_trampoline(void *ctx, UART_HandleTypeDef *huart, std::uint16_t size) noexcept;
    static void error_trampoline(void *ctx, UART_HandleTypeDef *huart) noexcept;
    static bool stale_trampoline(void *ctx, std::uint32_t now_ms) noexcept;
    static bool recover_trampoline(void *ctx, bool hard) noexcept;

    // -------------------- 协议常量 --------------------
    static constexpr uint8_t  kFrameSize = 25U;      ///< SBUS 定长帧
    static constexpr uint8_t  kHeader    = 0x0FU;    ///< byte0 帧头
    static constexpr uint8_t  kFooter    = 0x00U;    ///< byte24 帧尾
    static constexpr uint8_t  kFlagCh17  = 0x01U;    ///< byte23 bit0
    static constexpr uint8_t  kFlagCh18  = 0x02U;    ///< byte23 bit1
    static constexpr uint8_t  kFlagLost  = 0x04U;    ///< byte23 bit2 丢帧
    static constexpr uint8_t  kFlagFail  = 0x08U;    ///< byte23 bit3 失控
    static constexpr uint16_t kRawMin    = 242;     ///< -100%
    static constexpr uint16_t kRawMid    = 1024;     ///< 中位
    static constexpr uint16_t kRawMax    = 1802;    ///< +100%

    /** DMA 接收缓冲区大小：一帧 25 字节，留余量（与 RM25 的 32 一致） */
    static constexpr uint16_t kRxBufSize = 32U;

    /** 停帧判定门限：收到过帧后超过这么久没有新帧即认为接收停住 */
    static constexpr uint32_t kStaleMs = 500U;

    // -------------------- HAL 回调入口（由 uart_bsp 的 trampoline 调用） --------------------

    /** 对应 HAL_UARTEx_RxEventCallback；huart 不匹配时直接返回 */
    void OnRxEvent(UART_HandleTypeDef *huart, uint16_t size) noexcept;

    /** 对应 HAL_UART_ErrorCallback；只做错误分类计数，不在这里 arm() */
    void OnError(UART_HandleTypeDef *huart) noexcept;

    // -------------------- 故障恢复（任务上下文调用） --------------------

    /** 软恢复：AbortReceive -> 重新挂载接收。返回重挂是否成功。 */
    bool Recover_Soft() noexcept;

    /**
     * 硬恢复：AbortReceive -> DeInit -> Init（复用句柄里的 Init 配置，
     * 不调用 MX_xxx_Init）-> 重新挂载接收。返回整套是否成功。
     */
    bool Recover_Hard() noexcept;

    /**
     * 是否停帧：距最近一帧（或首次 arm）已经 >= kStaleMs。
     * 上电后从未收到帧也会在超时后返回 true，避免接收机静默时永不恢复。
     */
    bool Is_Stale(uint32_t now_ms) const noexcept;

    /**
     * 把一帧 25 字节 SBUS 数据解成 16 个 11 bit 通道值。
     * 只做位解包，不校验帧头/帧尾。
     */
    static void Decode(const uint8_t *frame, uint16_t *out) noexcept;

    int8_t arm() noexcept;

    UART_HandleTypeDef *huart_ = nullptr;   ///< 本实例占用的串口句柄
    uint8_t  port_id_ = 0xFFU;              ///< uart_bsp 注册 id（0xFF = 未注册）
    uint32_t decode_fail_ = 0U;             ///< 解帧失败计数
    uint32_t rearm_fail_  = 0U;             ///< 重挂失败计数
    uint32_t rearm_ok_    = 0U;             ///< 重挂成功计数
    uint32_t recover_ok_  = 0U;             ///< 恢复成功计数
    uint32_t boot_tick_   = 0U;             ///< 最近一次 Init/arm 的时刻（首帧超时基准）

    uint32_t error_count_{};
    uint32_t pe_count_  = 0U;               ///< UART_ERROR_PE
    uint32_t ne_count_  = 0U;               ///< UART_ERROR_NE
    uint32_t fe_count_  = 0U;               ///< UART_ERROR_FE
    uint32_t ore_count_ = 0U;               ///< UART_ERROR_ORE
    uint32_t dma_count_ = 0U;               ///< UART_ERROR_DMA

    /** DMA 接收缓冲：定义在 .cpp，用 RAM_D2_SECTION 落到 D2 SRAM（DMA 可访问） */
    static uint8_t s_rx_buf_[kRxBufSize];
};

#endif // DM_02_BSP_SBUS_H

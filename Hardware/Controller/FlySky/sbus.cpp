//
// 富斯 FS-iA6B SBUS 接收解码实现
//
// 接收链路：UART5 (PB12/RX) -> DMA1_Stream0 -> IDLE 中断 -> uart_bsp 分发 ->
//           本类 OnRxEvent。回调处理顺序与 RM25 application.c:412-435 的
//           DR16 完全一致。
//
// 故障链路：HAL_UART_ErrorCallback -> uart_bsp 分发 -> 本类 OnError（只记
//           分类计数）; 恢复动作由 App 的故障定时器调用 Recover_Soft /
//           Recover_Hard。
//

#include "sbus.h"

#include "uart_bsp.h"  // 端口注册 + 硬恢复的公共外设级实现
#include "dma_buffer.hpp"
#include <algorithm>

// ===========================================================================
// DMA 接收缓冲区
//   STM32H7 的通用 DMA 访问不到 DTCM(0x20000000)，而本工程 .data/.bss 全在 DTCM，
//   所以缓冲区必须用 RAM_D2_SECTION 落到 D2 SRAM(0x30000000)。
//   段定义见 STM32H723XG_FLASH.ld 里的 .RAM_D2 (NOLOAD)。
//   NOLOAD 意味着上电后内容不确定 —— 对本场景无影响，缓冲区每次由 DMA 填满。
// ===========================================================================
RAM_D2_SECTION uint8_t Sbus::s_rx_buf_[Sbus::kRxBufSize];

// ===========================================================================
// uart_bsp 回调 trampoline（Port 里的回调不认 this，这里转成成员调用）
// ===========================================================================
namespace {
Sbus* active_receiver = nullptr; // one board DMA receive buffer
}

void Sbus::rx_trampoline(void *ctx, UART_HandleTypeDef *huart, std::uint16_t size) noexcept {
    static_cast<Sbus *>(ctx)->OnRxEvent(huart, size);
}

void Sbus::error_trampoline(void *ctx, UART_HandleTypeDef *huart) noexcept {
    static_cast<Sbus *>(ctx)->OnError(huart);
}

bool Sbus::stale_trampoline(void *ctx, std::uint32_t now_ms) noexcept {
    return static_cast<Sbus *>(ctx)->Is_Stale(now_ms);
}

bool Sbus::recover_trampoline(void *ctx, bool hard) noexcept {
    Sbus *self = static_cast<Sbus *>(ctx);
    return hard ? self->Recover_Hard() : self->Recover_Soft();
}


// ===========================================================================
// 生命周期
// ===========================================================================

Sbus::Sbus(UART_HandleTypeDef *huart) noexcept : RcReceiver(kRawMin, kRawMid, kRawMax), huart_(huart) {

    // init() checks that this port can receive callbacks before arming DMA.
    const int8_t id = uart_bsp::Register({huart_,
                                          this,
                                          rx_trampoline,
                                          error_trampoline,
                                          stale_trampoline,
                                          recover_trampoline});
    if (id >= 0) {
        port_id_ = static_cast<uint8_t>(id);
    }
}

Sbus::~Sbus() noexcept {
    uart_bsp::Unregister(this);
    if (active_receiver == this) {
        active_receiver = nullptr;
        (void)HAL_UART_AbortReceive(huart_);
    }
}

int8_t Sbus::arm() noexcept {
    if (huart_ == nullptr) {
        return -1;
    }
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart_, s_rx_buf_, kRxBufSize) != HAL_OK) {
        return -1;
    }
    return 0;
}

bool Sbus::init() noexcept {
    if (port_id_ == 0xff || (active_receiver && active_receiver != this)) return false;
    if (snapshot().ready) return true;
    active_receiver = this;
    if (arm() != 0) {
        active_receiver = nullptr;
        return false;
    }
    setReady(true);
    boot_tick_ = HAL_GetTick();
    // 诊断：上板时可直接在这里打断点确认优先级对不对
    // (HAL_NVIC_GetPriority(UART5_IRQn) 应为 5，DMA1_Stream0_IRQn 应为 5)
    return true;
}

// ===========================================================================
// HAL 回调入口（由 uart_bsp 的 trampoline 调用）
// ===========================================================================

void Sbus::OnRxEvent(UART_HandleTypeDef *huart, uint16_t size) noexcept {
    if (huart != huart_ || active_receiver != this) {
        return;   // 不是本实例的串口
    }

    // 1) 半传输事件直接返回：此时 DMA 还在跑、RxState 仍是 BUSY_RX，
    //    重新挂载会拿到 HAL_BUSY，而且本次数据是被截断的半帧。
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) {
        return;
    }

    // Capture this frame before rearming DMA on the same memory.
    uint8_t received[kRxBufSize];
    const auto avail = std::min<uint16_t>(size, kRxBufSize);
    std::copy_n(s_rx_buf_, avail, received);
    if (arm() != 0) {
        ++rearm_fail_;
    } else {
        ++rearm_ok_;
    }

    // 3) 找帧头 + 校验帧尾
    const uint8_t *f = received;

    if ((avail >= kFrameSize) && (f[0] != kHeader)) {
        // 帧头错位（上电对齐、线路毛刺）：在本次收到的数据里重新找一帧完整的
        for (uint8_t k = 1U; static_cast<uint8_t>(k + kFrameSize) <= avail; ++k) {
            if ((f[k] == kHeader) && (f[k + kFrameSize - 1U] == kFooter)) {
                f += k;
                break;
            }
        }
    }

    if ((avail < kFrameSize) || (f[0] != kHeader) || (f[kFrameSize - 1U] != kFooter)) {
        ++decode_fail_;
        return;
    }

    uint16_t ch[kMaxChannels];
    Decode(f, ch);
    markUpdated(ch, kMaxChannels,
                (f[23] & kFlagFail) != 0U,    // bit3 失控
                (f[23] & kFlagLost) != 0U);   // bit2 丢帧
}

void Sbus::OnError(UART_HandleTypeDef *huart) noexcept {
    if ((huart != nullptr) && (huart != huart_)) {
        return;   // 不是本实例的串口
    }

    // 只做分类计数，绝不在中断里 arm()/DeInit()/Init()：
    // 由业务层根据 pending/stale 调度恢复。
    // huart 可能为 nullptr（防御性调用），此时只累加总错误数。
    const std::uint32_t code = (huart != nullptr) ? huart->ErrorCode : 0U;

    if ((code & HAL_UART_ERROR_PE) != 0U) {
        ++pe_count_;
    }
    if ((code & HAL_UART_ERROR_NE) != 0U) {
        ++ne_count_;
    }
    if ((code & HAL_UART_ERROR_FE) != 0U) {
        ++fe_count_;
    }
    if ((code & HAL_UART_ERROR_ORE) != 0U) {
        ++ore_count_;
    }
    if ((code & HAL_UART_ERROR_DMA) != 0U) {
        ++dma_count_;
    }
    ++error_count_;
}

// ===========================================================================
// 故障恢复（任务上下文）
// ===========================================================================

bool Sbus::Recover_Soft() noexcept {
    if (active_receiver != this) {
        return false;
    }
    // AbortReceive 会停掉 DMA/IDLE 中断并把 RxState 复位到 READY；
    // 之后再 arm() 重新挂载一帧接收。
    const bool aborted = (HAL_UART_AbortReceive(huart_) == HAL_OK);
    if (arm() != 0) {
        ++rearm_fail_;
        return false;
    }
    ++rearm_ok_;
    if (!aborted) {
        return false;   // DMA 中止超时，数据通路可能仍然被占着
    }
    ++recover_ok_;
    return true;
}

bool Sbus::Recover_Hard() noexcept {
    if (active_receiver != this) {
        return false;
    }
    // 复用 uart_bsp 的公共实现：AbortReceive -> DeInit -> Init，并在过程中
    // 只屏蔽该 USART 自己的 NVIC 线。Init 用的是句柄里 MX 阶段写好的配置，
    // 不调用 MX_UART5_Init()。
    const bool restored = uart_bsp::Restore_Raw(huart_);
    if (arm() != 0) {
        ++rearm_fail_;
        return false;
    }
    ++rearm_ok_;
    if (!restored) {
        return false;
    }
    ++recover_ok_;
    return true;
}

bool Sbus::Is_Stale(uint32_t now_ms) const noexcept {
    // 首次上电从未收到帧时，以最近一次 arm 的时刻作为起点；
    // 否则接收机静默时永远不进入恢复。
    const auto frame = snapshot();
    const uint32_t last = frame.count ? frame.last_update_tick : boot_tick_;
    return (now_ms - last) >= kStaleMs;
}

// ===========================================================================
// 位解包
// ===========================================================================

void Sbus::Decode(const uint8_t *frame, uint16_t *out) noexcept {
    if ((frame == nullptr) || (out == nullptr)) {
        return;
    }
    for (uint8_t i = 0U, bit = 0U; i < kMaxChannels; ++i, bit = static_cast<uint8_t>(bit + 11U)) {
        const uint8_t byte = static_cast<uint8_t>(1U + (bit >> 3));   // 数据从 byte1 起
        const uint8_t off  = static_cast<uint8_t>(bit & 7U);
        // 每通道 11 bit，跨 3 字节窗口取。最后一路 byte = 21，byte + 2 = 23
        // （标志字节），高位被 & 0x07FF 掩掉，不会越出 25 字节帧。
        const uint32_t win = static_cast<uint32_t>(frame[byte])
                           | (static_cast<uint32_t>(frame[byte + 1U]) << 8)
                           | (static_cast<uint32_t>(frame[byte + 2U]) << 16);
        out[i] = static_cast<uint16_t>((win >> off) & 0x07FFU);
    }
}

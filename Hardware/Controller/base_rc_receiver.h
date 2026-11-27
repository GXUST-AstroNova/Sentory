#pragma once

#include "RcSnapshot.h"
#include <cstddef>

class RcReceiver {
public:
    virtual ~RcReceiver() = default;
    virtual bool init() noexcept = 0;
    [[nodiscard]] bool online(std::uint32_t timeout_ms = 100) const noexcept;

    // 整帧读取，所有通道及状态来自同一次发布。
    [[nodiscard]] RcSnapshot snapshot() const noexcept;
    // 从 0 开始编号，返回 [-1, 1]。中位死区、首帧前及无效通道返回 0。
    [[nodiscard]] float channel(std::size_t index) const noexcept;

protected:
    static constexpr std::uint8_t kMaxChannels = 16;
    RcReceiver(std::uint16_t lo, std::uint16_t mid, std::uint16_t hi) noexcept;
    void setReady(bool ready) noexcept;
    void markUpdated(const std::uint16_t *channels, std::uint8_t count,
                     bool failsafe, bool frame_lost) noexcept;

private:
    RcSnapshot frame_{};
};

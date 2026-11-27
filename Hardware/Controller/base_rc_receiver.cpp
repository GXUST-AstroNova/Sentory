#include "base_rc_receiver.h"
#include "irq_guard.hpp"
#include <algorithm>

RcSnapshot RcReceiver::snapshot() const noexcept {
    IrqGuard guard;
    return frame_;
}

float RcReceiver::channel(std::size_t index) const noexcept {
    return snapshot().Normalised(index);
}

bool RcReceiver::online(uint32_t timeout_ms) const noexcept {
    IrqGuard guard;
    return frame_.count != 0 && HAL_GetTick() - frame_.last_update_tick < timeout_ms;
}

RcReceiver::RcReceiver(uint16_t lo, uint16_t mid, uint16_t hi) noexcept {
    frame_.raw_min = lo;
    frame_.raw_mid = mid;
    frame_.raw_max = hi;
}

void RcReceiver::setReady(bool ready) noexcept {
    IrqGuard guard;
    frame_.ready = ready;
}

void RcReceiver::markUpdated(const uint16_t *channels, uint8_t count,
                             bool failsafe, bool frame_lost) noexcept {
    if (!channels) return;
    IrqGuard guard;
    frame_.count = std::min(count, kMaxChannels);
    std::copy_n(channels, frame_.count, frame_.channels);
    frame_.failsafe = failsafe;
    frame_.frame_lost = frame_lost;
    frame_.last_update_tick = HAL_GetTick();
    ++frame_.update_count;
}

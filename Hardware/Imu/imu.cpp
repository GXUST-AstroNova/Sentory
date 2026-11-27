#include "imu.hpp"
#include "irq_guard.hpp"

ImuFeedback Imu::feedback() const noexcept {
    IrqGuard guard;
    return feedback_;
}

bool Imu::online(std::uint32_t timeout_ms) const noexcept {
    IrqGuard guard;
    return feedback_.valid && HAL_GetTick() - feedback_.last_update_ms <= timeout_ms;
}

void Imu::publish(ImuFeedback sample) noexcept {
    IrqGuard guard;
    sample.last_update_ms = HAL_GetTick();
    sample.sequence = feedback_.sequence + 1U;
    sample.valid = true;
    feedback_ = sample;
}

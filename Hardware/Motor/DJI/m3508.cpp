#include "m3508.hpp"
#include "irq_guard.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace {
struct Group {
    FDCAN_HandleTypeDef* bus{};
    std::array<M3508*, 8> slots{};
};
std::array<Group, 3> groups{};
constexpr double radians_per_count = 2.0 * std::numbers::pi / 8192;
constexpr int32_t max_current_raw = 16384;
constexpr float raw_per_amp = max_current_raw / 20.0f;

Group* findGroup(FDCAN_HandleTypeDef* bus) noexcept {
    for (auto& group : groups) {
        if (group.bus == bus) return &group;
    }
    return nullptr;
}

int16_t signedWord(uint8_t msb, uint8_t lsb) noexcept {
    return static_cast<int16_t>((msb << 8) | lsb);
}
} // namespace

M3508::M3508(FDCAN_HandleTypeDef* bus, uint8_t id, float torque_constant) noexcept
    : bus_(bus), id_(id), kt_(torque_constant) {}

bool M3508::init() noexcept {
    if (!bus_ || id_ < 1 || id_ > 8 || !std::isfinite(kt_) || kt_ <= 0) return false;
    IrqGuard guard;
    if (registered_) return true;
    auto* group = findGroup(bus_);
    if (!group) group = findGroup(nullptr);
    if (!group || group->slots[id_ - 1]) return false;
    group->bus = bus_;
    group->slots[id_ - 1] = this;
    registered_ = true;
    enabled_ = true;
    return true;
}

M3508::~M3508() {
    IrqGuard guard;
    if (!registered_) return;
    auto* group = findGroup(bus_);
    group->slots[id_ - 1] = nullptr;
    if (std::all_of(group->slots.begin(), group->slots.end(),
                    [](auto* motor) { return motor == nullptr; })) group->bus = nullptr;
}

bool M3508::setTorque(float torque_nm) noexcept {
    if (!std::isfinite(torque_nm)) return false;
    IrqGuard guard;
    if (!registered_) return false;
    const float amps = std::clamp(torque_nm / kt_, -20.0f, 20.0f);
    current_target_ = static_cast<int16_t>(std::lround(amps * raw_per_amp));
    return true;
}

bool M3508::setCurrentRaw(int32_t current_raw) noexcept {
    IrqGuard guard;
    if (!registered_) return false;
    current_target_ = static_cast<int16_t>(std::clamp(current_raw, -max_current_raw, max_current_raw));
    return true;
}

int16_t M3508::pendingCurrentRaw() const noexcept {
    IrqGuard guard;
    return enabled_ ? current_target_ : 0;
}

bool M3508::disable() noexcept {
    IrqGuard guard;
    current_target_ = 0;
    enabled_ = false;
    return registered_;
}

bool M3508::enable() noexcept {
    IrqGuard guard;
    enabled_ = registered_;
    return enabled_;
}

bool M3508::resetAccumulatedAngle() noexcept {
    IrqGuard guard;
    if (!feedback_.valid) return false;
    total_counts_ = 0;
    feedback_.multi_turn_rad = 0;
    return true;
}

M3508Feedback M3508::feedback() const noexcept {
    IrqGuard guard;
    return {feedback_, encoder_counts_, total_counts_, speed_rpm_, current_raw_,
            current_raw_ / raw_per_amp, temperature_c_};
}

void M3508::receive(const motor_can::Payload& data) noexcept {
    const uint16_t raw = (data[0] << 8) | data[1];
    if (raw >= 8192) return;
    if (feedback_.valid) {
        int delta = raw - encoder_counts_;
        if (delta > 4096) delta -= 8192;
        if (delta < -4096) delta += 8192;
        total_counts_ += delta;
    } else {
        total_counts_ = raw;
    }
    encoder_counts_ = raw;
    speed_rpm_ = signedWord(data[2], data[3]);
    current_raw_ = signedWord(data[4], data[5]);
    temperature_c_ = data[6];
    feedback_.single_turn_rad = (raw < 4096 ? raw : raw - 8192) * radians_per_count;
    feedback_.multi_turn_rad = total_counts_ * radians_per_count;
    feedback_.velocity_rad_s = speed_rpm_ * (std::numbers::pi_v<float> / 30);
    feedback_.torque_nm = current_raw_ * (kt_ / raw_per_amp);
    feedback_.last_rx_ms = HAL_GetTick();
    feedback_.valid = true;
}

bool DjiMotorGroup::dispatch(FDCAN_HandleTypeDef* bus, uint16_t feedback_id,
                             const motor_can::Payload& data) noexcept {
    if (!bus || feedback_id < 0x201 || feedback_id > 0x208) return false;
    IrqGuard guard;
    auto* group = findGroup(bus);
    auto* motor = group ? group->slots[feedback_id - 0x201] : nullptr;
    if (!motor) return false;
    motor->receive(data);
    return true;
}

SendResult DjiMotorGroup::sendAll() noexcept {
    SendResult result{};
    for (auto& group : groups) {
        for (unsigned first : {0u, 4u}) {
            IrqGuard guard;
            motor_can::Payload bytes{};
            bool populated = false;
            for (unsigned i = 0; i < 4; ++i) {
                auto* motor = group.slots[first + i];
                if (!motor) continue;
                populated = true;
                const uint16_t raw = motor->enabled_ ? motor->current_target_ : 0;
                bytes[2 * i] = uint8_t(raw >> 8);
                bytes[2 * i + 1] = uint8_t(raw);
            }
            if (populated) {
                ++result.attempted_frames;
                result.queued_frames += motor_can::enqueue(group.bus, first == 0 ? 0x200 : 0x1ff, bytes);
            }
        }
    }
    return result;
}

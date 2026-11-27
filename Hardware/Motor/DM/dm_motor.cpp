#include "dm_motor.hpp"
#include "irq_guard.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace {
constexpr float pi = std::numbers::pi_v<float>;

uint16_t encode(float value, float low, float high, uint16_t top) noexcept {
    const float fraction = (std::clamp(value, low, high) - low) / (high - low);
    return static_cast<uint16_t>(std::lround(fraction * top));
}

float decode(unsigned raw, float limit, unsigned top) noexcept {
    return (2.0f * raw / top - 1.0f) * limit;
}

float wrappedAngle(float value) noexcept {
    float angle = std::remainder(value, 2.0f * pi);
    return angle >= pi ? angle - 2.0f * pi : angle;
}
} // namespace

std::array<DmMotor::Registration, 48> DmMotor::motors_{};

DmMotor::DmMotor(FDCAN_HandleTypeDef* bus, float t_max) noexcept
    : bus_(bus), t_max_(t_max), torque_limit_(t_max) {}

DmMotor::DmMotor(FDCAN_HandleTypeDef* bus, float t_max, DmCanId command_id, DmCanId feedback_id) noexcept
    : bus_(bus), preset_command_id_(command_id.value()), preset_feedback_id_(feedback_id.value()),
      t_max_(t_max), torque_limit_(t_max) {}

bool DmMotor::init(DmCanMode mode) noexcept {
    return preset_command_id_ != no_preset && registerMotor(preset_command_id_, preset_feedback_id_, mode);
}

bool DmMotor::init(DmCanId command_id, DmCanId feedback_id, DmCanMode mode) noexcept {
    if (preset_command_id_ != no_preset &&
        (preset_command_id_ != command_id.value() || preset_feedback_id_ != feedback_id.value())) return false;
    return registerMotor(command_id.value(), feedback_id.value(), mode);
}

bool DmMotor::registerMotor(uint16_t command_id, uint16_t feedback_id, DmCanMode mode) noexcept {
    if (!bus_ || !std::isfinite(t_max_) || t_max_ <= 0) return false;

    const bool fd_brs = mode == DmCanMode::BusDefault ? motor_can::usesFdBrs(bus_)
                                                       : mode == DmCanMode::FdBrs;
    if (fd_brs && !motor_can::usesFdBrs(bus_)) return false;
    IrqGuard guard;
    if (registration_) {
        return registration_->command_id == command_id && registration_->feedback_id == feedback_id &&
               registration_->fd_brs == fd_brs;
    }
    Registration* free_slot = nullptr;
    for (auto& entry : motors_) {
        if (!entry.motor) {
            if (!free_slot) free_slot = &entry;
            continue;
        }
        if (entry.motor->bus_ != bus_) continue;
        // A shared Master ID is valid if the embedded motor ID distinguishes each reply.
        if (entry.command_id == command_id) return false;
        if (entry.feedback_id == feedback_id && (entry.command_id & 15) == (command_id & 15)) return false;
    }
    if (!free_slot) return false;
    cacheTarget({}, torque_limit_);
    *free_slot = {this, command_id, feedback_id, fd_brs};
    registration_ = free_slot;
    return true;
}

DmMotor::~DmMotor() {
    IrqGuard guard;
    if (registration_) *registration_ = {};
}

void DmMotor::setObserveOnly() noexcept {
    IrqGuard guard;
    observe_only_ = true;
    enabled_ = false;
}

bool DmMotor::specialCommand(uint8_t opcode) noexcept {
    if (observe_only_) return false;
    motor_can::Payload bytes{};
    bytes.fill(0xff);
    bytes.back() = opcode;
    return registration_ && motor_can::enqueue(bus_, registration_->command_id, bytes, registration_->fd_brs);
}

bool DmMotor::enable() noexcept {
    IrqGuard guard;
    if (!specialCommand(0xfc)) return false;
    enabled_ = true;
    return true;
}

bool DmMotor::disable() noexcept {
    IrqGuard guard;
    enabled_ = false;
    if (registration_) cacheTarget({}, torque_limit_);
    return specialCommand(0xfd);
}

bool DmMotor::clearFault() noexcept {
    IrqGuard guard;
    return specialCommand(0xfb);
}

bool DmMotor::setTorque(float torque_nm) noexcept {
    return setMit({.torque_ff_nm = torque_nm});
}

bool DmMotor::setMit(const MitTarget& target) noexcept {
    if (!std::isfinite(target.position_rad) || !std::isfinite(target.velocity_rad_s) ||
        !std::isfinite(target.kp) || !std::isfinite(target.kd) ||
        !std::isfinite(target.torque_ff_nm)) return false;
    IrqGuard guard;
    if (!registration_ || observe_only_) return false;
    return cacheTarget(target, torque_limit_);
}

bool DmMotor::setTorqueLimit(float limit_nm) noexcept {
    if (!std::isfinite(limit_nm) || limit_nm < 0 || limit_nm > t_max_) return false;
    IrqGuard guard;
    return registration_ && !observe_only_ && cacheTarget(target_, limit_nm);
}

bool DmMotor::cacheTarget(const MitTarget& target, float limit_nm) noexcept {
    const unsigned p = encode(target.position_rad, -P_MAX, P_MAX, 65535);
    const unsigned v = encode(target.velocity_rad_s, -V_MAX, V_MAX, 4095);
    const unsigned kp = encode(target.kp, 0, 500, 4095);
    const unsigned kd = encode(target.kd, 0, 5, 4095);
    const float ff = std::clamp(target.torque_ff_nm, -limit_nm, limit_nm);
    const unsigned t = encode(ff, -t_max_, t_max_, 4095);
    motor_can::Payload bytes{uint8_t(p >> 8), uint8_t(p), uint8_t(v >> 4), uint8_t((v << 4) | (kp >> 8)),
                             uint8_t(kp), uint8_t(kd >> 4), uint8_t((kd << 4) | (t >> 8)), uint8_t(t)};
    // Seven FF bytes followed by FB..FE are management commands, including encoder zero.
    if (bytes.back() >= 0xfb && bytes.back() <= 0xfe &&
        std::all_of(bytes.begin(), bytes.end() - 1, [](uint8_t b) { return b == 0xff; })) return false;
    target_ = target;
    torque_limit_ = limit_nm;
    payload_ = bytes;
    return true;
}

SendResult DmMotor::sendAll() noexcept {
    SendResult result{};
    for (const auto& entry : motors_) {
        IrqGuard guard;
        auto* motor = entry.motor;
        if (!motor || motor->observe_only_ || !motor->enabled_) continue;
        ++result.attempted_frames;
        result.queued_frames += motor_can::enqueue(motor->bus_, entry.command_id, motor->payload_, entry.fd_brs);
    }
    return result;
}

bool DmMotor::dispatch(FDCAN_HandleTypeDef* bus, uint16_t feedback_id,
                       const motor_can::Payload& bytes, bool fd_brs) noexcept {
    IrqGuard guard;
    for (const auto& entry : motors_) {
        auto* motor = entry.motor;
        if (motor && motor->bus_ == bus && entry.fd_brs == fd_brs && entry.feedback_id == feedback_id &&
            (entry.command_id & 15) == (bytes[0] & 15)) {
            motor->receive(bytes);
            return true;
        }
    }
    return false;
}

void DmMotor::receive(const motor_can::Payload& data) noexcept {
    const float position = decode((data[1] << 8) | data[2], P_MAX, 65535);
    if (feedback_.valid) {
        // Unwrap the native MIT position range before folding any mechanical turn.
        // Successive valid samples must be less than half a mechanical turn apart.
        const float delta = std::remainder(position - last_position_rad_, 2.0f * P_MAX);
        feedback_.multi_turn_rad += wrappedAngle(delta);
    } else {
        feedback_.multi_turn_rad = position;
    }
    last_position_rad_ = position;
    feedback_.single_turn_rad = wrappedAngle(position);
    feedback_.velocity_rad_s = decode((data[3] << 4) | (data[4] >> 4), V_MAX, 4095);
    feedback_.torque_nm = decode(((data[4] & 15) << 8) | data[5], t_max_, 4095);
    feedback_.last_rx_ms = HAL_GetTick();
    feedback_.valid = true;
    if ((data[0] >> 4) >= 8) {
        enabled_ = false;
        cacheTarget({}, torque_limit_);
    }
}

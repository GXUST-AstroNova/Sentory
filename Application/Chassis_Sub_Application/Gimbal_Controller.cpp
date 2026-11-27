#include "Gimbal_Controller.hpp"
#include "OmniChassis.hpp"
#include "RobotMessage.hpp"
#include "dm_motor.hpp"
#include "main.h"

#include <cmath>
#include <numbers>

float Gimbal_Controller::shortestAngleError(float current_rad, float target_rad) noexcept {
    constexpr float pi = std::numbers::pi_v<float>;
    const float error = std::remainder(target_rad - current_rad, 2.0f * pi);
    return error == pi ? -pi : error;
}

void Gimbal_Controller::transitionTo(State next, std::uint32_t now) noexcept {
    if (next == state_) return;
    if (state_ == State::Follow) follow_pid_.Reset();
    state_ = next;
    if (state_ == State::Spin) {
        spin_phase_rad_ = 0.0f;
        last_spin_ms_ = now;
    }
}

float Gimbal_Controller::yawRate(float yaw_error, std::uint32_t now) noexcept {
    switch (state_) {
    case State::Stopped:
    case State::Translate:
        return 0.0f;
    case State::Follow:
        if (std::abs(yaw_error) <= yaw_deadband_rad) {
            follow_pid_.Reset();
            return 0.0f;
        }
        // In the gimbal frame the chassis heading is -yaw_error; alignment is zero.
        return follow_pid_.Calculate(0.0f, -yaw_error);
    case State::Spin: {
        constexpr float turn = 2.0f * std::numbers::pi_v<float>;
        const float dt_s = static_cast<float>(now - last_spin_ms_) * 0.001f;
        last_spin_ms_ = now;
        spin_phase_rad_ = std::fmod(spin_phase_rad_ + turn * spin_frequency_hz * dt_s, turn);
        return spin_base_rad_s + spin_amplitude_rad_s * std::sin(spin_phase_rad_);
    }
    }
    return 0.0f;
}

void Gimbal_Controller::loop(Ctrl_By_Gimbal cmd) {
    const auto yaw = yaw_motor_.feedback();
    const std::uint32_t now = HAL_GetTick();
    State next = static_cast<State>(cmd.mode);
    const bool yaw_available = yaw.valid && now - yaw.last_rx_ms <= yaw_timeout_ms &&
                               std::isfinite(yaw.single_turn_rad);
    float theta = 0.0f, vx = 0.0f, vy = 0.0f;
    if (yaw_available && next >= State::Follow && next <= State::Spin) {
        theta = shortestAngleError(0.0f, encoder_sign * (yaw.single_turn_rad - encoder_zero_rad));
        const float c = std::cos(theta), s = std::sin(theta);
        vx = c * cmd.vx - s * cmd.vy;
        vy = s * cmd.vx + c * cmd.vy;
        if (!std::isfinite(vx) || !std::isfinite(vy)) next = State::Stopped;
    } else {
        next = State::Stopped;
    }

    transitionTo(next, now);
    if (state_ == State::Stopped) vx = vy = 0.0f;
    chassis_.SetTarget(vx, vy, yawRate(theta, now));
}

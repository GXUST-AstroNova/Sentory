#pragma once
#include "pid.hpp"
#include <cstdint>

struct Ctrl_By_Gimbal;
class OmniChassis;
class DmMotor;

class Gimbal_Controller {
public:
    Gimbal_Controller(OmniChassis& chassis, DmMotor& yaw_motor) noexcept
        : chassis_(chassis), yaw_motor_(yaw_motor) {}
    void loop(Ctrl_By_Gimbal cmd);
    // 从当前角到目标角的有符号最短角差，单位 rad，范围 [-π, π)。
    [[nodiscard]] static float shortestAngleError(float current_rad, float target_rad) noexcept;
private:
    enum class State : std::uint8_t { Stopped = 0, Follow = 1, Translate = 2, Spin = 3 };
    void transitionTo(State next, std::uint32_t now) noexcept;
    float yawRate(float yaw_error, std::uint32_t now) noexcept;

    static constexpr float spin_base_rad_s = 8.0f;
    static constexpr float spin_frequency_hz = 2.0f;
    static constexpr float spin_amplitude_rad_s = 1.0f;
    static constexpr float yaw_deadband_rad = 0.035f;
    static constexpr float encoder_sign = 1.0f;
    static constexpr float encoder_zero_rad = 0.0f;
    static constexpr std::uint32_t yaw_timeout_ms = 100;

    State state_{State::Stopped};
    // kp, ki, kd, integral limit, nominal dt (s), yaw rate limit (rad/s).
    PositionalPID follow_pid_{5.0f, 0.0f, 0.0f, 0.0f, 0.001f, 20.0f};
    float spin_phase_rad_{};
    std::uint32_t last_spin_ms_{};
    OmniChassis& chassis_;
    DmMotor& yaw_motor_;
};

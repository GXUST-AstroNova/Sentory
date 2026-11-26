#pragma once

#include "motor.hpp"
#include "pid.hpp"
#include <array>
#include <cstdint>

class OmniChassis {
public:
    // Left front, left rear, right rear, right front. +1 means positive motor
    // rotation drives along the counterclockwise tangent around the chassis.
    OmniChassis(Motor& m1, Motor& m2, Motor& m3, Motor& m4,
                std::array<std::int8_t, 4> motor_signs);
    void Init();
    void Tick(); // Call at 1 kHz, before submitting the motor group commands.
    bool SetTarget(float vx_mps, float vy_mps, float wz_rad_s);
    PositionalPID vx_pid_{1, 0, 0, 10, dt_s_, 100};
    PositionalPID vy_pid_{1, 0, 0, 10, dt_s_, 100};
    PositionalPID wz_pid_{1, 0, 0, 10, dt_s_, 100};
private:
    static constexpr float center_distance_m_ = 0.24328f;
    static constexpr float wheel_radius_m_ = 0.075f;
    static constexpr float gear_ratio_ = 268.0f / 17.0f;
    static constexpr float dt_s_ = 0.001f;
    static constexpr std::uint32_t offline_timeout_ms_ = 500;

    std::array<Motor*, 4> motors_;
    const std::array<std::int8_t, 4> motor_signs_;
    std::array<bool, 4> enabled_{};
    unsigned previous_mask_{};
    std::array<float, 3> target_{};
    
};

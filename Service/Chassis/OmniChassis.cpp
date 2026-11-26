#include "OmniChassis.hpp"
#include "irq_guard.hpp"
#include <bit>
#include <cmath>

namespace {
using Vector = std::array<float, 3>;
using WheelColumns = std::array<Vector, 4>;
constexpr float diagonal = 0.7071067811865475f;
constexpr std::array<float, 4> tangent_x{-diagonal, -diagonal, diagonal, diagonal};
constexpr std::array<float, 4> tangent_y{diagonal, -diagonal, -diagonal, diagonal};

// Columns of the wheel-to-body velocity map. Its transpose allocates wheel
// forces by virtual work. For four wheels the three geometry axes are orthogonal.
constexpr auto makeVelocityMaps(float radius) {
    std::array<WheelColumns, 16> maps{};
    auto& full = maps[15];
    for (unsigned i = 0; i < 4; ++i)
        full[i] = {tangent_x[i] / 2, tangent_y[i] / 2, 1 / (4 * radius)};

    // For rigid-body motion u0 - u1 + u2 - u3 = 0. Substitute the missing
    // wheel's speed into the full map to get each three-wheel inverse.
    constexpr int parity[] = {1, -1, 1, -1};
    for (unsigned missing = 0; missing < 4; ++missing) {
        auto& reduced = maps[15 ^ (1u << missing)];
        for (unsigned i = 0; i < 4; ++i)
            for (unsigned axis = 0; axis < 3; ++axis)
                reduced[i][axis] = full[i][axis] -
                    parity[i] * parity[missing] * full[missing][axis];
    }
    return maps;
}
} // namespace

OmniChassis::OmniChassis(Motor& m1, Motor& m2, Motor& m3, Motor& m4,
                         std::array<std::int8_t, 4> motor_signs)
    : motors_{&m1, &m2, &m3, &m4}, motor_signs_(motor_signs) {}

void OmniChassis::Init() {
    IrqGuard guard;
    target_ = {};
    enabled_ = {};
    previous_mask_ = 0;
    vx_pid_.Reset();
    vy_pid_.Reset();
    wz_pid_.Reset();

    bool signs_valid = true;
    for (auto sign : motor_signs_)
        signs_valid &= (sign == 1 || sign == -1);
    for (unsigned i = 0; i < motors_.size(); ++i) {
        const bool cleared = motors_[i]->setTorque(0);
        enabled_[i] = signs_valid && cleared && motors_[i]->enable();
    }
}

bool OmniChassis::SetTarget(float vx_mps, float vy_mps, float wz_rad_s) {
    if (!std::isfinite(vx_mps) || !std::isfinite(vy_mps) || !std::isfinite(wz_rad_s))
        return false;
    IrqGuard guard;
    target_ = {vx_mps, vy_mps, wz_rad_s};
    return true;
}

void OmniChassis::Tick() {
    static constexpr auto maps = makeVelocityMaps(center_distance_m_);
    constexpr float rotor_to_rim = wheel_radius_m_ / gear_ratio_;
    Vector target;
    std::array<MotorFeedback, 4> feedback;
    std::uint32_t now;
    {
        IrqGuard guard;
        target = target_;
        for (unsigned i = 0; i < motors_.size(); ++i)
            feedback[i] = motors_[i]->feedback();
        now = HAL_GetTick();
    }

    unsigned mask = 0;
    std::array<float, 4> speed{};
    for (unsigned i = 0; i < motors_.size(); ++i) {
        const auto& sample = feedback[i];
        if (enabled_[i] && sample.valid && std::isfinite(sample.velocity_rad_s) &&
            now - sample.last_rx_ms <= offline_timeout_ms_) {
            mask |= 1u << i;
            speed[i] = sample.velocity_rad_s * rotor_to_rim * motor_signs_[i];
        }
    }
    if (mask != previous_mask_) {
        vx_pid_.Reset();
        vy_pid_.Reset();
        wz_pid_.Reset();
        previous_mask_ = mask;
    }

    std::array<float, 4> force{};
    const int online = std::popcount(mask);
    if (online >= 3) {
        const auto& columns = maps[mask];
        Vector observed{};
        for (unsigned i = 0; i < motors_.size(); ++i)
            for (unsigned axis = 0; axis < 3; ++axis)
                observed[axis] += columns[i][axis] * speed[i];
        const Vector wrench{vx_pid_.Calculate(target[0], observed[0]),
                            vy_pid_.Calculate(target[1], observed[1]),
                            wz_pid_.Calculate(target[2], observed[2])};
        for (unsigned i = 0; i < motors_.size(); ++i)
            for (unsigned axis = 0; axis < 3; ++axis)
                force[i] += columns[i][axis] * wrench[axis];
    } else if (online == 2) {
        constexpr float rim_speed_kp = 1.0f; // N per m/s
        for (unsigned i = 0; i < motors_.size(); ++i) {
            if (mask & (1u << i)) {
                const float desired = tangent_x[i] * target[0] + center_distance_m_ * target[2];
                force[i] = rim_speed_kp * (desired - speed[i]);
            }
        }
    }
    for (unsigned i = 0; i < motors_.size(); ++i) {
        const float torque = force[i] * rotor_to_rim * motor_signs_[i];
        motors_[i]->setTorque(std::isfinite(torque) ? torque : 0.0f);
    }
}

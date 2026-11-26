#include "PowerLimit.hpp"
#include "m3508.hpp"
#include "irq_guard.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace power_limit {
namespace {
constexpr float limit_w = 100.0f;
constexpr float margin_w = 5.0f;
constexpr std::uint32_t feedback_timeout_ms = 500;
constexpr double amp_per_raw = 20.0 / 16384;

// M3508 rotor-side fit from DM_02_bsp ChassisProfile. I in A, w in rad/s.
// P = idle(w) + (kI + kIw*w)*I + kII*I^2. Do not credit regenerative power.
constexpr double k0 = 2.1599749, kI = 0.0015595104, kw = -0.00017931848;
constexpr double kIw = 0.016782476, kII = 0.11301039, kww = 1.1531789e-05;

using Wheels = std::array<M3508*, 4>;

PowerLimitResult clearInvalid(const Wheels& wheels) noexcept {
    for (auto* wheel : wheels)
        if (wheel) wheel->setCurrentRaw(0);
    return {};
}

double budgetWatts(const SupercapState& cap, std::uint32_t now) noexcept {
    double watts = limit_w - margin_w;
    if (cap.online && cap.discharge_allowed && now - cap.last_update_ms < cap.valid_for_ms &&
        std::isfinite(cap.extra_power_w) && cap.extra_power_w >= 0)
        watts += cap.extra_power_w;
    return watts;
}
} // namespace

PowerLimitResult Apply(M3508* front_left, M3508* back_left, M3508* back_right,
                       M3508* front_right, const SupercapState& supercap) noexcept {
    const Wheels wheels{front_left, back_left, back_right, front_right};
    for (auto it = wheels.begin(); it != wheels.end(); ++it)
        if (!*it || std::find(wheels.begin(), it, *it) != it)
            return clearInvalid(wheels);

    std::array<MotorFeedback, 4> samples;
    std::array<std::int16_t, 4> raw;
    std::uint32_t now;
    {
        IrqGuard guard;
        for (unsigned i = 0; i < wheels.size(); ++i) {
            samples[i] = wheels[i]->Motor::feedback();
            raw[i] = wheels[i]->pendingCurrentRaw();
        }
        now = HAL_GetTick();
    }

    double quadratic = 0, linear = 0, idle = 0;
    std::array<double, 4> watts_per_amp{};
    for (unsigned i = 0; i < wheels.size(); ++i) {
        const auto& sample = samples[i];
        if (!sample.valid || now - sample.last_rx_ms > feedback_timeout_ms) {
            raw[i] = 0;
            continue;
        }
        const double speed = sample.velocity_rad_s;
        if (!std::isfinite(speed))
            return clearInvalid(wheels);
        const double amps = raw[i] * amp_per_raw;
        watts_per_amp[i] = kI + kIw * speed;
        quadratic += kII * amps * amps;
        linear += std::max(watts_per_amp[i] * amps, 0.0);
        idle += std::max(k0 + kw * speed + kww * speed * speed, 0.0);
    }

    const double budget = budgetWatts(supercap, now);
    const double before = quadratic + linear + idle;
    PowerLimitResult result{0, static_cast<float>(before), 0, true, idle <= budget};
    if (before <= budget) {
        result.scale = 1;
    } else if (idle < budget) {
        // Stable positive root of quadratic*s^2 + linear*s = budget - idle.
        // Here at least one current is nonzero, hence quadratic > 0.
        const double room = budget - idle;
        const double scale = 2 * room / (linear + std::sqrt(linear * linear + 4 * quadratic * room));
        result.scale = std::nextafter(static_cast<float>(scale), 0.0f);
    }

    double after = idle;
    for (unsigned i = 0; i < wheels.size(); ++i) {
        const auto limited = static_cast<std::int16_t>(raw[i] * static_cast<double>(result.scale));
        wheels[i]->setCurrentRaw(limited);
        const double amps = limited * amp_per_raw;
        after += kII * amps * amps + std::max(watts_per_amp[i] * amps, 0.0);
    }
    result.after_w = static_cast<float>(after);
    return result;
}
} // namespace power_limit

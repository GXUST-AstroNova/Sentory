#include "FusionService.hpp"

#include <cmath>

FusionService::FusionService(float sample_rate_hz, FusionRemapAlignment mounting) noexcept
    : mounting_(mounting), configured_(std::isfinite(sample_rate_hz) && sample_rate_hz > 0.0f) {
    if (configured_) {
        FusionAhrsInitialise(&ahrs_);
        auto settings = fusionAhrsDefaultSettings;
        settings.sampleRate = sample_rate_hz;
        FusionAhrsSetSettings(&ahrs_, &settings);
    }
}

bool FusionService::update(const ImuFeedback& sample) noexcept {
    if (!configured_ || !sample.valid) return false;

    const std::uint32_t elapsed_ms = sample.sample_time_ms - last_sample_ms_;
    if (have_sample_) {
        // Forward deltas occupy the lower half of each wrapping counter's range.
        const std::uint32_t seq_delta = sample.sequence - last_sequence_;
        constexpr std::uint32_t half_range = 1u << 31;
        if (seq_delta == 0 || seq_delta >= half_range || elapsed_ms == 0 || elapsed_ms >= half_range)
            return false;
    }

    FusionVector accel{}, gyro{};
    for (unsigned i = 0; i < 3; ++i) {
        accel.array[i] = sample.accel_m_s2[i] / 9.80665f;
        gyro.array[i] = FusionRadiansToDegrees(sample.gyro_rad_s[i]);
        if (!std::isfinite(accel.array[i]) || !std::isfinite(gyro.array[i])) return false;
    }

    last_sequence_ = sample.sequence;
    last_sample_ms_ = sample.sample_time_ms;
    if (!have_sample_ || elapsed_ms > Imu::default_timeout_ms) {
        have_sample_ = true;
        return false;
    }

    FusionAhrsSetSamplePeriod(&ahrs_, static_cast<float>(elapsed_ms) * 0.001f);
    FusionAhrsUpdateNoMagnetometer(&ahrs_, FusionRemap(gyro, mounting_), FusionRemap(accel, mounting_));
    const auto angles = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs_)).angle;
    attitude_ = {FusionDegreesToRadians(angles.roll), FusionDegreesToRadians(angles.pitch),
                 FusionDegreesToRadians(angles.yaw), true};
    return true;
}

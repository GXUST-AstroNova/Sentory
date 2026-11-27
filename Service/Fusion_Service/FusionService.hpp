#pragma once

#include "Fusion.h"
#include "imu.hpp"

class FusionService {
public:
    struct Euler {
        float roll_rad{};
        float pitch_rad{};
        float yaw_rad{}; // Relative heading in the NWU frame.
        bool valid{};
    };

    // Mounting specifies the sensor axis (with sign) for each body axis X, Y, Z.
    explicit FusionService(float sample_rate_hz,
                           FusionRemapAlignment mounting = FusionRemapAlignmentPXPYPZ) noexcept;

    // Feed successive snapshots from one IMU. Returns true when a new attitude is available.
    bool update(const ImuFeedback& sample) noexcept;
    [[nodiscard]] Euler eular() const noexcept { return attitude_; }

private:
    FusionAhrs ahrs_{};
    FusionRemapAlignment mounting_;
    Euler attitude_{};
    std::uint32_t last_sequence_{};
    std::uint32_t last_sample_ms_{};
    bool configured_{};
    bool have_sample_{};
};

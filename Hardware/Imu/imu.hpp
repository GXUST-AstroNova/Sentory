#pragma once

#include <array>
#include <cstdint>

enum class StartResult { Started, Skipped, Error };
enum class ProcessResult { Updated, Pending, Error };

struct ImuFeedback {
    std::array<float, 3> accel_m_s2{};
    std::array<float, 3> gyro_rad_s{};
    float temperature_c{};
    std::uint32_t last_update_ms{};
    std::uint32_t sequence{};
    bool valid{};
    std::uint32_t sample_time_ms{}; // HAL tick at acquisition start, before DMA and publication.
};

class Imu {
public:
    virtual ~Imu() = default;
    virtual bool init() = 0;
    virtual StartResult start() = 0;
    virtual ProcessResult process() = 0;

    [[nodiscard]] ImuFeedback feedback() const noexcept;
    static constexpr std::uint32_t default_timeout_ms = 100;
    [[nodiscard]] bool online(std::uint32_t timeout_ms = default_timeout_ms) const noexcept;

protected:
    void publish(ImuFeedback sample) noexcept;

private:
    ImuFeedback feedback_{};
};

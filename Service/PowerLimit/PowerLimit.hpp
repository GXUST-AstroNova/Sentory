#pragma once
#include <cstdint>

class M3508;

namespace power_limit {
struct SupercapState {
    bool online{};
    bool discharge_allowed{};
    float extra_power_w{};
    std::uint32_t last_update_ms{};
    std::uint32_t valid_for_ms{};
};

struct PowerLimitResult {
    float scale{};
    float before_w{};
    float after_w{}; // Predicted upper bound using the quantized CAN commands.
    bool valid{};
    bool feasible{};
};

// Run once after the chassis writes all four targets and before CAN submission.
[[nodiscard]] PowerLimitResult Apply(M3508* front_left, M3508* back_left,
                                     M3508* back_right, M3508* front_right,
                                     const SupercapState& supercap) noexcept;
}

#ifndef DM_02_BSP_RC_SNAPSHOT_H
#define DM_02_BSP_RC_SNAPSHOT_H

#include <cstdint>
#include <cstddef>

/** 单帧遥控快照，无 HAL 依赖，通道号为 0 基。 */
struct RcSnapshot {
    std::uint16_t channels[16]{};
    std::uint16_t raw_min = 0;
    std::uint16_t raw_mid = 0;
    std::uint16_t raw_max = 0;
    std::uint32_t last_update_tick = 0;
    std::uint32_t update_count = 0;
    std::uint8_t count = 0;
    bool ready = false;
    bool failsafe = false;
    bool frame_lost = false;

    /** 归一化到 -1~+1，中位 0，带死区；通道非法返回 0。 */
    float Normalised(std::size_t ch, float deadband = 0.01f) const noexcept {
        if (ch >= count || ch >= 16U || !(raw_min < raw_mid && raw_mid < raw_max)) {
            return 0.0f;
        }
        const float span = channels[ch] >= raw_mid
                               ? static_cast<float>(raw_max - raw_mid)
                               : static_cast<float>(raw_mid - raw_min);
        if (span <= 0.0f) {
            return 0.0f;
        }
        float value = (static_cast<float>(channels[ch]) - static_cast<float>(raw_mid)) / span;
        if (value > 1.0f) {
            value = 1.0f;
        }
        if (value < -1.0f) {
            value = -1.0f;
        }
        return (value >= -deadband && value <= deadband) ? 0.0f : value;
    }
};

#endif  // DM_02_BSP_RC_SNAPSHOT_H

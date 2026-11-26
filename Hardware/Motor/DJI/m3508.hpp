#pragma once

#include "motor_can.hpp"

class DjiMotorGroup {
public:
    static SendResult sendAll() noexcept;
    static bool dispatch(FDCAN_HandleTypeDef* bus, uint16_t feedback_id,
                         const motor_can::Payload& data) noexcept;
};

// C620 同一反馈帧的快照。角度、编码器和转速均为转子轴数据。
struct M3508Feedback : MotorFeedback {
    uint16_t encoder_counts{};       // 单圈编码器，0..8191
    int64_t accumulated_counts{};    // 与 multi_turn_rad 一起清零
    int16_t speed_rpm{};
    int16_t current_raw{};           // 反馈电流，16384 对应 20 A
    float current_a{};
    uint8_t temperature_c{};
};

class M3508 final : public Motor {
public:
    // Rotor-side N·m/A, inferred from the P19 output specification and reduction.
    static constexpr float rotor_kt = 0.3f / (3591.0f / 187.0f);
    M3508(FDCAN_HandleTypeDef* bus, uint8_t id, float torque_constant = rotor_kt) noexcept;
    ~M3508() override;
    [[nodiscard]] bool init() noexcept;   // ID 1..8; joins the group at zero current
    bool setTorque(float torque_nm) noexcept override;
    // 写入 C620 原始电流目标，越界限幅到 ±16384（±20 A），由 sendAll() 发送。
    bool setCurrentRaw(int32_t current_raw) noexcept;
    [[nodiscard]] int16_t pendingCurrentRaw() const noexcept;
    bool disable() noexcept override;
    bool enable() noexcept override;
    bool resetAccumulatedAngle() noexcept;
    [[nodiscard]] M3508Feedback feedback() const noexcept;
private:
    friend class DjiMotorGroup;
    void receive(const motor_can::Payload& data) noexcept;
    FDCAN_HandleTypeDef* const bus_;
    const uint8_t id_;
    const float kt_;
    int64_t total_counts_{};
    uint16_t encoder_counts_{};
    int16_t speed_rpm_{};
    int16_t current_raw_{};
    uint8_t temperature_c_{};
    int16_t current_target_{};
    bool registered_{};
    bool enabled_{};
};

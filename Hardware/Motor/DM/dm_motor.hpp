#pragma once

#include "motor_can.hpp"
#include "dm_address.hpp"
#include <limits>

struct MitTarget {
    float position_rad{};
    float velocity_rad_s{};
    float kp{};
    float kd{};
    float torque_ff_nm{};
};

enum class DmCanMode { BusDefault, Classic, FdBrs };

// Register both 11-bit CAN identifiers before enable() and cyclic sendAll().
class DmMotor : public Motor {
public:
    static constexpr float P_MAX = 12.5664f;
    static constexpr float V_MAX = 45.0f;
    ~DmMotor() override;

    [[nodiscard]] bool init(DmCanId command_id, DmCanId feedback_id,
                            DmCanMode mode = DmCanMode::BusDefault) noexcept;
    // Convenience for models constructed with both IDs.
    [[nodiscard]] bool init(DmCanMode mode = DmCanMode::BusDefault) noexcept;
    // Permanently suppress new TX frames from this instance; feedback remains active.
    void setObserveOnly() noexcept;
    bool enable() noexcept override;
    bool disable() noexcept override;
    bool clearFault() noexcept;
    bool setTorque(float torque_nm) noexcept override;
    bool setMit(const MitTarget& target) noexcept;
    bool setTorqueLimit(float limit_nm) noexcept;  // clamps only the feed-forward command

    static SendResult sendAll() noexcept;
    static bool dispatch(FDCAN_HandleTypeDef* bus, uint16_t feedback_id,
                         const motor_can::Payload& data, bool fd_brs) noexcept;

protected:
    DmMotor(FDCAN_HandleTypeDef* bus, float t_max) noexcept;
    DmMotor(FDCAN_HandleTypeDef* bus, float t_max, DmCanId command_id, DmCanId feedback_id) noexcept;

    template<float TorqueMax>
    static consteval float checkedTorqueMax() {
        static_assert(TorqueMax > 0 && TorqueMax <= std::numeric_limits<float>::max(),
                      "Set a finite, positive T_MAX for this DM motor model");
        return TorqueMax;
    }

private:
    struct Registration {
        DmMotor* motor{};
        uint16_t command_id{};
        uint16_t feedback_id{};
        bool fd_brs{};
    };
    static std::array<Registration, 48> motors_;
    Registration* registration_{};
    bool registerMotor(uint16_t command_id, uint16_t feedback_id, DmCanMode mode) noexcept;
    bool specialCommand(uint8_t opcode) noexcept;
    bool cacheTarget(const MitTarget& target, float limit_nm) noexcept;
    void receive(const motor_can::Payload& data) noexcept;

    FDCAN_HandleTypeDef* const bus_;
    static constexpr uint16_t no_preset = 0xffff;
    const uint16_t preset_command_id_ = no_preset;
    const uint16_t preset_feedback_id_ = no_preset;
    const float t_max_;
    float torque_limit_;
    MitTarget target_{};
    motor_can::Payload payload_{};
    float last_position_rad_{};
    bool enabled_{};
    bool observe_only_{};
};


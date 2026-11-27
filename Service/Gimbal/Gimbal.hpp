#pragma once

#include "pid.hpp"
#include "FusionService.hpp"
#include "imu.hpp"
class DmMotor;

#include "irq_guard.hpp"

// 电机由应用层持有，生命周期须长于云台对象。
class Gimbal {
public:
    virtual ~Gimbal() = default;
    Gimbal(const Gimbal&) = delete;
    Gimbal& operator=(const Gimbal&) = delete;

    virtual void Init() = 0;
    virtual void Tick() = 0;

    enum class Mode { Idle, Angle, Speed };
    struct AxisTarget {
        Mode mode = Mode::Idle;
        float angle_rad{};
        float speed_rad_s{};
        float ff_speed_rad_s{};
        float ff_accel_rad_s2{};
    };
    struct Targets {
        AxisTarget pitch;
        AxisTarget yaw;
    };
    void SetYawAngle(float angle_rad);
    void SetPitchAngle(float angle_rad);
    void SetYawSpeed(float speed_rad_s);
    void SetPitchSpeed(float speed_rad_s);
    #ifdef Gimbal_Pid_Debug
    //用于阶跃调试使用，常态不启用
    void SetYawMode(Mode mode) {
        IrqGuard guard;
        targets_.yaw.mode = mode;
    }
    void SetPitchMode(Mode mode) {
        IrqGuard guard;
        targets_.pitch.mode = mode;
    }
    #endif
    void Yaw_FF(float speed_rad_s, float accel_rad_s2);
    void Pitch_FF(float speed_rad_s, float accel_rad_s2);
    [[nodiscard]] Targets GetTargets() const;

protected:
    Gimbal(DmMotor& pitch_motor, DmMotor& yaw_motor, Imu& imu) noexcept;
    static constexpr auto mounting = FusionRemapAlignmentPYNXPZ;
    Imu& imu_;
    FusionService fusion{1000.0f, mounting};
    DmMotor& pitch_motor_;
    DmMotor& yaw_motor_;

private:
    Targets targets_{};
};

class PID_Gimbal final : public Gimbal {
public:
    PID_Gimbal(DmMotor& pitch_motor, DmMotor& yaw_motor, Imu& imu) noexcept
        : Gimbal(pitch_motor, yaw_motor, imu) {}

    void Init() override;
    void Tick() override;
    PositionalPID yaw_angle_pid_{0.1f, 0, 0, 3, dt_s_};
    PositionalPID pitch_angle_pid_{0.1f, 0, 0, 3, dt_s_};
    PositionalPID yaw_speed_pid_{0.1f, 0, 0, 3, dt_s_, 7};
    PositionalPID pitch_speed_pid_{0.1f, 0, 0, 3, dt_s_, 7};
private:
    static constexpr float dt_s_ = 0.001f;
};

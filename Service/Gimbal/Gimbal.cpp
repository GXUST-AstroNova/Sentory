#include "Gimbal.hpp"
#include "dm_motor.hpp"
#include "irq_guard.hpp"

Gimbal::Gimbal(DmMotor& pitch_motor, DmMotor& yaw_motor, Imu& imu) noexcept
    : imu_(imu), pitch_motor_(pitch_motor), yaw_motor_(yaw_motor) {}

void Gimbal::SetYawAngle(float angle_rad) {
    IrqGuard guard;
    targets_.yaw.mode = Mode::Angle;
    targets_.yaw.angle_rad = angle_rad;
}

void Gimbal::SetPitchAngle(float angle_rad) {
    IrqGuard guard;
    targets_.pitch.mode = Mode::Angle;
    targets_.pitch.angle_rad = angle_rad;
}

void Gimbal::SetYawSpeed(float speed_rad_s) {
    IrqGuard guard;
    targets_.yaw.mode = Mode::Speed;
    targets_.yaw.speed_rad_s = speed_rad_s;
}

void Gimbal::SetPitchSpeed(float speed_rad_s) {
    IrqGuard guard;
    targets_.pitch.mode = Mode::Speed;
    targets_.pitch.speed_rad_s = speed_rad_s;
}

void Gimbal::Yaw_FF(float speed_rad_s, float accel_rad_s2) {
    IrqGuard guard;
    targets_.yaw.ff_speed_rad_s = speed_rad_s;
    targets_.yaw.ff_accel_rad_s2 = accel_rad_s2;
}

void Gimbal::Pitch_FF(float speed_rad_s, float accel_rad_s2) {
    IrqGuard guard;
    targets_.pitch.ff_speed_rad_s = speed_rad_s;
    targets_.pitch.ff_accel_rad_s2 = accel_rad_s2;
}

Gimbal::Targets Gimbal::GetTargets() const {
    IrqGuard guard;
    return targets_;
}

void PID_Gimbal::Init() {
    IrqGuard guard;
    yaw_angle_pid_.Reset();
    pitch_angle_pid_.Reset();
    yaw_speed_pid_.Reset();
    pitch_speed_pid_.Reset();
}

void PID_Gimbal::Tick() {
    //loop中完成imu采样，在tick开始时算出当前姿态
    auto imu_feedback = imu_.feedback();
    const auto& rates = imu_feedback.gyro_rad_s;
    const auto gyro = FusionRemap(FusionVector{{rates[0], rates[1], rates[2]}}, mounting).axis;
    const auto targets = GetTargets();
    fusion.update(imu_feedback);
    auto eular = fusion.eular();
    switch (targets.yaw.mode) {
        case Mode::Idle:
            yaw_motor_.setTorque(0.0f);
            break;
        case Mode::Angle: {
            auto pid_output = yaw_angle_pid_.Calculate(targets.yaw.angle_rad, eular.yaw_rad);
            yaw_motor_.setTorque(yaw_speed_pid_.Calculate(pid_output, gyro.z));
            break;
        }
        case Mode::Speed:
            yaw_motor_.setTorque(yaw_speed_pid_.Calculate(targets.yaw.speed_rad_s, gyro.z));
            break;
    }
    switch (targets.pitch.mode) {
        case Mode::Idle:
            pitch_motor_.setTorque(0.0f);
            break;
        case Mode::Angle: {
            auto pid_output = pitch_angle_pid_.Calculate(targets.pitch.angle_rad, eular.pitch_rad);
            pitch_motor_.setTorque(pitch_speed_pid_.Calculate(pid_output, gyro.y));
            break;
        }
        case Mode::Speed:
            pitch_motor_.setTorque(pitch_speed_pid_.Calculate(targets.pitch.speed_rad_s, gyro.y));
            break;
    }
}
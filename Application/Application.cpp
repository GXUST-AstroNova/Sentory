#include "Application.h"
#include "fdcan.h"
#include "hfdcan_bsp.h"
#include "usart.h"
#include "vofa.h"
#include "dm_models.hpp"
#include "m3508.hpp"
#include "sbus.h"
#include "OmniChassis.hpp"
#include "PowerLimit/PowerLimit.hpp"
namespace Application {
    vofa debug(&huart1);
    Sbus receiver(&huart5);
    M3508 front_left_motor(&hfdcan1, 1);
    M3508 front_right_motor(&hfdcan1, 2);
    M3508 back_left_motor(&hfdcan1, 4);
    M3508 back_right_motor(&hfdcan1, 3);
    OmniChassis chassis(front_left_motor, back_left_motor, back_right_motor, front_right_motor, {1, 1, 1, 1});
}
namespace {
    // SBUS channel numbers on the transmitter are one-based.
    constexpr uint8_t forward_channel = 2 - 1;
    constexpr uint8_t lateral_channel = 4 - 1;
    constexpr uint8_t yaw_channel = 1 - 1;
    constexpr float max_linear_mps = 5.0f;
    constexpr float max_yaw_rad_s = 5.0f;
    constexpr uint32_t receiver_timeout_ms = 100;

    void updateChassisTarget() {
        RcSnapshot frame;
        Application::receiver.Get_Snapshot(frame);
        if (!frame.ready || frame.update_count == 0 || frame.failsafe || frame.frame_lost ||
            frame.count <= lateral_channel || HAL_GetTick() - frame.last_update_tick >= receiver_timeout_ms) {
            Application::chassis.SetTarget(0, 0, 0);
            return;
        }

        Application::chassis.SetTarget( frame.Normalised(forward_channel) * max_linear_mps,
                                      -frame.Normalised(lateral_channel) * max_linear_mps,
                                      -frame.Normalised(yaw_channel) * max_yaw_rad_s);
    }
}

void Application_Init() {
    using namespace Application;
    if (!can_bus::start(&hfdcan1) || !can_bus::start(&hfdcan2) || !can_bus::start(&hfdcan3)) {
        Error_Handler();
        return;
    }
    if (!front_left_motor.init() || !back_left_motor.init() ||
        !back_right_motor.init() || !front_right_motor.init() || receiver.Init() != 0) {
        Error_Handler();
        return;
    }
    chassis.Init();
}

void Application_Loop() {}

void Application_Timer_Callback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM7) {
        updateChassisTarget();
        Application::chassis.Tick();
        (void)power_limit::Apply(&Application::front_left_motor, &Application::back_left_motor,
                                 &Application::back_right_motor, &Application::front_right_motor, {});
        DmMotor::sendAll();
        DjiMotorGroup::sendAll();
        Application::debug.send();
    }
}

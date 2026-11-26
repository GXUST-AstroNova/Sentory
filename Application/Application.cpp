#include "Application.h"
#include "RoleApps.hpp"
#include "BoardConfig/BootConfig.hpp"
#include "tim.h"
#include "ws2812.h"

using board_config::Role;
using board_config::Status;

// Startup diagnostics, readable by symbol from a debugger.
extern "C" {
volatile std::uint8_t g_board_role = static_cast<std::uint8_t>(Role::Unknown);
volatile std::int32_t g_board_config_status = static_cast<std::int32_t>(Status::NotConfigured);
volatile std::uint32_t g_board_config_sequence = 0;
}

namespace {
Role active_role = Role::Unknown;
bool boot_attempted = false;
Ws2812 led(&hspi6, Ws2812::Config{});

void updateLed() {
    if (!led.Is_Ready()) return;
    const auto now = HAL_GetTick();
    LedColor desired{};
    switch (active_role) {
    case Role::Chassis: desired.g = 255; break;
    case Role::Gimbal: desired.b = 255; break;
    case Role::Unknown: desired.r = (now / 250U) % 2U == 0 ? 255 : 0; break;
    }
    static LedColor submitted{}; // Init submits black.
    if (desired.r == submitted.r && desired.g == submitted.g && desired.b == submitted.b)
        return;
    led.Set_Color(desired);
    if (led.Show(now)) submitted = desired;
}
}

void Application_Init() {
    if (boot_attempted) return;
    boot_attempted = true;
    const auto config = board_config::LoadBootConfig();
    g_board_config_sequence = config.sequence;
    g_board_role = static_cast<std::uint8_t>(config.role);
    g_board_config_status = static_cast<std::int32_t>(config.status);
    (void)led.Init();

    switch (config.role) {
    case Role::Chassis:
        if (!ChassisApp::Init()) {
            Error_Handler();
            return;
        }
        break;
    case Role::Gimbal:
        if (!GimbalApp::Init()) {
            Error_Handler();
            return;
        }
        break;
    case Role::Unknown:
        updateLed();
        return;
    }

    active_role = config.role;
    updateLed();
    __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
    if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK) Error_Handler();
}

void Application_Loop() {
    updateLed();
    switch (active_role) {
    case Role::Chassis: ChassisApp::Loop(); break;
    case Role::Gimbal: GimbalApp::Loop(); break;
    case Role::Unknown: break;
    }
}

void Application_Timer_Callback(TIM_HandleTypeDef* timer) {
    if (timer->Instance != TIM7) return;
    switch (active_role) {
    case Role::Chassis: ChassisApp::Tick(); break;
    case Role::Gimbal: GimbalApp::Tick(); break;
    case Role::Unknown: break;
    }
}

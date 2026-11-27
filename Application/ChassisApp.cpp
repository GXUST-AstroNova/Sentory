#include "RoleApps.hpp"
#include "fdcan.h"
#include "hfdcan_bsp.h"
#include "usart.h"
#include "vofa.h"
#include "dm_models.hpp"
#include "m3508.hpp"
#include "sbus.h"
#include "OmniChassis.hpp"
#include "Gimbal_Controller.hpp"
#include "PowerLimit/PowerLimit.hpp"
#include "RobotMessage.hpp"
#include "btb_service.hpp"
namespace {
struct Devices {
    vofa debug{&huart1};
    M3508 front_left_motor{&hfdcan1, 1};
    M3508 front_right_motor{&hfdcan1, 2};
    M3508 back_left_motor{&hfdcan1, 4};
    M3508 back_right_motor{&hfdcan1, 3};
    Dm4310 yaw_motor{&hfdcan2, 0x10, 0x20};
    OmniChassis chassis{front_left_motor, back_left_motor, back_right_motor, front_right_motor, {1, 1, 1, 1}};
    Gimbal_Controller controller{chassis, yaw_motor};
    GimbalCommandMessage gimbal_cmd{GimbalCommandCanId, {}};
    BtbService gimbal_service{hfdcan2, gimbal_cmd};
};

Devices& devices() {
    static Devices instance;
    return instance;
}
}

bool ChassisApp::Init() {
    auto& d = devices();
    if (!can_bus::start(&hfdcan1) || !can_bus::start(&hfdcan2) || !can_bus::start(&hfdcan3)) {
        return false;
    }
    if (!d.front_left_motor.init() || !d.back_left_motor.init() ||
        !d.back_right_motor.init() || !d.front_right_motor.init()) {
        return false;
    }
    d.yaw_motor.setObserveOnly();
    if (!d.yaw_motor.init()) return false;
    d.chassis.Init();
    return d.gimbal_service.init();
}

void ChassisApp::Loop() {
    auto& d = devices();
    constexpr std::uint32_t command_timeout_ms = 100;
    d.controller.loop(d.gimbal_service.online(command_timeout_ms) ? d.gimbal_cmd.value() : Ctrl_By_Gimbal{});
}

void ChassisApp::Tick() {
    auto& d = devices();
    d.chassis.Tick();
    (void)power_limit::Apply(&d.front_left_motor, &d.back_left_motor,
                             &d.back_right_motor, &d.front_right_motor, {});
    DmMotor::sendAll();
    DjiMotorGroup::sendAll();
    d.debug.send();
}

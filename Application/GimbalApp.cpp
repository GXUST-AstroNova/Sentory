#include "RoleApps.hpp"
#include "fdcan.h"
#include "hfdcan_bsp.h"
#include "spi.h"
#include "usart.h"
#include "vofa.h"
#include "sbus.h"
#include "RobotMessage.hpp"
#include "btb_message.hpp"
#include "btb_service.hpp"
#include "dm_models.hpp"
#include "m3508.hpp"

namespace {


struct Devices {
    vofa debug{&huart1};
    Sbus receiver{&huart5};
    GimbalCommandMessage gimbal_cmd{GimbalCommandCanId, {}};
    BtbService gimbal_service{hfdcan2, gimbal_cmd};
};

Devices& devices() {
    static Devices instance;
    return instance;
}
}

bool GimbalApp::Init() {
    if (!can_bus::start(&hfdcan1) || !can_bus::start(&hfdcan2) || !can_bus::start(&hfdcan3)) {
        return false;
    }
    return devices().gimbal_service.init();
}

void GimbalApp::Loop() {
    
}

void GimbalApp::Tick() {
    auto& d = devices();
    DmMotor::sendAll();
    DjiMotorGroup::sendAll();
    d.gimbal_service.send();
}
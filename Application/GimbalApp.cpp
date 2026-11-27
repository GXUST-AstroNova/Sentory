#include "RoleApps.hpp"
#include "fdcan.h"
#include "hfdcan_bsp.h"
#include "spi.h"
#include "usart.h"
#include "vofa.h"
#include "sbus.h"
#include "main.h"
#include "bmi088.h"
#include "RobotMessage.hpp"
#include "btb_message.hpp"
#include "btb_service.hpp"
#include "dm_models.hpp"
#include "m3508.hpp"
#include "Gimbal.hpp"
#include "GimbalDebug.hpp"
namespace {


struct Devices {
    Bmi088 imu{&hspi2};
    vofa debug{&huart1};
    Sbus receiver{&huart5};
    Dm4310 pitch_motor{&hfdcan1,0x15,0x25};
    Dm4310 yaw_motor{&hfdcan2,0x10,0x20};
    GimbalCommandMessage gimbal_cmd{GimbalCommandCanId, {}};
    BtbService gimbal_service{hfdcan2, gimbal_cmd};
    PID_Gimbal gimbal{pitch_motor, yaw_motor, imu};
#ifdef Gimbal_Pid_Debug
    Gimbal_Debugger step{gimbal, Gimbal_Debugger::Mode::Pitch_Speed, 9, 1.0f};
#endif
};

Devices& devices() {
    static Devices instance;
    return instance;
}
}

bool GimbalApp::Init() {
    auto& d = devices();
    if(!d.receiver.init()) {
        return false;
    }
    if (!can_bus::start(&hfdcan1) || !can_bus::start(&hfdcan2) || !can_bus::start(&hfdcan3)) {
        return false;
    }
    if(!d.imu.init()) {
        return false;
    }
    d.gimbal.Init();
    return d.gimbal_service.init();
}

void GimbalApp::Loop() {
    auto& d = devices();
    if(d.imu.online()) d.imu.process();
#ifdef Gimbal_Pid_Debug
    d.step.loop(d.receiver.snapshot());
    Ctrl_By_Gimbal c={0,0,2,0};
    d.gimbal_cmd.set(c);
#endif
}

void GimbalApp::Tick() {
    auto& d = devices();
    d.imu.start();
    d.gimbal.Tick();
    DmMotor::sendAll();
    DjiMotorGroup::sendAll();
    d.gimbal_service.send();
}
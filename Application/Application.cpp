#include "Application.h"
#include "fdcan.h"
#include "hfdcan_bsp.h"
#include "vofa.h"
#include "dm_models.hpp"
#include "m3508.hpp"
#include "pid.hpp"
namespace Application {
    vofa debug(&huart1);
}

void Application_Init() {
    //can总线初始化
    can_bus::start(&hfdcan1);
    can_bus::start(&hfdcan2);
    can_bus::start(&hfdcan3);
}

void Application_Loop() {

}

void Application_Timer_Callback(TIM_HandleTypeDef *htim) {
    (void)htim;
    if(htim->Instance == TIM7) {
        Application::debug.send();
        DmMotor::sendAll();
        DjiMotorGroup::sendAll();
    }
}

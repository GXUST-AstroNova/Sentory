#pragma once

#include "RcSnapshot.h"
#include "Gimbal.hpp"

//云台阶跃控制器
class Gimbal_Debugger{
public:
    enum class Mode{
        Yaw_Angle,
        Yaw_Speed,
        Pitch_Angle,
        Pitch_Speed
    };
#ifdef Gimbal_Pid_Debug
    Gimbal_Debugger(Gimbal& gimbal, Mode mode,uint8_t step_channel,float target) : gimbal_(gimbal), mode_(mode), step_channel_(step_channel), targ_(target) {}
#else
    Gimbal_Debugger(Gimbal&, Mode, uint8_t, float) = delete;
    Gimbal_Debugger(const Gimbal_Debugger&) = delete;
#endif
    void loop(const RcSnapshot& snapshot);
private:
    Gimbal& gimbal_;
    Mode mode_;
    uint8_t step_channel_;
    float targ_;
};

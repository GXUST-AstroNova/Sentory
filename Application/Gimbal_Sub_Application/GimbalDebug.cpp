#include "GimbalDebug.hpp"
#include "Gimbal.hpp"
#ifdef Gimbal_Pid_Debug
void Gimbal_Debugger::loop(const RcSnapshot& snapshot){
    if (step_channel_ < snapshot.count && step_channel_ < 16 && snapshot.channels[step_channel_] > 1400){
        switch(mode_){
            case Mode::Yaw_Angle:
                gimbal_.SetYawMode(Gimbal::Mode::Angle);
                gimbal_.SetPitchMode(Gimbal::Mode::Idle);
                gimbal_.SetYawAngle(targ_);
                break;
            case Mode::Yaw_Speed:
                gimbal_.SetYawMode(Gimbal::Mode::Speed);
                gimbal_.SetPitchMode(Gimbal::Mode::Idle);
                gimbal_.SetYawSpeed(targ_);
                break;
            case Mode::Pitch_Angle:
                gimbal_.SetYawMode(Gimbal::Mode::Idle);
                gimbal_.SetPitchMode(Gimbal::Mode::Angle);
                gimbal_.SetPitchAngle(targ_);
                break;
            case Mode::Pitch_Speed:
                gimbal_.SetYawMode(Gimbal::Mode::Idle);
                gimbal_.SetPitchMode(Gimbal::Mode::Speed);
                gimbal_.SetPitchSpeed(targ_);
                break;
        }
    }else{
        switch(mode_){
            case Mode::Yaw_Angle:
                gimbal_.SetYawMode(Gimbal::Mode::Angle);
                gimbal_.SetPitchMode(Gimbal::Mode::Idle);
                gimbal_.SetYawAngle(0.0f);
                break;
            case Mode::Yaw_Speed:
                gimbal_.SetYawMode(Gimbal::Mode::Speed);
                gimbal_.SetPitchMode(Gimbal::Mode::Idle);
                gimbal_.SetYawSpeed(0.0f);
                break;
            case Mode::Pitch_Angle:
                gimbal_.SetYawMode(Gimbal::Mode::Idle);
                gimbal_.SetPitchMode(Gimbal::Mode::Angle);
                gimbal_.SetPitchAngle(0.0f);
                break;
            case Mode::Pitch_Speed:
                gimbal_.SetYawMode(Gimbal::Mode::Idle);
                gimbal_.SetPitchMode(Gimbal::Mode::Speed);
                gimbal_.SetPitchSpeed(0.0f);
                break;
        }
    }
}
#endif
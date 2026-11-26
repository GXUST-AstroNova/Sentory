#include "btb_message.hpp"
#include <cstdint>

//云台下发的控制命令
struct Ctrl_By_Gimbal{
    float vx;
    float vy;
    uint8_t mode;
    uint8_t toggle;
};
using GimbalCommandMessage = BtbMessage<Ctrl_By_Gimbal>;
uint16_t constexpr GimbalCommandCanId = 0x201;
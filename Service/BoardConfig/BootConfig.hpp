#pragma once
#include "BoardConfigDefs.h"

namespace board_config {
struct BootConfig {
    Status status = Status::NotConfigured;
    Role role = Role::Unknown;
    std::uint32_t sequence = 0;
};

BootConfig LoadBootConfig();
}

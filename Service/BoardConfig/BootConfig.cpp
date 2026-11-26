#include "BootConfig.hpp"
#include "BoardConfigCodec.h"
#include "octospi.h"
#include "w25q.h"
#include <algorithm>

namespace board_config {
namespace {
BootConfig readSlot(W25Qxx& flash, std::uint32_t address) {
    Record record{};
    auto* bytes = reinterpret_cast<std::uint8_t*>(&record);
    if (flash.Read(bytes, address, sizeof(record)) != W25Qxx::Status::Ok)
        return {Status::IoError};
    if (std::all_of(bytes, bytes + sizeof(record), [](auto byte) { return byte == 0xff; }))
        return {Status::NotConfigured};
    const auto status = Validate(record);
    if (status != Status::Ok) return {status};
    return {status, Role_From_U8(record.header.role), record.header.sequence};
}
}

BootConfig LoadBootConfig() {
    W25Qxx flash(&hospi2);
    for (unsigned attempt = 0; flash.Init() != W25Qxx::Status::Ok; ++attempt) {
        if (attempt == 2) return {Status::IoError};
        HAL_Delay(10);
    }

    const auto a = readSlot(flash, kSlotAddress[0]);
    const auto b = readSlot(flash, kSlotAddress[1]);
    if (a.status == Status::Ok && b.status == Status::Ok)
        return static_cast<std::int32_t>(b.sequence - a.sequence) > 0 ? b : a;
    if (b.status == Status::Ok || a.status == Status::NotConfigured) return b;
    return a;
}
}

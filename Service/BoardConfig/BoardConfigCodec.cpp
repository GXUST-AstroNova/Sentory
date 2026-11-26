//
// 板级固化配置（W25Q64）：纯逻辑编解码实现
//

#include "BoardConfigCodec.h"

#include <cstddef>
#include <cstring>

namespace board_config {
namespace {

constexpr std::uint32_t kCrcPolynomial = 0xEDB88320U;
constexpr std::uint32_t kCrcInit       = 0xFFFFFFFFU;
constexpr std::uint32_t kCrcXorOut     = 0xFFFFFFFFU;

/** CRC 覆盖范围：[0, offsetof(Record, crc32)) */
constexpr std::uint32_t kCrcCovered =
    static_cast<std::uint32_t>(offsetof(Record, crc32));

}  // namespace

std::uint32_t Crc32(const void *data, std::uint32_t len) noexcept {
    if (data == nullptr || len == 0U) {
        return 0U;
    }
    const auto *bytes = static_cast<const std::uint8_t *>(data);
    std::uint32_t crc = kCrcInit;
    for (std::uint32_t i = 0U; i < len; ++i) {
        crc ^= bytes[i];
        for (std::uint32_t bit = 0U; bit < 8U; ++bit) {
            const std::uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1U) ^ (kCrcPolynomial & mask);
        }
    }
    return crc ^ kCrcXorOut;
}

void Seal(Record *record) noexcept {
    if (record == nullptr) {
        return;
    }
    record->header.magic          = kMagic;
    record->header.format_version = kFormatVersion;
    record->header.header_size    = kHeaderSize;
    std::memset(record->pad, 0, sizeof(record->pad));
    record->crc32 = Crc32(record, kCrcCovered);
}

Status Validate(const Record &record) noexcept {
    if (record.header.magic != kMagic) {
        return Status::BadMagic;
    }
    if (record.header.format_version != kFormatVersion) {
        return Status::BadVersion;
    }
    if (record.header.header_size != kHeaderSize ||
        record.header.payload_size > kPayloadSize ||
        record.header.ext_size > kExtSize) {
        return Status::BadLayout;
    }
    if (!Is_Valid_Role(Role_From_U8(record.header.role))) {
        return Status::BadRole;
    }
    if (Crc32(&record, kCrcCovered) != record.crc32) {
        return Status::BadCrc;
    }
    return Status::Ok;
}

Status Decode(const std::uint8_t *raw, std::uint32_t len, Record *out) noexcept {
    if (raw == nullptr || out == nullptr || len != kRecordSize) {
        return Status::BadLayout;
    }
    std::memcpy(out, raw, kRecordSize);
    return Validate(*out);
}

Record Make_Record(Role role) noexcept {
    Record record{};
    record.header.role         = static_cast<std::uint8_t>(role);
    record.header.payload_size = 0U;
    record.header.ext_size     = 0U;
    record.header.sequence     = 0U;
    Seal(&record);
    return record;
}

bool Equivalent(const Record &a, const Record &b) noexcept {
    if (a.header.format_version != b.header.format_version ||
        a.header.role != b.header.role ||
        a.header.payload_size != b.header.payload_size ||
        a.header.ext_size != b.header.ext_size) {
        return false;
    }
    if (a.header.payload_size > kPayloadSize || a.header.ext_size > kExtSize) {
        return false;
    }
    if (std::memcmp(a.payload, b.payload, a.header.payload_size) != 0) {
        return false;
    }
    return std::memcmp(a.ext, b.ext, a.header.ext_size) == 0;
}

}  // namespace board_config

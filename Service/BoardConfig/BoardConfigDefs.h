//
// 板级固化配置（W25Q64）：记录格式与常量
//
// 用途：DM_02 统一固件上电读取“这块板是底盘板还是云台板”，决定运行哪个 App。
// 写入只由独立固件 Board_Configurator 完成；主固件只读。
//
// ============================================================================
// v1 记录（固定 256 B，小端）
// ============================================================================
//   偏移  长度  字段
//   0     4     magic = 0x32304D44（内存字节序 'D','M','0','2'）
//   4     2     format_version = 1
//   6     2     header_size = 32
//   8     1     role：1=底盘，2=云台
//   9     1     flags：预留（v1 写 0，读取忽略）
//   10    2     payload_size：参数区已用字节数（v1 写 0，<=128）
//   12    2     ext_size：扩展区已用字节数（v1 写 0，<=64）
//   14    2     reserved0
//   16    4     sequence：写入序号，递增；Load 取最大
//   20    4     build_tag：预留
//   24    8     reserved1
//   32    128   payload：参数区预留（v1 全 0，读取忽略）
//   160   64    ext：扩展预留区（v1 全 0，读取忽略）
//   224   4     crc32：CRC-32/ISO-HDLC，覆盖 [0,224)
//   228   28    pad：0x00，不参与 CRC
//
// 扩展规则：
//   - 后续加参数：追加进 payload/ext，只增加 payload_size/ext_size，format_version 不变；
//     旧固件忽略未知字节。
//   - 只有记录头布局变化才升 format_version；旧固件会判 BadVersion 并安全停机。
//
// 存储位置：W25Q64 的扇区 0（0x000000）与扇区 1（0x001000），A/B 双槽交替写。
// 自检固件使用最后一个扇区 0x7FF000，不冲突。
//

#ifndef DM_02_BSP_BOARD_CONFIG_DEFS_H
#define DM_02_BSP_BOARD_CONFIG_DEFS_H

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace board_config {

/** 记录魔数：内存字节序为 'D','M','0','2'（小端 0x32304D44） */
inline constexpr std::uint32_t kMagic = 0x32304D44U;

/** 记录格式版本；头布局变化时递增 */
inline constexpr std::uint16_t kFormatVersion = 1U;

/** 记录头长度 */
inline constexpr std::uint16_t kHeaderSize = 32U;

/** 参数区预留长度（v1 未定义字段） */
inline constexpr std::uint16_t kPayloadSize = 128U;

/** 扩展预留区长度（v1 未定义字段） */
inline constexpr std::uint16_t kExtSize = 64U;

/** 单条记录总长度（含 CRC 与尾部填充） */
inline constexpr std::uint32_t kRecordSize = 256U;

/** W25Q 扇区大小（擦除最小单位） */
inline constexpr std::uint32_t kSectorSize = 4096U;

/** A/B 槽的 flash 地址 */
inline constexpr std::uint32_t kSlotAddress[2] = {0x000000U, 0x001000U};

/** 槽数量 */
inline constexpr std::uint8_t kSlotCount = 2U;

/** 板角色：与 App 选择强相关，其余值一律非法 */
enum class Role : std::uint8_t {
    Unknown = 0,  ///< 未配置/读取失败
    Chassis = 1,  ///< 底盘板
    Gimbal  = 2,  ///< 云台板
};

/** 读取/校验结果码；同时发布到调试快照，供外置调试器观察 */
enum class Status : std::int32_t {
    Ok            = 0,  ///< 有效配置
    NotConfigured = 1,  ///< 两槽都是空片（0xFF）
    BadMagic      = 2,  ///< 魔数不符
    BadVersion    = 3,  ///< 格式版本不符
    BadLayout     = 4,  ///< 长度/头字段非法
    BadRole       = 5,  ///< 角色值非法
    BadCrc        = 6,  ///< CRC 不符
    IoError       = 7,  ///< 读/擦/写通信失败
    WriteFailed   = 8,  ///< 擦除或写入失败
    VerifyFailed  = 9,  ///< 回读校验失败
};

/** 记录头（32 B） */
struct Header {
    std::uint32_t magic;           ///< kMagic
    std::uint16_t format_version;  ///< kFormatVersion
    std::uint16_t header_size;     ///< kHeaderSize
    std::uint8_t  role;            ///< Role 数值
    std::uint8_t  flags;           ///< 预留，v1 写 0
    std::uint16_t payload_size;    ///< 参数区已用字节数
    std::uint16_t ext_size;        ///< 扩展区已用字节数
    std::uint16_t reserved0;       ///< 0
    std::uint32_t sequence;        ///< 写入序号
    std::uint32_t build_tag;       ///< 预留
    std::uint8_t  reserved1[8];    ///< 0
};

/** 一条完整记录（256 B）。所有字段显式排布，保证与 flash 字节一一对应。 */
struct Record {
    Header        header;
    std::uint8_t  payload[kPayloadSize];
    std::uint8_t  ext[kExtSize];
    std::uint32_t crc32;  ///< 覆盖 [0, offsetof(crc32))
    std::uint8_t  pad[kRecordSize - sizeof(Header) - kPayloadSize - kExtSize -
                      sizeof(std::uint32_t)];  ///< 不参与 CRC
};

static_assert(sizeof(Header) == kHeaderSize, "记录头必须是 32 字节");
static_assert(offsetof(Record, header) == 0U, "header 必须在记录起点");
static_assert(offsetof(Record, payload) == kHeaderSize, "payload 偏移错误");
static_assert(offsetof(Record, ext) == kHeaderSize + kPayloadSize, "ext 偏移错误");
static_assert(offsetof(Record, crc32) == kHeaderSize + kPayloadSize + kExtSize,
              "crc32 偏移错误");
static_assert(sizeof(Record) == kRecordSize, "记录必须是 256 字节");
static_assert(std::is_trivially_copyable_v<Record>, "记录必须可按字节读写");

/** 角色是否合法（非 Unknown） */
inline constexpr bool Is_Valid_Role(Role role) noexcept {
    return role == Role::Chassis || role == Role::Gimbal;
}

/** Role 数值 -> 枚举（非法值返回 Unknown） */
inline constexpr Role Role_From_U8(std::uint8_t value) noexcept {
    return (value == static_cast<std::uint8_t>(Role::Chassis))
               ? Role::Chassis
               : ((value == static_cast<std::uint8_t>(Role::Gimbal)) ? Role::Gimbal
                                                                    : Role::Unknown);
}

}  // namespace board_config

#endif  // DM_02_BSP_BOARD_CONFIG_DEFS_H

//
// 板级固化配置（W25Q64）：纯逻辑编解码
//
// 不依赖 HAL / flash 驱动，可在主机侧单测。记录格式见 BoardConfigDefs.h。
//

#ifndef DM_02_BSP_BOARD_CONFIG_CODEC_H
#define DM_02_BSP_BOARD_CONFIG_CODEC_H

#include <cstdint>

#include "BoardConfigDefs.h"

namespace board_config {

/** CRC-32/ISO-HDLC（多项式 0xEDB88320，init/xorout 0xFFFFFFFF） */
std::uint32_t Crc32(const void *data, std::uint32_t len) noexcept;

/**
 * 补齐固定头字段并计算 CRC：
 *   magic / format_version / header_size / pad 清零；crc32 覆盖 [0, 224)。
 * 不修改 role/sequence/payload_size/ext_size/payload/ext（由调用方设置）。
 */
void Seal(Record *record) noexcept;

/**
 * 校验一条已在内存中的记录：
 *   magic -> version -> header_size/payload_size/ext_size -> role -> CRC。
 * 不修改记录。
 */
Status Validate(const Record &record) noexcept;

/**
 * 从 flash 原始字节解码：长度必须为 kRecordSize；先整体拷贝再 Validate。
 * @param raw  原始字节（至少 len 字节）
 * @param len  必须等于 kRecordSize
 * @param out  成功与否都可写入（失败时为原始拷贝，便于调试）
 */
Status Decode(const std::uint8_t *raw, std::uint32_t len, Record *out) noexcept;

/** 构造一条指定角色的空参数记录（payload/ext 全 0，payload_size=ext_size=0）。 */
Record Make_Record(Role role) noexcept;

/**
 * 内容等价判断（配置器用于“已是最新则跳过写入”）：
 *   比较 format_version、role、payload_size、ext_size、payload、ext。
 * 不比较 sequence/CRC/pad。
 */
bool Equivalent(const Record &a, const Record &b) noexcept;

}  // namespace board_config

#endif  // DM_02_BSP_BOARD_CONFIG_CODEC_H

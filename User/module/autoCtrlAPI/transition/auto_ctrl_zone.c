#include "module/autoCtrlAPI/transition/auto_ctrl_zone.h"

/*
 * auto_ctrl_zone.c
 *
 * 作用：
 * - 维护区块静态属性表；
 * - 提供 zone -> 信息/高度/平台属性 查询；
 * - 提供 zone 枚举合法性判断。
 *
 * 关键约束：
 * - k_auto_ctrl_zone_table 的索引必须与 auto_ctrl_zone_e 枚举值保持一致。
 */

/* 区块元数据表，索引与 auto_ctrl_zone_e 一一对应。 */
static const auto_ctrl_zone_info_t k_auto_ctrl_zone_table[] = {
    {AUTO_CTRL_ZONE_INVALID, "INVALID", 0, false},
  {AUTO_CTRL_ZONE_R2_ENTRY1, "R2_ENTRY1", 0, false},
  {AUTO_CTRL_ZONE_R2_ENTRY2, "R2_ENTRY2", 0, false},
  {AUTO_CTRL_ZONE_R2_ENTRY3, "R2_ENTRY3", 0, false},
    {AUTO_CTRL_ZONE_PLATFORM_1, "PLATFORM_1", 400, true},
    {AUTO_CTRL_ZONE_PLATFORM_2, "PLATFORM_2", 200, true},
    {AUTO_CTRL_ZONE_PLATFORM_3, "PLATFORM_3", 400, true},
    {AUTO_CTRL_ZONE_PLATFORM_4, "PLATFORM_4", 200, true},
    {AUTO_CTRL_ZONE_PLATFORM_5, "PLATFORM_5", 400, true},
    {AUTO_CTRL_ZONE_PLATFORM_6, "PLATFORM_6", 600, true},
    {AUTO_CTRL_ZONE_PLATFORM_7, "PLATFORM_7", 400, true},
    {AUTO_CTRL_ZONE_PLATFORM_8, "PLATFORM_8", 600, true},
    {AUTO_CTRL_ZONE_PLATFORM_9, "PLATFORM_9", 400, true},
    {AUTO_CTRL_ZONE_PLATFORM_10, "PLATFORM_10", 200, true},
    {AUTO_CTRL_ZONE_PLATFORM_11, "PLATFORM_11", 400, true},
    {AUTO_CTRL_ZONE_PLATFORM_12, "PLATFORM_12", 200, true},
    {AUTO_CTRL_ZONE_R2_EXIT1, "R2_EXIT1", 0, false},
    {AUTO_CTRL_ZONE_R2_EXIT2, "R2_EXIT2", 0, false},
    {AUTO_CTRL_ZONE_R2_EXIT3, "R2_EXIT3", 0, false},
};

/* 查询区块信息。越界时返回 INVALID，避免上层空指针判断分散。 */
const auto_ctrl_zone_info_t *AutoCtrlZone_GetInfo(auto_ctrl_zone_e zone) {
  if (zone <= AUTO_CTRL_ZONE_INVALID || zone >= AUTO_CTRL_ZONE_COUNT) {
    return &k_auto_ctrl_zone_table[0];
  }
  return &k_auto_ctrl_zone_table[(uint32_t)zone];
}

/* 获取区块高度（mm），内部复用统一信息查询接口。 */
int16_t AutoCtrlZone_GetHeightMm(auto_ctrl_zone_e zone) {
  return AutoCtrlZone_GetInfo(zone)->height_mm;
}

/* 判断区块是否平台。 */
bool AutoCtrlZone_IsPlatform(auto_ctrl_zone_e zone) {
  return AutoCtrlZone_GetInfo(zone)->is_platform;
}

/* 判断区块枚举是否处于有效范围。 */
bool AutoCtrlZone_IsValid(auto_ctrl_zone_e zone) {
  return zone > AUTO_CTRL_ZONE_INVALID && zone < AUTO_CTRL_ZONE_COUNT;
}
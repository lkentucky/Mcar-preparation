#ifndef _FLASH_H
#define _FLASH_H

#include "zf_driver_flash.h"
#include "Motor.h"
#include "route_follow.h"

/* 使用 4MB Flash 最后一个 32KB 扇区（127 号扇区）保存参数，远离程序代码区。
 * 与原工程保持一致的分区选择，擦写只影响该扇区。 */
#define FLASH_SECTION_INDEX 127U
#define FLASH_PAGE_INDEX FLASH_PAGE_3

/* 掉电保存的菜单参数。字段按本工程（Mcar）的菜单内容重新定义：
 * 四轮速度 PID、Position 位置外环参数、Navigation 安装参数。
 * 原来 ASC 工程的 checkpoint_vision / show_map 等字段在本工程没有对应功能，已删除。 */
typedef struct
{
    /* 四轮速度 PID，顺序 UL UR DL DR，与 PID 菜单一致 */
    float wheel_kp[MOTOR_WHEEL_COUNT];
    float wheel_ki[MOTOR_WHEEL_COUNT];
    float wheel_kd[MOTOR_WHEEL_COUNT];
    /* v5: nonnegative PWM magnitudes, UL/UR/DL/DR. */
    int32 wheel_deadzone_fwd[MOTOR_WHEEL_COUNT];
    int32 wheel_deadzone_rev[MOTOR_WHEEL_COUNT];

    /* Position 外环参数，与 position_config_t 字段一一对应 */
    float xy_kp;
    float xy_kd;
    float yaw_kp;
    float max_speed_cmps;
    float max_omega_radps;
    float max_accel_cmps2;
    float max_alpha_radps2;
    float xy_tolerance_cm;
    float yaw_tolerance_deg;
    /* ivision 式开环规划参数（Flash v3 新增，v1/v2 存档加载时用默认值） */
    float brake_limit;
    float brake_ceiling_cmps2;
    float short_segment_cm;
    float short_boost_gain;
    float approach_zone_cm;
    float approach_ratio;
    float approach_acc_k;
    float yaw_lin_band_rad;
    float yaw_kd;
    float yaw_kd_translate;
    /* Flash v4 新增（v1/v2/v3 存档加载时用默认值） */
    float yaw_stiction_radps;
    /* Flash v6 新增：斜向调整两参数（v1~v5 存档加载时用默认值） */
    float diag_speed_k;
    float diag_comp;

    /* Navigation 安装参数 */
    float mount_deg;
    float scale_x;
    float scale_y;
    uint8 yaw_reversed;

    /* Route data only: runtime state and Run are never persisted. */
    uint8 route_node_count;
    route_node_t route_nodes[ROUTE_MAX_NODES];
} menu_flash_config_t;

/* Save: 1 OK; -1 config, -2 erase, -3 program, -4 verify, -5 init.
 * Load: 1 OK; -1 empty, -2 header/checksum, -3 config, -4 init.
 * BadWord is the offending serialized field index (19=Acc,63..70=deadzones). */
extern volatile int32 menu_flash_save_status, menu_flash_load_status;
extern volatile uint32 menu_flash_bad_word;
extern volatile float menu_flash_stored_accel;

uint8 Data_save_to_flash(const menu_flash_config_t *config);
uint8 Data_load_from_flash(menu_flash_config_t *config);
uint8 Data_clear_flash(void);

/* 便捷接口：直接保存/恢复当前生效的全局参数（PID、位置环、导航安装角等）。
 * 保存必须由主循环（菜单按键）触发，不能在中断中调用。 */
uint8 menu_flash_save_current(void);
uint8 menu_flash_load_current(void);

#endif

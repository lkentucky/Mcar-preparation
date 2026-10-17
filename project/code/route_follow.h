/** 多点路线跟随：中间节点圆滑通过，终点用现有位置环停车。
 * 节点在世界坐标（Navigation/Zero 坐标系）中定义，单位 cm 与度。
 * Blend_cm=0 时恢复逐点停车；故障自动停车。 */
#ifndef MCAR_ROUTE_FOLLOW_H
#define MCAR_ROUTE_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>
#include "zf_common_typedef.h"
#include "position_control.h"

#define ROUTE_BLEND_DEFAULT_CM 2.0f
#define ROUTE_BLEND_MIN_CM 0.1f
#define ROUTE_FLOW_DEFAULT_CM 20.0f

#define ROUTE_MAX_NODES 15
#define ROUTE_DEFAULT_NODE_COUNT 5
#define ROUTE_DEFAULT_NODES {{0.0f,0.0f,0.0f},{0.0f,100.0f,0.0f},{100.0f,100.0f,0.0f},{200.0f,0.0f,0.0f},{0.0f,0.0f,0.0f}}

enum {
    ROUTE_FAULT = -1,   /* 位置环故障（定位无效/参数非法），已停车 */
    ROUTE_IDLE = 0,     /* 未运行 */
    ROUTE_ARMING = 1,   /* 正在切入位置模式，等待模式切换周期 */
    ROUTE_RUNNING = 2,  /* 正在驶向当前节点 */
    ROUTE_ADVANCE = 3,  /* 节点到达，下一拍切换新目标 */
    ROUTE_DONE = 4      /* 全部节点完成，已停稳 */
};

typedef struct {
    float x_cm, y_cm, yaw_deg;
} route_node_t;

/* 菜单直接绑定的变量 */
extern bool route_run_flag;                      /* Route/Run 开关（显示用） */
extern volatile int16_t route_node_count;        /* 生效节点数 1..ROUTE_MAX_NODES */
extern volatile int32_t route_state;             /* 上方枚举，Route/State 显示 */
extern volatile int32_t route_current_idx;       /* 当前目标节点下标（0 起） */
extern route_node_t route_nodes[ROUTE_MAX_NODES];
/* Nominal node tracking tolerance; 0 = stop at every node.
 * Through-node geometry interpolates nodes, not the connecting straight lines. */
extern volatile float route_blend_cm;
/* Bezier tangent handle length, 1..100 cm. Short legs limit it automatically. */
extern volatile float route_flow_cm;
enum {
    ROUTE_MODE_WAIT = 0,
    ROUTE_MODE_CURVE = 1,
    ROUTE_MODE_POINT = 2, /* explicitly selected by Blend_cm=0 */
    ROUTE_MODE_SETTLE = 3 /* local yaw/reversal stop, or final settling */
};
enum {
    ROUTE_STOP_NONE = 0,
    ROUTE_STOP_DISABLED = 1,
    ROUTE_STOP_YAW = 2,
    ROUTE_STOP_REVERSE = 3,
    ROUTE_STOP_SINGLE = 4
};
/* Runtime diagnostics, not persisted. Vref is the planned scalar speed;
 * Err is cross-track error in the navigation frame, not physical error. */
extern volatile int32_t route_flow_mode, route_stop_reason;
extern volatile float route_vref_cmps, route_error_cm;
/* Called inside the position-mode motor tick, using the fresh encoder pose. */
void route_follow_control_update(position_control_t *control,
                                 const position_config_t *config,
                                 const position_goal_t *goal,
                                 const navigation_snapshot_t *pose, float dt);

void route_follow_init(void);
/* 10ms 周期调用（isr.c 中紧跟 app_control_motor_tick_10ms 之后） */
void route_follow_tick_10ms(void);
/* 菜单 Run 置 On/Off 时调用 */
void route_follow_start(void);
void route_follow_stop(void);
/* 菜单中修改节点数或任一节点坐标后调用：运行中则停车取消路线 */
void route_follow_config_edited(void);
/* 判断菜单指针是否落在节点数据区，用于编辑拦截 */
uint8 route_follow_is_node_field(const void *ptr);
/* 节点字段限幅：yaw ±180°，X/Y ±10000cm */
float route_follow_clamp_node_field(const void *field, float value);

#endif

#ifndef MCAR_APP_NAVIGATION_H
#define MCAR_APP_NAVIGATION_H

#include "navigation_fusion.h"

extern volatile float navigation_mount_deg;
extern volatile bool navigation_yaw_reversed;
/* Body-forward / body-left odometry multipliers, independent of yaw.
 * Editing either rebuilds the Zero origin and cancels active position Run. */
extern volatile float navigation_scale_x, navigation_scale_y;

void app_navigation_init(void);
void app_navigation_imu_tick_5ms(void);
void app_navigation_encoder_tick_10ms(void);
void app_navigation_request_reset(void);
/* 主循环调用时先关中断，调用后恢复；中断内可直接读取。 */
void app_navigation_get_snapshot(navigation_snapshot_t *snapshot);

#endif

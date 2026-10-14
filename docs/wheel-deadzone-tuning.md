# 四轮正反转死区临时调节

全局数组：`motor_deadzone_fwd[4]`、`motor_deadzone_rev[4]`，顺序为 UL、UR、DL、DR。它们直接被四轮速度闭环读取。

Drive 菜单新增 `UL_DeadF/UL_DeadR`、`UR_DeadF/UR_DeadR`、`DL_DeadF/DL_DeadR`、`DR_DeadF/DR_DeadR`。F 为正转，R 为反转；两种方向均填写非负 PWM 幅值，0 关闭该方向补偿。

对应 WiFi 名称：

| 轮子 | 正转 | 反转 |
|---|---|---|
| UL | `ul_deadzone_fwd` | `ul_deadzone_rev` |
| UR | `ur_deadzone_fwd` | `ur_deadzone_rev` |
| DL | `dl_deadzone_fwd` | `dl_deadzone_rev` |
| DR | `dr_deadzone_fwd` | `dr_deadzone_rev` |

例如发送 `[slider,dr_deadzone_rev,600]`，也接受 `["slider","dr_deadzone_rev",600]`。成功应答 `MCAR SLIDER dr_deadzone_rev 600`。范围 0～2000，只接受整数；小数或越界值不修改参数。可用 SUB 订阅这些名称确认当前值。

修改即时生效，暂不写 Flash，重启恢复 `Motor.h` 的默认值。不同轮子的值无需相等，应按各轮实际起转响应调节。

补偿作用于速度闭环 `motor_control()`；PWM_Test 直接输出指定 PWM，不额外叠加这些值。添加死区不会启动电机或改变运行模式。

# Mcar-preparation

RT1064 麦克纳姆轮底盘准备工程。当前版本已集成：

- ASC-AIVision_Group2 的四轮电机、编码器、麦轮逆运动学和独立轮速 PID；
- ASC 菜单的树形节点与四键交互，并按本工程功能重组菜单项；
- rt1064_imu_vofa 的 IMU660RA 读取、静止零偏标定、Mahony/Madgwick 六轴融合、四元数积分和欧拉角解算；
- rt1064_imu_vofa 的 WiFi-SPI2.0 UDP 姿态输出。

## 实时任务

| PIT | 周期 | 内容 |
|---|---:|---|
| CH0 | 5 ms | IMU 读取、标定和姿态融合 |
| CH1 | 10 ms | 编码器读取、直接 PWM 测试或四轮速度闭环 |
| CH2 | 20 ms | 四键扫描 |

UDP 发送和 IMU 重新初始化在主循环执行，不阻塞上述中断。

## 菜单

IPS200 使用竖屏 240×320，按键沿用 ASC 的操作方式：

- KEY1：进入文件夹、选择参数；参数已选中时切换步进；
- KEY2：上移；参数已选中时增加；
- KEY3：返回；参数已选中时取消选择；
- KEY4：下移；参数已选中时减小。

选中数值参数后，KEY2/KEY4 按住 1 秒开始连续加减，每 80 ms 按当前步进调整一次。长按不会连续切换文件夹或开启 Run；松开后不再补一次短按。

菜单包含 Drive、PWM_Test、Encoder、IMU、Sensor、WiFi 和 PID。默认进入直接 PWM 测试模式，电机输出禁止，四轮 PWM 均为 0。

进入 `PWM_Test`，保持 `OpenLoop=On`，设置 `UL_PWM`（左前）、`UR_PWM`（右前）、`DL_PWM`（左后）、`DR_PWM`（右后），再将 `Run` 设为 On。测试单轮时其余三轮保持 0；例如 `UL_PWM=1000` 为左前正转 10% 占空比，`-1000` 为反转 10%。当前范围为 -2000～2000（由 `Motor.h` 的 `LIMIT_PWM_MIN/MAX` 设置），频率为 17 kHz；PWM 参数步进为 1、10 或 100，原菜单小数步进在这里按 1 处理。

测试链路为 `motor_test_pwm[] → motor_pwm() → DIR/PWM`，绕过逆运动学、速度 PID、死区补偿及横移启动补偿。编码器仍读取并显示，反馈不会改变 PWM。测试模式下 `Run=Off` 在下一个 10 ms 周期将四路占空比直接置零。

`Encoder` 页面每 100 ms 自动刷新，四轮均显示 `Raw`（方向校正后、滤波前的 10 ms 计数）、`Filt`（滤波后的 10 ms 计数）和 `Total`（方向校正后的累计计数）。`ZeroTotal` 选中后按 KEY2 清零显示累计值，操作完成自动回到 Off，不影响电机输出和 PID。底部 `PWM` 显示四轮最终软件输出，顺序为 UL、UR、DL、DR。

编码器测试先保持 `OpenLoop=On`、`Run=Off`，手动逐个转动车轮，确认只对应那一轮的 Raw/Total 变化。再用 PWM_Test 分别给单轮正、负 PWM，确认正 PWM 时该轮读数为正、负 PWM 时为负；慢速手转时 Raw 可能间歇为 0，应观察 Total。若电机正 PWM 的物理方向不正确，调整该轮 `MOTORn_FORWARD_LEVEL`；若物理方向正确但编码器符号相反，调整对应 `ENCODER_n_FORWARD_SIGN`。如累计值跳动而轮子静止，应先检查 A/B 接线和共地。

`PWM_Test/OpenLoop=Off` 可恢复 `Drive` 中的车体速度闭环。切换模式会停止输出、清空轮速 PID，并将共享的 `Run` 设为 Off；需要重新开启 Run。

## 电机与编码器

电机输出、轮速 PID 和调试快照统一按左前、右前、左后、右后排列。驱动使用 `project/code/Motor.h` 中的当前接线定义，仅保留一套实现。

| 车轮 | 电机通道 | DIR / PWM 引脚 | 编码器模块 | A / B 引脚 |
|---|---|---|---|---|
| 左前 UL | MOTOR1 | D12 / D14 | QTIMER3_ENCODER2 | B18 / B19 |
| 右前 UR | MOTOR2 | D13 / D15 | QTIMER2_ENCODER1 | C3 / C25 |
| 左后 DL | MOTOR3 | D0 / D2 | QTIMER1_ENCODER1 | C0 / C1 |
| 右后 DR | MOTOR4 | D1 / D3 | QTIMER1_ENCODER2 | C2 / C24 |

PWM 频率为 17 kHz，当前命令限幅为 ±2000。电机 1、2 使用 PWM1_MODULE1 的 A、B 通道；电机 3、4 使用 PWM2_MODULE3 的 A、B 通道。编码器物理编号依次对应左后、右后、右前、左前，读取时转换为统一轮序，再进入滤波和 PID。

每轮正转电平由 `MOTOR1_FORWARD_LEVEL` 至 `MOTOR4_FORWARD_LEVEL` 设置，编码器方向由 `ENCODER_1_FORWARD_SIGN` 至 `ENCODER_4_FORWARD_SIGN` 设置。当前值沿用原来各逻辑轮的方向校正；实际电机接线后的正转与反馈符号仍需在车上确认。

`tests/motor_mapping_test.c` 用记录 GPIO、PWM 和编码器调用的主机替身检查独轮正反转、初始化、限幅、反馈顺序、PID 输出通道以及 PWM 测试模式，不驱动实物。使用真实 PWM 和编码器枚举头文件，编译时将 `tests/motor_stubs`、`libraries/zf_driver` 和 `project/code` 加入头文件路径，链接 `Motor.c`、`PID.c`、`PID_config.c`、`app_control.c` 和数学库。

## IMU

IMU660RA 使用 SPI4：C23/C22/C21/C20。上电后须保持静止，连续取得 400 个合格样本后进入运行态。状态值：

- `0`：标定中；`1`：正常运行；
- `-1`：初始化失败；`-2`：采样超时；
- `-3`：采样间隔异常；`-4`：融合计算异常。

在 IMU/Recal 中置 On 可重新标定。算法由 `project/code/config.h` 的 `AHRS_METHOD` 选择。

## WiFi-SPI

当前 `config.h` 的 `IMU_WIFI_ENABLED=0`：电机测试固件禁用 WiFi 初始化及发送，菜单状态为 -5。原 WiFi-SPI1 的 SCK=D12、MOSI=D14、MISO=D15、CS=D13，与前轮 DIR/PWM 冲突；WiFi 初始化会覆盖电机引脚复用，即使联网失败也会造成冲突。恢复 WiFi 前必须确认新的非冲突接线并修改驱动引脚；继续使用 SPI1 时重新启用会被编译检查阻止。

WiFi 模块使用逐飞 WiFi-SPI2.0 驱动，通过 UDP 发送 JustFloat 数据。发送间隔由 `project/code/config.h` 的 `IMU_WIFI_PERIOD_MS` 决定，当前为 2 ms（目标 500 帧/秒，实际频率受主循环及同步发送耗时影响）。热点、密码、目标 IP 和端口也在该文件配置。WiFi 初始化失败不会停止 IMU 与电机控制。

默认 CH0/CH1/CH2 为 roll/pitch/yaw（单位：度）。报文格式是 `N 个小端 float32 + 00 00 80 7F`，总长度为 `4*N+4` 字节，默认 16 字节。不自动添加时间戳。VOFA+ 应选择 UDP 接收和 JustFloat 解析；原来只接收 12 字节姿态数据的程序需要适配新帧长与帧尾。

修改周期发送内容，只需编辑 `project/code/app_wifi_telemetry.c` 中的 `channels[]` 列表。通道数和帧长自动计算，最多 40 通道，不需要修改打包或发送函数。每次在短暂关中断期间取得数据快照，恢复中断后才打包并同步发送。

例如添加四轮编码器，在该文件包含 `Motor.h` 后，将列表改为：

```c
const float channels[] = {
    imu_roll_deg, imu_pitch_deg, imu_yaw_deg, /* CH0~2，度 */
    up_L_all, up_R_all, down_L_all, down_R_all /* CH3~6，编码器计数 */
};
```

主循环中的其他调试位置也可用快捷接口 `wifi_justfloat(imu_roll_deg, imu_pitch_deg, imu_yaw_deg)`，或 `imu_wifi_send_floats(data, count)`。快捷接口会自动计算参数数量，并将整数转成浮点数；整数超过 float32 的精确表示范围时会丢失精度。不要在中断或关中断区内调用发送接口。通道顺序变化后，上位机的名称、单位和曲线绑定也需同步更新。

## 构建

用 Keil MDK 打开 `project/mdk/rt1064.uvprojx`，构建目标 `nor_sdram_zf_dtcm`。已使用 Arm Compiler 6.19 验证：0 errors，0 warnings。

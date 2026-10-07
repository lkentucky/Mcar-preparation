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

`Encoder` 页面每 100 ms 自动刷新，四轮均显示 `Raw`（方向校正后、滤波前的 10 ms 计数）、`Filt`（滤波后的 10 ms 计数）和 `Total`（方向校正后的累计计数），这些列均保留编码器的真实计数。下方 `cm/s` 按各轮的分辨率换算轮缘速度。`ZeroTotal` 选中后按 KEY2 清零显示累计值，操作完成自动回到 Off，不影响电机输出和 PID。底部 `PWM` 显示四轮最终软件输出，顺序为 UL、UR、DL、DR。

编码器测试先保持 `OpenLoop=On`、`Run=Off`，手动逐个转动车轮，确认只对应那一轮的 Raw/Total 变化。再用 PWM_Test 分别给单轮正、负 PWM，确认正 PWM 时该轮读数为正、负 PWM 时为负；慢速手转时 Raw 可能间歇为 0，应观察 Total。若电机正 PWM 的物理方向不正确，调整该轮 `MOTORn_FORWARD_LEVEL`；若物理方向正确但编码器符号相反，调整对应 `ENCODER_n_FORWARD_SIGN`。如累计值跳动而轮子静止，应先检查 A/B 接线和共地。

`PWM_Test/OpenLoop=Off` 可恢复 `Drive` 中的车体速度闭环。切换模式会停止输出、清空轮速 PID，并将共享的 `Run` 设为 Off；需要重新开启 Run。

## 电机与编码器

电机输出、轮速 PID 和调试快照统一按左前、右前、左后、右后排列。驱动使用 `project/code/Motor.h` 中的当前接线定义，仅保留一套实现。

| 车轮 | 电机通道 | DIR / PWM 引脚 | 编码器模块 | A / B 引脚 |
|---|---|---|---|---|
| 左前 UL | MOTOR1 | D13 / D15 | QTIMER3_ENCODER2 | B18 / B19 |
| 右前 UR | MOTOR2 | D12 / D14 | QTIMER2_ENCODER1 | C3 / C25 |
| 左后 DL | MOTOR3 | D0 / D2 | QTIMER1_ENCODER1 | C0 / C1 |
| 右后 DR | MOTOR4 | D1 / D3 | QTIMER1_ENCODER2 | C2 / C24 |

PWM 频率为 17 kHz，当前命令限幅为 ±2000。电机 1、2 使用 PWM1_MODULE1 的 A、B 通道；电机 3、4 使用 PWM2_MODULE3 的 A、B 通道。编码器物理编号依次对应左后、右后、右前、左前，读取时转换为统一轮序，再进入滤波和 PID。

当前车轮直径为 11 cm，减速比约 2.3。UL/UR/DL 编码器为 1024 线，DR 为 512 线；底层按 A 相上升沿计数、B 相判方向，不采用四倍频。因此前三轮约为 2355.2 计数/车轮圈，DR 约为 1177.6 计数/车轮圈。轮缘速度 `cm/s = 每10ms的计数 × 100 × π × 0.11 × 100 / 该轮每圈计数`；前三轮每计数约为 1.4673 cm/s，DR 每计数约为 2.9346 cm/s。

四轮 PID 目标统一按 UL 的参考分辨率计算，PID 入口将 DR 的真实反馈乘 2；横移距离检测、四轮平均计数及闭环停止阈值也使用参考计数。Raw/Filt/Total 不乘 2，所以同速时 DR 的计数应约为其他轮的一半，页面 cm/s 应相同。减速比是近似值，最终应以各轮实测每圈计数校准；旧 PID 参数仍需在当前电机上调试。

每轮正转电平由 `MOTOR1_FORWARD_LEVEL` 至 `MOTOR4_FORWARD_LEVEL` 设置，编码器方向由 `ENCODER_1_FORWARD_SIGN` 至 `ENCODER_4_FORWARD_SIGN` 设置。当前值沿用原来各逻辑轮的方向校正；实际电机接线后的正转与反馈符号仍需在车上确认。

DR 实测反馈方向相反，已将 `ENCODER_2_FORWARD_SIGN` 修正为 +1。方向修正发生在原始计数进入滤波和累计之前，页面计数、速度及闭环反馈使用同一符号。PWM 测试中不使用反馈调整输出；相同 PWM 下应比较 cm/s，原始计数相差一倍符合 1024/512 线的硬件差异。

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

## 编码器与 IMU 定位融合

融合在 `navigation_fusion.c` 实现，由 `app_navigation.c` 接入现有调度：每个成功的 5 ms IMU 新帧预测速度，每 10 ms 读取一次方向修正后的四轮原始计数，修正速度并更新位置。重复 IMU 帧不会重复积分；编码器不会被二次读取或清零。PWM 测试和 Run=Off 时仍跟踪实际运动，融合输出已接到可选的位置外环，不包含路线回放。

坐标为起点车头 +X、左侧 +Y、从上方看逆时针航向为正。按本车实测，左转 Yaw 增大、静止 Az=+1g、IMU 的 X 朝车尾，故默认 `Mount_deg=180`、`YawFlip=Off`。去倾斜后用安装角把水平加速度映射到车体轴，再转换到固定坐标系。四轮各用自己的每圈实际计数换算距离，DR 的 512 线不会使其位移减半。位置积分使用原始增量及 IMU 航向变化，采用圆弧积分处理同时平移和旋转；不会按速度指令推算位置。

参考 [HDU 车端组合里程计](https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield) 的惯性预测、编码器修正、静止归零和打滑减权思路，以及旧 `project/code/path_follow.c` 的二维坐标变换，结合当前接口独立实现。正常编码器权重为 1，保留实际位移；速度创新与加速度差同时超限时，编码器权重短时降为 0.15，持续 8 个控制周期。此检测是启发式判断，不能识别所有打滑或消除长期漂移。参数集中在 `navigation_config.h`，需在本车重新验证。加速度预测也受安装误差、振动及 IMU 距离旋转中心的偏移影响，当前未做传感器杆臂补偿。

操作：

1. 确认后轮已换为标准 X 排列，四轮编码器正方向正确；静止上电，等待原有 IMU 标定结束，再静止约 0.5 s 建立水平加速度偏置。
2. 打开 `Navigation`，`State=2`、`Valid=1`、`Bias=1` 表示可用。`X_cm/Y_cm` 是位置，`Yaw_deg` 是相对起点的连续航向；下方显示固定坐标速度、`Slip` 和 `Rest`。根菜单新增第八个文件夹后会自动滚动，PID 仍可访问。
3. 停车后选中 `Zero` 按 KEY2，在下一个 10 ms 周期重建位置和航向起点，重新静止标定约 0.5 s；不重置 Encoder 页的 Total。修改 Mount_deg/YawFlip 同样重建定位起点。位置模式中这些操作使定位暂时不可用，位置控制随即关闭 Run、清轮速 PID 并停止 PWM；手动 Drive/PWM 模式仍由各自的 Run 控制。
4. State：`0` 等待 IMU，`1` 静止偏置标定，`2` 正常；`-1` 输入/配置非法，`-2` IMU 失效，`-3` 编码器异常。IMU 约 50 ms 无新帧、姿态重标定或明显异常编码器脉冲会冻结位置并使 Valid=0；排除原因后停车 Zero，不能直接继续使用旧坐标。

首次硬件验证：手推前进 50 cm，X 应增加约 50；向左推 50 cm，Y 应增加；绕车体中心旋转时航向变化、XY 应基本不变；最后测试圆弧运动。定点的实际距离取决于每圈计数及前后/横向距离比例。当前减速比仍为约 2.3，`LATERAL_CORRECTION_FACTOR` 沿用旧值，不能把菜单小数位数当成定位精度。

主机验证：运行 `tests/run_navigation_tests.ps1`，覆盖混合编码器分辨率、前后/横移、四分之一圆弧、原地旋转、倾斜重力消除、180° 安装、航向跨圈、偏置、打滑与故障冻结；并回归实际电机调度、IMU 新帧/重标定适配、Zero 和 240×320 菜单边界。

## 定点位置外环

`position_control.c` 是独立控制器。`app_control_motor_tick_10ms()` 在采集编码器并更新融合后，取得同一时刻的 X/Y/Yaw 和固定坐标速度，每 10 ms 计算一次位置指令，再调用原有 `Kinematics_Inverse()` 和 `motor_control()`。完整链路为目标点 → 融合位置反馈 → 位置外环 → 车体 Vx/Vy/Omega → 麦轮四轮目标 → 轮速 PID → PWM。PWM 测试模式不经过位置环或轮速 PID；普通 Drive 保持手动速度控制。

目标 XY 的单位是 cm，使用 Navigation/Zero 的固定坐标，不随当前车头旋转；目标 Yaw 单位是度，以 Zero 时车头为 0，逆时针为正。平移采用 `世界速度 = XY_Kp × 位置误差 − XY_Kd × 世界实测速度`，按向量长度限速；航向采用最短角误差的 P 控制并限转速。用当前融合航向 θ 转成车体指令：`Vx = cosθ × V世界X + sinθ × V世界Y`，`Vy = −sinθ × V世界X + cosθ × V世界Y`。例如车头已经左转 90°、目标仍在起点正前方时，控制器会发出车体右移指令。同时设定 XY 和 Yaw 可边平移边旋转。

默认参数在 `position_control.h`：XY_Kp=2/s，XY_Kd=0.2，Yaw_Kp=2/s；最大平移速度 20 cm/s、最大转速 1 rad/s，世界平移加速度上限 40 cm/s²、角加速度上限 2 rad/s²。平移容差 2 cm、航向容差 3°；进入容差后将指令缓降到零，实测平移速度 ≤3 cm/s 且航向变化速度 ≤0.1 rad/s，连续保持 0.2 s 才判定到达。无位置积分；容差内完成后关闭 Run，不持续锁住该点。减速比误差、IMU 漂移、打滑仍会影响真实落点，主机仿真不能替代实车调参。

菜单操作：

1. 静止上电，等 Navigation 的 State=2、Valid=1、Bias=1。需要重设起点时先停车，再 Zero 并等待定位重新就绪。
2. 在根菜单最后一项 `Position` 中将 Enable 设 On；会自动令 PWM_Test/OpenLoop=Off，并令共享 Run=Off。初始上电仍是 PWM 测试，位置模式默认关闭。文件夹和参数超出七行后自动滚动。
3. 设置 TargetX_cm / TargetY_cm / TargetYaw。例如 `(50, 0, 0)` 表示向起点前方走到 50 cm，`(0, 50, 0)` 表示向左走到 50 cm，`(50, 0, 90)` 表示走向该点并左转至 90°。目标 X/Y 可为负，Yaw 在 −180° 至 +180°。
4. 首次可设 MaxV_cmps=10，其余参数先保留默认值，最后将 Position/Run 设 On。Drive 显示的是此时自动生成的 Vx/Vy/Omega，位置模式中不应在 Drive 修改速度。Position 下方显示指令和当前 XY/Yaw，ErrXY_cm / ErrYaw_deg 显示误差。
5. State=0 待启动，1 移动，2 到达范围内等待停止，3 已到达；−1 定位不可用，−2 参数非法或 PWM/位置模式冲突。到达或故障会自动关 Run、清轮速 PID、PWM 归零；新目标需要再次 Run。定位恢复不会自动续跑。切换 Enable/OpenLoop 也取消 Run；打开 PWM_Test/OpenLoop 会退出位置模式。

所有目标以最近一次 Zero 的起点为参照，完成一个目标后设置新目标不会重置坐标。Run=On 时修改目标允许平滑转向新目标，并重新计算到达等待时间；Run=Off 时修改目标不会启动电机。

代码接口是 `app_control.h` 的 `motor_position_enabled`、`motor_position_goal` 和 `motor_position_config`。切换模式先关闭 Run，再置 `motor_pwm_test_enabled=false`、`motor_position_enabled=true`，等待至少一个 10 ms 控制周期完成切换后才能打开 Run；菜单负责这一操作。主循环读取 `app_control_get_position_snapshot()` 时要在短暂关中断区内复制快照，与导航接口一致。

主机测试额外覆盖世界/车体坐标变换、跨 ±180° 转向、限速与加速度限制、前后左右和同时转向的带滞后模型收敛、低速持续到达判定、真实融合→解算→轮速 PID→PWM 接线、定位失效和运行中 Zero 停车、模式互斥、手动 Drive/PWM 回归，以及 Position 的负数编辑和全部 17 项菜单滚动。

## 构建固件

用 Keil MDK 打开 `project/mdk/rt1064.uvprojx`，构建目标 `nor_sdram_zf_dtcm`。已使用 Arm Compiler 6.19 验证：0 errors，0 warnings。

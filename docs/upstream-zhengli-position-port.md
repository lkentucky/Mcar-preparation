# 上游版本比较与位置环移植

日期：2026-10-07。

## 版本与历史

- 上游仓库：`https://github.com/zt3543038264-cloud/Mcar-preparation.git`，remote 为 `upstream`。
- fork：`https://github.com/lkentucky/Mcar-preparation.git`，remote 为 `origin`。
- 对比基线：`00c49d8`，提交说明“麦轮运动学解算”。
- 拉取的上游：`整理`，完整 SHA `980c83390a664d1b5009d7f69be4624f88b33259`。
- 移植参考：fork 的 `master`，`191ddb966155b49024867e3231ed687d3e72d88e`，提交说明“位置环”。

本地 `整理` 直接从上游同名分支创建，保留其祖先提交；位置环作为后续提交接入。上游代码通过 Git fetch 拉取，位置环按新接口移植，不用文件目录副本代替上游历史。fork 的 `master` 保留原位置环版本。

## 与麦轮运动学解算版本的比较

使用 `git diff 00c49d8 980c833` 核对，共 29 个路径变化，主要为 IMU/WiFi 文件合并和代码整理。

| 模块 | 上游“整理”相对 `00c49d8` 的变化 | 本次处理 |
|---|---|---|
| 电机、编码器、麦轮解算、轮速 PID | `Motor.c/.h`、`PID.c/.h`、`PID_config.c/.h` 与基线一致 | 保留现有引脚、UL/UR/DL/DR 顺序、1024/512 线换算、2.3 减速比和 11 cm 轮径 |
| IMU 采样与状态机 | `imu_attitude.c/.h` 合并为 `imu.c/.h`；标定结构归入姿态模块 | 在 `imu.c` 发布同帧定位样本，包含姿态、实际 dt、原加速度和已扣陀螺零偏的角速度 |
| 姿态算法 | Mahony、Madgwick 及公共数学合入 `attitude.c/.h` | 保留该实现与编译期算法选择；未恢复旧的拆分文件 |
| WiFi/JustFloat/遥测 | 合并为 `wifispi.c/.h`，替代旧 WiFi、打包和遥测文件 | 保留合并结构、默认禁用和 SPI1 引脚冲突保护；修复快捷宏仍调用旧函数名的问题 |
| 主循环与 PIT | 主循环使用新初始化/服务接口，中断代码简化 | 保留整理结果；5 ms IMU 后接定位预测，10 ms 电机周期内接编码器修正和位置环 |
| 菜单 | 代码压缩整理，仍是原有七个文件夹 | 增加 Navigation 和 Position，菜单池扩为 96 项，恢复超过七行的滚动显示 |
| 定位融合、位置外环 | 上游未含这两层 | 移植 `191ddb9` 的对应模块及控制调度 |
| 主机测试 | 上游部分测试仍包含已删除的旧头文件 | 更新测试到新接口，加入定位/位置环测试和真实 `imu.c` 发布测试 |

`config.h` 及底层编码器驱动与对比基线一致。本次不改变减速比、轮径、速度 PID 参数、麦轮解算公式或 WiFi 开关。

## 接口适配

`imu_get_navigation_sample()` 只在 IMU 运行且已有成功帧时返回快照。每个成功采样后增加 sequence；无新硬件数据时不增加，定位层不会重复积分。IMU 初始化或重新标定时增加 generation，并清空旧快照；定位层发现 generation 变化后冻结旧坐标，位置环随即关闭 Run，等待停车 Zero 后重新启动。

`app_navigation.c` 使用新 `imu.h` 和新 getter；不恢复 `imu_attitude.h`。`Mymenu.c` 使用 `imu_request_recalibration()` 和 `wifispi.h`。新增模块已加入 Keil 工程，调用链为：

```text
PIT CH0，5 ms：imu_update_5ms → 发布新 IMU 帧 → app_navigation_imu_tick_5ms
PIT CH1，10 ms：encoder_get → 定位编码器修正 → 位置控制 → 麦轮逆解 → 轮速 PID → PWM
```

目标坐标为最近一次 Navigation/Zero 的固定坐标：X 前进、Y 左移、Yaw 逆时针为正。位置环支持同时给定 XY 和 Yaw，以当前融合航向将世界速度转为车体 Vx/Vy。默认最大速度 20 cm/s、最大转速 1 rad/s，带加速度限制；位置 2 cm、航向 3° 范围内低速稳定 0.2 s 后关闭 Run。定位失效、Zero 和模式切换都会中止位置模式运行，恢复定位不会自动续跑。

## 验证

运行 `tests/run_navigation_tests.ps1`：

- 用真实 `imu.c` 和 `attitude.c`，分别选择 Mahony/Madgwick，验证新帧、零偏扣除、同帧数据、generation、超时和 dt 故障。
- 定位融合覆盖四轮混合分辨率、前后/横移、圆弧、旋转、倾斜、安装角、航向跨圈、打滑和故障。
- 位置环覆盖固定坐标到车体坐标转换、角度环绕、限速/加速度、带滞后模型收敛与持续到达判定。
- 实际调度覆盖融合→解算→轮速 PID→PWM、车头转 90° 后朝同一目标右移、定位失效/Zero 停车、模式互斥及手动 Drive/PWM 回归。
- 菜单覆盖 240×320 显示边界、负数目标、全部 Position 参数滚动和模式切换。
- JustFloat 测试链接实际合并后的打包代码，仅包装传输入口；WiFi 禁用测试无需 SPI/GPIO 依赖，不启用 WiFi。

使用 `build.ps1` 构建 `nor_sdram_zf_dtcm`，本机 Arm Compiler 6.19 为 0 errors、0 warnings。尚未烧录和实车验证，实际落点精度仍受减速比、编码器比例、轮速 PID、IMU 漂移与打滑影响。

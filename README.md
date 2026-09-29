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
| CH1 | 10 ms | 编码器读取、四轮 PID 和 PWM 输出 |
| CH2 | 20 ms | 四键扫描 |

UDP 发送和 IMU 重新初始化在主循环执行，不阻塞上述中断。

## 菜单

IPS200 使用竖屏 240×320，按键沿用 ASC 的操作方式：

- KEY1：进入文件夹、选择参数；参数已选中时切换步进；
- KEY2：上移；参数已选中时增加；
- KEY3：返回；参数已选中时取消选择；
- KEY4：下移；参数已选中时减小。

菜单包含 Drive、Encoder、IMU、Sensor、WiFi 和 PID。电机默认禁止输出，必须在 Drive/Run 中手动开启。

## IMU

IMU660RA 使用 SPI4：C23/C22/C21/C20。上电后须保持静止，连续取得 400 个合格样本后进入运行态。状态值：

- `0`：标定中；`1`：正常运行；
- `-1`：初始化失败；`-2`：采样超时；
- `-3`：采样间隔异常；`-4`：融合计算异常。

在 IMU/Recal 中置 On 可重新标定。算法由 `project/code/config.h` 的 `AHRS_METHOD` 选择。

## WiFi-SPI

WiFi 模块使用逐飞 WiFi-SPI2.0 驱动，UDP 每 20 ms 发送 12 字节小端 `float32`：roll、pitch、yaw（单位：度）。热点、密码、目标 IP 和端口配置在 `project/code/config.h`。WiFi 初始化失败不会停止 IMU 与电机控制。

## 构建

用 Keil MDK 打开 `project/mdk/rt1064.uvprojx`，构建目标 `nor_sdram_zf_dtcm`。已使用 Arm Compiler 6.19 验证：0 errors，0 warnings。

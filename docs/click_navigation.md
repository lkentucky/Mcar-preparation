# 地图点选导航

本功能在 `MCARpreparation` 项目内实现。上位机源码位于 `tools/wifi_spi_udp_gui`，固件改动位于 `project/code/wifispi.c`、`app_control.c` 和 `app_control.h`。

## 使用

1. 使用本次构建的固件更新车端。Keil 工程生成 `project/mdk/Objects/rt1064.axf`；导出的 HEX 为 `C:\Users\csz18\Desktop\imu\MCAR_ClickNav.hex`。
2. 车端完成 IMU 初始化、定位校准，并在需要的起点执行 Zero。导航目标始终采用此固定坐标系，单位 cm：初始车头前方 +X，左侧 +Y，逆时针为正航向。
3. 运行 `C:\Users\csz18\Desktop\imu\WiFiSPI_UDP_MCAR_ClickNav.exe`。调参页填写本机监听端口、模块 IP 和模块命令端口，开始监听。
4. 打开「04 路线图」，点击「连接点选导航 / 订阅位置回传」。收到有效车位后，保持「单击地图立即前往」勾选，单击图内目标位置。拖动距离达到 5 像素视为拖图，不发送目标。
5. 默认保持点击时的当前航向；取消勾选后使用输入的目标航向。也可以输入 X/Y，点击「前往输入目标」。途中点击新目标会替换旧目标。
6. 页面显示目标、实际车位、车头方向、剩余距离、Run 和位置控制状态。点击「停止小车」发送停止指令并等待回执。取消「自动适应」后可手动缩放、拖动查看地图。

导航地图直接使用固件 `x_cm/y_cm`，不使用 PC 轮速积分，也不在 PC 端平移或旋转坐标。「清空」仅清除显示轨迹，不改变车端 Zero。CSV 回放和轮速重建模式不允许下发点选目标。

这是调用现有位置控制器的单目标直达功能，没有障碍物地图、绕障或全局路径规划。位置回传超过 0.5 秒未更新时，上位机禁止发送新目标；已经接受的目标仍由车端自主执行。UDP 停止只有收到设备回执才能确认到达设备。

## 下发协议

ASCII UDP，通过已有绑定的监听 socket 发送，每条指令以 LF 换行结束；回执发往固件配置的 PC 接收地址。没有新增端口。

```text
GOTO <uint32请求号> <X_cm> <Y_cm> <yaw_deg>\n
STOP <uint32请求号>\n
```

例如发往 X=50cm、Y=-25cm、航向 35°：

```text
GOTO 100 50.000 -25.000 35.000
```

对应回执：

```text
MCAR NAV 100 50 -25 35
MCAR STOP 101
MCAR NAVERR 102 pose_invalid
```

回执 NAV 仅表示设备接收目标；到达必须由后续位置回传中的 `pos_status=3` 判断。其他错误原因包括 `invalid_id`、`invalid_goal`、`goal_out_of_range`、`invalid_stop`、`stale_request`。

X/Y 有限值范围 ±10000 cm，航向有限值范围 ±180°。固件先验证整条命令，再在中断保护下取消菜单多点路线，安装完整目标，切入位置模式；控制中断完成模式切换后启动 Run。STOP 清除待启动目标、停止 Run 并清零电机输出。

上位机每秒重发同一请求，最多发送三次；同一请求号和目标的重复包只回执，不重新启动。固件采用 32 位序号的串行顺序比较，拒绝旧 GOTO，同号不同目标也拒绝。STOP 总会停车，新 STOP 的序号同时拦截滞后的旧 GOTO；序号由单个上位机统一递增，不支持多个控制端并发控制。确认超时表示执行状态未知，页面不会当作成功。

## 回传订阅

点击连接导航后，自动按已有确认流程发送：

```text
STREAM 0
SUB x_cm,y_cm,nav_yaw_deg,goal_x_cm,goal_y_cm,goal_yaw_deg,distance_cm,yaw_error_deg,pos_status,nav_valid,run,position_enabled
RATE 50
STREAM 1
```

每一步等待对应 `MCAR` 回执。遥测沿用 JustFloat：按上述顺序发送 12 个小端 float32，再追加 `00 00 80 7F`，总长度 52 字节，周期 50ms。

| 字段 | 含义 |
| --- | --- |
| x_cm / y_cm / nav_yaw_deg | 实际车位和相对 Zero 航向 |
| goal_x_cm / goal_y_cm / goal_yaw_deg | 车端正在使用的目标 |
| distance_cm / yaw_error_deg | 距离及航向误差 |
| pos_status | -3 航向停滞，-2 参数错误，-1 定位无效，0 待机，1 行驶，2 稳定中，3 到达 |
| nav_valid | 1 定位有效，0 无效 |
| run / position_enabled | Run 开关和位置模式开关，0/1 |

## 验证与构建

- Keil 编译：0 错误、0 警告。
- `tests/run_click_navigation_tests.ps1`：实际命令服务验证回执、重复/乱序、STOP、非法目标及无效定位；实际电机控制代码验证模式切换、换目标、停止及取消待启动请求，硬件 I/O 使用桩函数。
- `tools/wifi_spi_udp_gui/test_click_navigation.py`：真实窗口和本机 UDP，验证订阅、绝对坐标、点选处理、目标确认、到达、超时数据拦截、重发、停止及 CSV 模式拦截。测试环境无法注入原生鼠标输入，因此鼠标坐标/悬停通过桩传入点击处理函数。
- 原上位机 18 项单元测试通过；路线图导入和积分回归通过。
- 原生完整回归脚本仍在既有 `deadzone_slider_test.c:21` 失败；使用本次修改前的 wifispi.c 也能复现该断言，本次没有修改死区参数逻辑。
- 尚未烧录或进行实车运动验证。

在上位机源码目录使用 PyInstaller 构建 `WiFiSPI_UDP_MCAR_ClickNav.spec`。原有 EXE 保留。固件修改前备份位于 `C:\Users\csz18\Desktop\imu\click_navigation_backup_20261017_160118`。

# 逐飞 WiFi-SPI2.0 UDP 麦轮遥测上位机

从 `C:/Users/csz18/Desktop/imu/wifi_spi_udp_gui` 现有源码适配，保留 UDP 二进制数据监视、绘图、CSV 和模型姿态显示。新增设备上传变量订阅、周期调整、变量查询与自动解析映射。原上位机文件保持原样。

推荐使用 Python 3.11 与已固定版本的 DearPyGui 1.11.1。该组合也是 Windows EXE 的构建环境。
程序启动时从 Windows 字体目录加载中文字体，避免 DearPyGui 默认字体把汉字显示为问号。

## 安装与启动

在本目录执行：

```bash
python -m pip install -r requirements.txt
python app.py
```

Windows 用户可双击本目录的 `dist/WiFiSPI_UDP_MCAR.exe`，无需另行启动 Python。

也可以直接安装唯一的第三方依赖：

```bash
python -m pip install dearpygui==1.11.1
```

## 使用步骤

麦轮固件详细操作和协议见 [WiFi 遥测协议](../../docs/wifi-telemetry-protocol.md)。开始监听后，点击“模块地址取最近来源”，选“定位预设”并点击“应用上传配置”。设备逐步确认后，解析表会自动更新。自定义上传顺序最多 40 项，也可“查询变量”并从下拉列表添加。默认 JustFloat 校验开启；接收旧的纯 12 字节 IMU 包时可关闭。

左侧有三个独立工作区，切换页面不会停止后台收包和 CSV 记录：

1. **调参**：默认监听 `0.0.0.0:8081`，点击“开始监听”。如 Windows 弹出防火墙提示，请允许本程序接收专用网络 UDP 数据。RT1064 固件的目标 IP 要填写电脑实际局域网地址，不能填 `0.0.0.0`。本页还能编辑二进制解析规则、发送文本/十六进制 UDP 指令、导出 CSV 和查看日志。
2. **实时绘图**：左侧显示所有解析字段的实时值，勾选任意字段即可绘制曲线。每字段最多保留 100～5000 个采样点，曲线画面最多刷新 15 次/秒。
3. **IMU 姿态**：用 `roll`、`pitch`、`yaw` 三个字段（角度单位：度）驱动导入的 OBJ 飞机模型，并单独显示三轴角度。可选择等轴、正视、俯视视角、缩放比例以及“精细/流畅”网格。默认规则为 3 个小端 float32，起始字节依次是 0、4、8。

解析规则中的“起始字节”从 0 开始；逐飞模块通常使用 `little`（小端）。支持 float32、float64、int8/16/32/64、uint8/16/32/64。“保存全部解析数据”导出时间戳、来源 IP/端口和已解析字段。

为减少界面开销，隐藏页面不刷新曲线或 3D 画面，高频接收日志做限频。实际性能是否优于 VOFA+ 取决于电脑、字段数和报文频率，尚未做同机基准对比。

Windows 版本会在创建窗口前启用逐显示器 DPI 感知，并按屏幕缩放比例重新栅格化中文字体与主要布局尺寸，避免系统将低分辨率画面整体放大造成模糊。

## 滑杆单步与自动存档

点击要调节的滑杆后，可以继续鼠标拖动，或按键盘 ↑ 加一个步长、↓ 减一个步长。每次按下只调节一次；长按不会连续重复发送。步长可编辑，最小为 `0.001`，修改后点击“应用”。到达范围边界不再调节。点击输入框或切换页面后不会误调之前的滑杆。

拖动和上下键都沿用 `[slider,参数名,数值]` 协议。先开始监听，填写实际模块 IP 和模块端口；“已发送”表示本机已发送 UDP，不等于设备已经应用。MCAR 固件确认后会显示 `设备已确认：pos_xy_kp 2.501`。界面初始值与存档值不是从单片机读取的。

工作区变更后每秒自动检查并保存，正常关闭时再保存一次，也可点击“保存工作区”。保存滑杆的增删、名字、最小/最大值、步长、当前值，以及网络地址、解析字段、曲线选择和姿态视角；不是 CSV 遥测记录，也不是单片机 Flash 存档。重新启动只恢复界面，不自动监听、不自动发送参数。

EXE 的存档位于它旁边：`WiFiSPI_UDP_MCAR.workspace.json`；源码运行则在 `app.py` 旁。请保持所在目录可写。主文件使用原子替换，上一份有效存档保留为 `.json.bak`，主文件损坏时自动尝试备份。删除全部滑杆后退出，再次打开也保留空列表。

新增模块：`slider_workspace.py`（滑杆/按键/UI 恢复）、`workspace_state.py`（存档及备份）、`slider_protocol.py`（十进制步长和报文编码）。`test_workspace_state.py` 验证磁盘存档，`test_slider_gui.py` 验证真实 Windows 按键、UDP 回环和关闭后恢复。测试不连接实车。

## 工程结构

- `app.py`：GUI、配置编辑、绘图、日志和 CSV 导出。
- `udp_receiver.py`：独立后台 UDP 接收线程，只向线程安全队列投递原始数据，不会阻塞 GUI。
- `mcar_telemetry.py`：设备订阅协议、帧校验、配置确认及超时重试状态机。
- `packet_parser.py`：独立的 `parse_packet(packet, rules)` 二进制解析函数。
- `aircraft_model.py`：导入 OBJ 网格并预计算面中心和法线；`aircraft_high.obj` / `aircraft_low.obj` 为两档模型资源。
- `attitude_3d.py`：使用 DearPyGui DrawList 与 ZYX 欧拉角绘制飞机网格、参考网格和方向罗盘，无额外 3D 依赖。

## 验证与打包

```powershell
python -m unittest test_mcar_telemetry.py
python app.py --self-test
python -m PyInstaller --noconfirm --onefile --windowed --noupx --name WiFiSPI_UDP_MCAR --add-data "aircraft_high.obj;." --add-data "aircraft_low.obj;." app.py
```

自测不向实车发送命令：模拟暂停→SUB→RATE→恢复应答、自动解析字段及二进制定位数据，同时实际渲染三页与模型。上位机与固件的 WiFi 实物互通需烧录后验证。
- `MODEL_LICENSE.md`：第三方飞机模型的来源与 CC0 授权说明。

> UDP 是无连接协议；“网络状态”表示本机端口是否成功绑定。设备掉线时 GUI 仍会保持正常，接收帧率会降为 0。

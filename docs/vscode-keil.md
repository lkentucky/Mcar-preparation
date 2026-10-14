# VS Code 与 Keil 配置同步

在 VS Code 选择“文件 → 从文件打开工作区”，打开项目根目录的 `MCARpreparation.code-workspace`。

配置对应 `project/mdk/rt1064.uvprojx` 的 `nor_sdram_zf_dtcm`：Arm Compiler、Cortex-M7、双精度 FPU、C99，以及 Keil 的头文件目录和预处理宏。`.vscode/compile_commands.json` 提供每个固件源码文件的实际编译配置，`.vscode/c_cpp_properties.json` 提供头文件和未列入数据库文件的默认配置。

切换分支、添加源码或修改 Keil 头文件路径后，在“终端 → 运行任务”执行 **Sync VS Code from Keil**。也可以运行：

```powershell
python tools/sync_vscode_keil.py
```

**Check firmware with Arm compiler** 对 Keil 目标源码做语法检查。**Ctrl+Shift+B** 调用实际 Keil 编译。两个操作都不烧录设备。

若 VS Code 仍保留旧的错误提示，执行命令面板的 **C/C++: Reset IntelliSense Database**，然后 **Developer: Reload Window**。

该配置针对 Keil 中的固件；`tests` 下的主机模拟测试属于另一套编译环境。配置没有通过关闭错误波浪线来隐藏代码错误，也没有改动固件源码。

编译器默认安装位置为 `C:/Keil_v5/ARM/ARMCLANG/bin/armclang.exe`。安装位置不同时使用 `--compiler` 指定实际路径。生成脚本会检查目录和源码是否存在，并拒绝忽略未来新增的文件/分组编译器覆盖选项。

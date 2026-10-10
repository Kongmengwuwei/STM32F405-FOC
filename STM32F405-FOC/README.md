# STM32F405 有感 FOC 工程导航

当前默认为 **M1 / L4225-6S420-A / TLE5012B / 安装 4 / FREE_SHAFT / WARN**，12 V 空载。SPI3/PA0 接线不变。唯一共用源码在 `variants/m1-mt6835/`；目录名沿用参考工程历史，接口、编码器、电机和负载可独立选择。

## 构建与测试

```powershell
cmake --preset Debug
cmake --build --preset Debug
python variants/m1-mt6835/tests/run_tests.py --cc gcc
```

需安装 CMake、Ninja、ARM GCC；主机测试另需可在本机执行的 GCC、Python、numpy 和 pyserial。本机可用 `Debug-local`。默认产物为 `build/longqiu/Debug/STM32F405-FOC.elf/.hex/.bin`。顶层和独立固件入口共用 `cmake/foc-defaults.cmake` 及工具链实现；显式 `-D` 和预设覆盖默认值，主机回归会检查预设与默认值一致。

## 阅读入口

| 目的 | 入口 |
| --- | --- |
| 了解调用链和模块职责 | [架构](docs/architecture.md)、[应用层](variants/m1-mt6835/App/README.md) |
| 学习公式与逐文件阅读 | [初学者教程](variants/m1-mt6835/docs/FOC-从零读懂这个工程.md)、[文件导览](variants/m1-mt6835/docs/文件导览.md) |
| 选择接口、编码器、电机和负载 | [统一固件指南](docs/unified-firmware.md)、[接线映射](docs/hardware-map.md) |
| 编译、烧录和调试 | [VS Code 操作](docs/vscode-build.md) |
| 使用 VOFA | [龙邱配置](docs/vofa-longqiu.md)、[通道表](docs/vofa-quickstart.md) |
| 自动适配电机 | [极对数与磁编扫描](docs/motor-sensor-autocal.md)、[R/L 与电流环整定](docs/motor-autotune.md)、[改进与实测](docs/longqiu-autotune-improvement-2026-10-10.md) |
| 改善低速波动 | [抗齿槽方案、命令与实测](docs/anticogging.md) |
| 查动态响应 | [龙邱动态响应](docs/longqiu-dynamic-response-2026-10-10.md) |
| 运行主机回归和实机工具 | [测试](variants/m1-mt6835/tests/README.md)、[台架工具](variants/m1-mt6835/tools/bench/README.md) |
| 查清理范围与后续优化 | [整理记录](docs/code-cleanup-2026-10-10.md) |

当前电流环 10 kHz、速度/位置外环 1 kHz、USB `send 6` 总览 2.5 kHz。VOFA 选 JustFloat、实际 COM 口、1000000、8N1、DTR。上电只进行静止电流零偏估计，加载有效 Flash 校准/电流参数；抗齿槽表恢复后保持关闭，需要 `cog on` 开启。首次新安装须校准和验收，日常运行结束发送 `stop`。WARN 下断开 USB 不会自动停机；抗齿槽采集和自动辨识仍有独立硬中止。

按日期保存的 ZH3620-1、塑料臂和旧安装记录仍可用于对照，不能代表当前电机的带载表现。HAL/CMSIS、USB 中间件和 CubeMX `.ioc` 保留；手写初始化与 `.ioc` 不完全等价。

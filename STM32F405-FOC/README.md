# STM32F405 有感 FOC

**本分支默认：M1 / L4225-6S420-A / 原 TLE5012B / 12 V / 安装 2 / FREE_SHAFT / WARN。** 原 SPI3/PA0 接线不变，电流环 10 kHz、USB 总览 2.5 kHz；Debug 输出位于 `build/longqiu/Debug/`。首次小电流运行与重新校准见[龙邱首测说明](docs/longqiu-first-test.md)。以下旧组合描述与负载性能记录保留为历史参考，新电机已完成校准保存、软件复位复用和空载短测，见[龙邱实测记录](docs/longqiu-bring-up-2026-10-09.md)；已继续完成速度和小角度定位短测，见[控制调试](docs/longqiu-control-tuning-2026-10-09.md)；2026-10-10 连续反向与定位提速见[动态响应记录](docs/longqiu-dynamic-response-2026-10-10.md)。低速纹波和带载性能仍未验收。

本工程的默认实物组合是 **M0 功率接口 + TLE5012B 编码器 + ZH3620-1 电机 + 塑料臂负载**。M0/M1 只是板上的功率接口，不是电机型号。固件源码在 `variants/m1-mt6835/`；这个目录名沿用参考工程历史，当前所有组合共用其中一套控制程序。

当前还支持停机 R/L 自动辨识、运行时电流 PI、Flash 保存/复位加载和实时母线 PWM 换算，已在 12 V 空载验证，见[电机自动适配方案](docs/motor-autotune.md)。

## 从哪里开始

| 想做什么 | 阅读入口 |
|---|---|
| 了解 FOC 原理和代码调用链 | [初学者教程](variants/m1-mt6835/docs/FOC-从零读懂这个工程.md)、[逐文件导览](variants/m1-mt6835/docs/文件导览.md)、[当前架构](docs/architecture.md) |
| 选择接口、编码器、电机、负载并校准 | [统一固件指南](docs/unified-firmware.md) |
| 在 VS Code 编译、烧录、调试 | [VS Code 操作](docs/vscode-build.md) |
| 连接 VOFA，看目标和实际电流、速度、位置 | [当前龙邱单电机配置](docs/vofa-longqiu.md)、[VOFA 操作与通道表](docs/vofa-quickstart.md) |
| 查引脚和实物接线 | [硬件映射](docs/hardware-map.md) |
| 理解告警和停机条件 | [台架告警模式](docs/bench-warning-mode.md) |
| 理解采样点、电流环和双 ADC 的选择 | [电流采样与调参](docs/current-sampling-noise.md) |
| 对照塑料臂与裸轴表现 | [负载对比](docs/free-shaft-vs-plastic-arm-2026-09-28.md) |
| 查最新带臂定位提速结果和改进顺序 | [性能记录](docs/position-speedup-and-foc-roadmap-2026-09-28.md) |

默认电流环 10 kHz，速度和位置外环 1 kHz；原生 USB CDC 默认 `send 6`，传输 24 个数值通道，M0 为 2.5 kHz。VOFA 选 JustFloat、当前 COM 口、1000000、8N1 并开启 DTR。首次运行先在轴可自由转动时发送 `cal`；运行结束发送 `stop`。`WARN` 模式下单次目标持续有效，断开 USB 不会停机。当前实机只验证了约 12 V、该塑料臂的短时工况，电流绝对比例和长期温升尚未独立验收。

项目根目录的 CMake 默认配置与 VS Code 默认任务均对应上述组合；其他组合须单独配置、核对接线和重新校准。`variants/m1-mt6835/tests/` 是当前主机回归测试，`variants/m1-mt6835/tools/bench/` 是采集和分析工具。STM32 HAL/CMSIS 与 CubeMX 生成文件保留在固件目录内；不要直接把 `.ioc` 重生成视为对手写配置的等价更新。

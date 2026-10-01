# STM32F405 有感 FOC

当前默认构建**双轴云台位置控制**：M0 水平轴允许连续多圈，M1 竖直轴命令 ±85°、实际行程保护 ±90°。参数按轴独立，zero/clear 不扩大行程。M0 电机 A 端通路断开，保持停用；M1 正在分环实机调试。详见[控制指南](docs/gimbal-position.md)和[最新调试记录](docs/gimbal-tuning-2026-09-30.md)。以下旧台架说明仅供 `FOC_GIMBAL=OFF` 对照。

当前实测与失败记录以[最新调试记录](docs/gimbal-tuning-2026-09-30.md)为准；[首次报告](docs/gimbal-bench-2026-09-30.md)保留作历史记录。

顶层工程默认构建 **m0 与 m1 同时控制**：两路均按 ZH3620-1 + TLE5012B + 塑料臂负载的 M0 参数运行。`m0` 仍驱动板上 M0 三相接口及其电流采样，编码器改接 SPI1 的 PB3/PB4/PB5 和 PA1；`m1` 仍驱动板上 M1 三相接口及其电流采样，编码器改接原 SPI3 的 PC10/PC11/PC12 和 PA0。两路编码器与各自轴的归属已实测；带电测试仍有 M0 输出切换时 SPI1 丢帧和电流读数异常，以及 M1 校准回零告警，尚未通过双路同时运行验收，细节见[双路控制指南](docs/dual-motor.md)。固件源码在 `variants/m1-mt6835/`；目录名沿用参考工程历史。

## 从哪里开始

| 想做什么 | 阅读入口 |
|---|---|
| 同时连接并控制两路 | [双路控制指南](docs/dual-motor.md) |
| 了解 FOC 原理和代码调用链 | [初学者教程](variants/m1-mt6835/docs/FOC-从零读懂这个工程.md)、[逐文件导览](variants/m1-mt6835/docs/文件导览.md)、[当前架构](docs/architecture.md) |
| 选择接口、编码器、电机、负载并校准 | [统一固件指南](docs/unified-firmware.md) |
| 在 VS Code 编译、烧录、调试 | [VS Code 操作](docs/vscode-build.md) |
| 连接 VOFA，看目标和实际电流、速度、位置 | [VOFA 操作与通道表](docs/vofa-quickstart.md) |
| 查引脚和实物接线 | [硬件映射](docs/hardware-map.md) |
| 理解告警和停机条件 | [台架告警模式](docs/bench-warning-mode.md) |
| 理解采样点、电流环和双 ADC 的选择 | [电流采样与调参](docs/current-sampling-noise.md) |
| 对照塑料臂与裸轴表现 | [负载对比](docs/free-shaft-vs-plastic-arm-2026-09-28.md) |
| 查最新带臂定位提速结果和改进顺序 | [性能记录](docs/position-speedup-and-foc-roadmap-2026-09-28.md) |

双路默认电流环均为 10 kHz，速度和位置外环均为 1 kHz；原生 USB CDC 固定传输 M0 与 M1 各 12 个数值通道，帧率 1 kHz。VOFA 选 JustFloat、当前 COM 口、1000000、8N1 并开启 DTR。首次运行先在两轴可自由转动时分别发送 `m0 cal`、`m1 cal`；运行结束发送 `stop`。`WARN` 模式下单次目标持续有效，断开 USB 不会停机。既有单路 M0 已完成约 12 V、塑料臂的短时测试；双路固件的实测进度以双路指南为准。

项目根目录的 CMake 默认配置与 VS Code 默认任务均对应上述双路组合；原单路和其他组合须设 `-DFOC_DUAL=OFF` 并使用独立构建目录。`variants/m1-mt6835/tests/` 是主机回归测试，`variants/m1-mt6835/tools/bench/` 的旧脚本按单路协议运行。STM32 HAL/CMSIS 与 CubeMX 生成文件保留在固件目录内；不要直接把 `.ioc` 重生成视为对手写配置的等价更新。

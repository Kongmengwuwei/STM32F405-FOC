# STM32F405 有感 FOC

当前默认构建**双轴云台位置控制**：M0 水平轴允许连续多圈，M1 竖直轴命令 ±85°、实际行程保护 ±90°。参数按轴独立，zero/clear 不扩大行程。M0 电机 A 端通路断开，保持停用；M1 正在分环实机调试。详见[控制指南](STM32F405-FOC/docs/gimbal-position.md)和[最新调试记录](STM32F405-FOC/docs/gimbal-tuning-2026-09-30.md)。以下旧台架说明仅供 `FOC_GIMBAL=OFF` 对照。

固件共用一套控制、USB CDC、USART2、CAN1 驱动和遥测代码。构建时分别选择 **功率接口、编码器型号、电机型号、安装实例**。M0/M1 只表示板上的三相输出与电流采样接口，不表示电机。CAN1 目前只有收发驱动，没有电机控制命令协议。

| 选择项 | 当前可选值 | 默认值 |
|---|---|---|
| `FOC_PORT` | `M0`、`M1` | `M0` |
| `FOC_ENCODER` | `TLE5012B`、`MT6835` | `TLE5012B` |
| `FOC_MOTOR` | `ZH3620_1`、`REFERENCE_24V` | `ZH3620_1` |
| `FOC_INSTALLATION_ID` | 1–15，换磁铁安装或三相接线后递增 | `1` |
| `FOC_PROTECTION` | `WARN` 告警、`TRIP` 越限跳闸 | `WARN` |

默认构建对应 **M0 接口 + TLE5012B + ZH3620-1**，使用 `STM32F405-FOC/` 下的 `Debug` 预设，生成 `build/Debug/STM32F405-FOC.elf`。原参考项目的电机型号尚未确认，`REFERENCE_24V` 是临时名称，保留原参数供对照，不能把它当作 ZH3620-1。原参考组合可在 `variants/m1-mt6835/` 单独构建。

图片中的 ZH3620-1 是 14 极（7 极对）、12 槽、500 KV、7.4–12.4 V，20 V 为最高电压，12 A 为堵转电流。默认已改为 **WARN 台架告警模式**：取消 ±0.30 A 命令拒绝、200 ms 超时关断及电流/转速/母线越限跳闸，单次电流目标保持至新命令或 `stop`。配置值继续用于告警，物理 PWM 边界和有效采样条件保留；详见[台架告警模式说明](STM32F405-FOC/docs/bench-warning-mode.md)。没有匹配的有效校准记录时先发送 `cal`；既有 v3 记录仍按组合身份复用。

电机、编码器与接口的选择方法、通信命令见[统一固件指南](STM32F405-FOC/docs/unified-firmware.md)；在 VS Code 中操作请看[构建与烧录步骤](STM32F405-FOC/docs/vscode-build.md)；用 USB 和 VOFA+ 看数据请看[VOFA+ 快速上手](STM32F405-FOC/docs/vofa-quickstart.md)；实机开始前按[ZH3620-1 首测清单](STM32F405-FOC/docs/zh3620-1-first-test.md)检查。旧 M0 [上板记录](STM32F405-FOC/docs/m0-torque-bring-up.md)和[FOC 初学者教程](STM32F405-FOC/variants/m1-mt6835/docs/FOC-从零读懂这个工程.md)用于了解硬件与算法；新组合只通过构建及主机侧测试，尚未完成实机验收。

## 当前调试基线（2026-09-27）

默认 M0/TLE5012B/ZH3620-1、WARN、USB 1000000、5 kHz 遥测，电流环仍为 10 kHz。`send 5` 提供目标/实际 Iq、Id、输出电压和饱和比例；电流网格继续绑定 I2/I9。已完成小电流短测，详见[问题与修复报告](STM32F405-FOC/docs/current-debug-2026-09-27.md)和[逐步改良路线](STM32F405-FOC/docs/foc-improvement-roadmap.md)。

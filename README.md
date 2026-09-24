# STM32F405 有感 FOC

本仓库现在从**同一套应用和控制源码**构建两种板卡配置。默认是 M0 电机桥与 TLE5012B；可选的是 M1 电机桥与 MT6835。两者都包含 USB CDC、USART2、CAN1 驱动、转矩/速度/位置命令、四组 JustFloat 遥测、校准与故障关断。CAN1 目前只有收发驱动，两种配置都没有 CAN 控制命令协议。

| 配置 | 构建位置 | 输出 | 采样与保护 |
|---|---|---|---|
| **M0 / TLE5012B（默认）** | `STM32F405-FOC/` | `build/Debug/STM32F405-FOC.elf` | TIM1/ADC1 注入，10 kHz；8–14 V；相电流 0.8 A 保护。首次需用 `cal` 显式校准。 |
| M1 / MT6835 | `STM32F405-FOC/variants/m1-mt6835/` | `build/Debug/405_FOC.elf` | TIM8/双 ADC DMA，20 kHz；8–36 V；相电流 10 A 保护。保留原 M1 校准记录兼容。 |

默认 M0 用 VS Code 的 `Debug-local` 预设构建；“STM32: Build and flash via ST-Link”只烧录默认 M0。M1 在其子目录使用 `Debug` 或 `Release` 预设单独构建。两个固件不能互换烧录后继续使用另一套电机接线。

**当前验证范围：**两套固件均已交叉编译；共同的命令与遥测做了主机测试。新 M0 固件的 USB 枚举、ADC/SPI 中断耗时、栅极波形以及带电机闭环尚未上板验证，不能把编译成功视为可以直接带负载运行。M0 极对数与电流增益仍按现有资料暂定。

从[统一架构与扩展指南](STM32F405-FOC/docs/unified-firmware.md)查看源码边界、添加编码器/电机的方法、首次接线和 VOFA+ 设置。原始 M0 的[上板记录](STM32F405-FOC/docs/m0-torque-bring-up.md)及参考 M1 的[FOC 初学者教程](STM32F405-FOC/variants/m1-mt6835/docs/FOC-从零读懂这个工程.md)仍可用于理解硬件与算法；这些历史记录不能证明新组合已通过实机验收。

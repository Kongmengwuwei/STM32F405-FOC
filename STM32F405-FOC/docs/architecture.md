# 当前 FOC 架构

当前可执行固件使用 `variants/m1-mt6835/` 内的同一套 `App/`、`Core/`、USB 中间件和 STM32 驱动。顶层 CMake 选择 `FOC_BOARD=M0`；在子工程单独构建时默认选择 `FOC_BOARD=M1`。顶层旧 `App/` 和 `Core/` 是上一轮 M0 上板试验的源码，**不在当前构建中**；其测试记录用于对照，不能与当前固件混用。

共用层包括 `App/Control/app.c`（命令、状态与遥测）、`control.c`（速度/位置外环与主机看门狗）、`App/FOC/foc.c`（校零、对齐、电流环、SVPWM 与故障状态）、USB CDC、UART 和 CAN 驱动。板卡差异只在 `App/Config/foc_profile.h`、`bsp_motor.c` 的引脚/定时器分支、选定的 ADC 和编码器驱动：

| 配置 | PWM/采样 | 编码器 | 采样回调 |
|---|---|---|---|
| M0 | TIM1 10 kHz，PC0/PC1 ADC1 注入，PA6 ADC2 母线 | SPI3 模式 1、16 位 TLE5012B，PA0 CS；安全字 CRC/状态校验 | ADC1 注入完成 IRQ 读取编码器与母线后调用同一个 `app_sample()` |
| M1 | TIM8 20 kHz，PC3/PC2 双 ADC DMA，PA6 母线 | SPI3 模式 3、8 位 MT6835，PA0 CS；DMA 与 CRC | ADC DMA 与 SPI DMA 完成后调用同一个 `app_sample()` |

数据流：`PWM 固定相位 → ADC 采样 → 编码器检查 → foc_step → PWM 预装载 → USB/UART 遥测`。命令从 USB CDC 或 USART2 到 `app_command()`，切换转矩/速度/位置模式。速度、位置命令超过 200 ms 未续发会故障停机；`stop` 立即关断，`clear` 只清除已经消失的故障。CAN1 目前没有应用层命令。

编译时由 `foc_profile.h` 指定极对数、电压与电流限制、采样率、对齐电压和环路参数。Flash 校准记录含板卡编号和极对数，避免把另一套配置的电角度零点用于当前电机；原 M1 v1 记录继续可读。M0 初次无校准记录时保持待机，必须显式发送 `cal`。M1 保持原先的首次自动对齐行为。

具体扩展步骤、使用方法和验证边界见 [统一固件指南](unified-firmware.md)。

# STM32F405 有感 FOC

固件共用一套控制、USB CDC、USART2、CAN1 驱动和遥测代码。构建时分别选择 **功率接口、编码器型号、电机型号、安装实例**。M0/M1 只表示板上的三相输出与电流采样接口，不表示电机。CAN1 目前只有收发驱动，没有电机控制命令协议。

| 选择项 | 当前可选值 | 默认值 |
|---|---|---|
| `FOC_PORT` | `M0`、`M1` | `M0` |
| `FOC_ENCODER` | `TLE5012B`、`MT6835` | `TLE5012B` |
| `FOC_MOTOR` | `ZH3620_1`、`REFERENCE_24V` | `ZH3620_1` |
| `FOC_INSTALLATION_ID` | 1–15，换磁铁安装或三相接线后递增 | `1` |

默认构建对应 **M0 接口 + TLE5012B + ZH3620-1**，使用 `STM32F405-FOC/` 下的 `Debug` 预设，生成 `build/Debug/STM32F405-FOC.elf`。原参考项目的电机型号尚未确认，`REFERENCE_24V` 是临时名称，保留原参数供对照，不能把它当作 ZH3620-1。原参考组合可在 `variants/m1-mt6835/` 单独构建。

图片中的 ZH3620-1 是 14 极（7 极对）、12 槽、500 KV、7.4–12.4 V，20 V 为最高电压，12 A 为堵转电流。默认固件仍采用首次测试的保守限值：**8–12.4 V 母线、±0.30 A 目标电流、0.80 A 相电流跳闸、±100 RPM**。堵转电流不是软件目标电流。所有组合首次上电均需显式发送 `cal`，旧版 Flash 校准记录不会被自动沿用。

电机、编码器与接口的选择方法、通信命令见[统一固件指南](STM32F405-FOC/docs/unified-firmware.md)；在 VS Code 中操作请看[构建与烧录步骤](STM32F405-FOC/docs/vscode-build.md)；用 USB 和 VOFA+ 看数据请看[VOFA+ 快速上手](STM32F405-FOC/docs/vofa-quickstart.md)；实机开始前按[ZH3620-1 首测清单](STM32F405-FOC/docs/zh3620-1-first-test.md)检查。旧 M0 [上板记录](STM32F405-FOC/docs/m0-torque-bring-up.md)和[FOC 初学者教程](STM32F405-FOC/variants/m1-mt6835/docs/FOC-从零读懂这个工程.md)用于了解硬件与算法；新组合只通过构建及主机侧测试，尚未完成实机验收。

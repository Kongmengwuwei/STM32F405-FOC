# STM32F405 有感 FOC 工程

本仓库保留两套独立固件。**默认方案是 Motor 0 + TLE5012B**，源码位于 [`STM32F405-FOC/`](STM32F405-FOC/)；原来的 M0 工程已恢复。参考项目的 **Motor 1 + MT6835** 电流环、速度环和位置环保留在 [`variants/m1-mt6835/`](STM32F405-FOC/variants/m1-mt6835/)，需要时单独构建。两套固件使用不同的引脚、编码器协议和电机参数，编译结果不能混用。

M0 保留其原有控制实现与上板状态；M1 的速度、位置和 USB 日志功能没有自动移植到 M0。

| 方案 | 构建目录 | 固件文件 | 说明 |
|---|---|---|---|
| **M0 / TLE5012B（默认）** | `STM32F405-FOC/` | `build/Debug/STM32F405-FOC.elf` | 本工作区的 CMake 和 VS Code 默认打开此方案；当前 M0 上板状态见[调试记录](STM32F405-FOC/docs/m0-torque-bring-up.md)。 |
| M1 / MT6835（可选） | `STM32F405-FOC/variants/m1-mt6835/` | `build/Debug/405_FOC.elf` | 参考项目 `F405_FOC_Fork` 的实现；入口见[方案说明](STM32F405-FOC/variants/m1-mt6835/README.md)。 |

## 构建默认 M0 方案

安装 CMake、Ninja 与 `arm-none-eabi` 工具链，并确保它们在 `PATH` 中，然后在默认固件目录执行：

```sh
cd STM32F405-FOC
cmake --preset Debug
cmake --build --preset Debug
```

当前电脑还保留本机专用的 `Debug-local` 预设和 VS Code 任务，它们现在也指向 **M0**。生成的 ELF、HEX、BIN 在 `STM32F405-FOC/build/Debug/`。切换到 M1 时请在其子工程目录配置、构建，输出留在该子工程自己的 `build/`；不要用默认烧录任务发送 M1 固件。

## 阅读资料

- 默认 M0 的[架构说明](STM32F405-FOC/docs/architecture.md)、[硬件映射](STM32F405-FOC/docs/hardware-map.md)和[上板记录](STM32F405-FOC/docs/m0-torque-bring-up.md)。
- 可选 M1 的[FOC 初学者教程](STM32F405-FOC/variants/m1-mt6835/docs/FOC-从零读懂这个工程.md)与[逐文件导览](STM32F405-FOC/variants/m1-mt6835/docs/文件导览.md)。

两套方案都已在电脑上编译验证；上板前应按所选方案核对 Motor 端口、编码器接线、电流采样和保护参数。

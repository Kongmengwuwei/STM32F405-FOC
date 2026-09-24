# VS Code 构建与烧录

工作区 `D:\STM32F405-FOC` 的 `Debug-local` 预设和 `STM32: Build and flash via ST-Link` 任务均针对**默认 M0/TLE5012B**。选 `Debug-local`，执行 CMake Configure/Build；生成的 `STM32F405-FOC/build/Debug/STM32F405-FOC.elf` 由烧录任务交给 ST-Link。USB、UART、CAN 和三种控制模式已经进入此默认固件。

可选 M1/MT6835 在 `variants/m1-mt6835/` 单独执行 `cmake --preset Debug`、`cmake --build --preset Debug`，输出 `build/Debug/405_FOC.elf`；默认 VS Code 烧录任务不适用于 M1。

当前新 M0 组合只通过编译与主机侧命令测试，尚未进行 USB 枚举、ADC/SPI 时序和电机输出实测。首次上板流程见 [统一固件指南](unified-firmware.md)。当前 M0 配置使用共享 Core/USB 项目中的 8 MHz HSE → 168 MHz SYSCLK/48 MHz USB；旧 M0 试验固件的 HSI 时钟与自动 60 rpm 流程已不在本次构建中。顶层旧 `Core/` 和 `App/` 保留作历史参考，不参与新固件编译。

两套配置仍有手工编写的板级差异，`.ioc` 尚未能完整表达它们。直接重新生成 CubeMX 源码可能覆盖条件编译和手工时序代码，须先同步 `.ioc` 与生成文件。

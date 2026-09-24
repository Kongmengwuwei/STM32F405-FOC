# VS Code 中构建固件

本机已经安装的 STM32CubeIDE GCC 13.3.1、STM32 扩展附带的 CMake 4.3.1 和 Ninja 1.13.2 已配置在 `CMakeUserPresets.json` 与工作区 `.vscode/settings.json` 中。`CMakePresets.json` 仍保留通用的 Debug/Release 配置。

1. 在 VS Code 打开 `D:\STM32F405-FOC`。
2. 在 CMake 扩展中选择配置预设 `Debug-local`。
3. 执行“CMake: Configure”，然后执行“CMake: Build”。以后修改代码只需重新 Build。
4. 输出位于 `STM32F405-FOC\build\Debug`，包括 `.elf`（调试）、`.hex` 和 `.bin`（烧录镜像）。

ST-Link 接好后，在 VS Code 的“终端 → 运行任务”选择 `STM32: Build and flash via ST-Link`。该任务先编译，再通过 STM32 扩展附带的 CubeProgrammer 烧录、回读验证并复位芯片。

命令行复现：

```powershell
cd D:\STM32F405-FOC\STM32F405-FOC
& 'C:\Users\kongmeng\AppData\Local\stm32cube\bundles\cmake\4.3.1+st.1\bin\cmake.exe' --preset Debug-local
& 'C:\Users\kongmeng\AppData\Local\stm32cube\bundles\cmake\4.3.1+st.1\bin\cmake.exe' --build --preset Debug-local
```

已验证 Debug 配置完整编译、经 ST-Link 烧录和回读校验成功。当前固件以 168 MHz 运行，M0 的 TLE5012B 使用 SPI 模式 1、16 位字、安全字校验和低 15 位角度；TIM1/ADC1 提供 10 kHz 同步电流环。上电先保持 M0 六路 PWM 关闭，随后自动尝试对齐和 60 rpm 速度环；任一校验或保护失败即锁存关断。**持续 60 rpm 尚未上板验收成功**，目前曾因电流达到 0.8 A 阈值停止，详见 `docs/m0-torque-bring-up.md`。

当前 `.ioc` 仍是 16 MHz 原始模板，而 `Core/Src/main.c` 与 `App/` 内有手写时钟及外设初始化；在将这些设置同步到 CubeMX 前，不要直接用 `.ioc` 重新生成覆盖当前固件。

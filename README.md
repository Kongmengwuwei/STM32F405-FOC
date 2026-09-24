# STM32F405 FOC

基于 STM32F405 的有感 FOC 实验工程。固件位于 [`STM32F405-FOC/`](STM32F405-FOC/)；仓库根目录是 VS Code 工作区。

## 目录

- `STM32F405-FOC/App/`：电机控制、编码器与板级接口代码。
- `STM32F405-FOC/Core/`、`Drivers/`、`cmake/`：启动、STM32 HAL/CMSIS 与构建配置。
- `STM32F405-FOC/tests/`：可在电脑上运行的 C 单元测试。
- `STM32F405-FOC/docs/`：硬件映射、架构和上板记录；`docs/schematic/` 保存原理图图片。

## 构建

安装 CMake、Ninja 与 `arm-none-eabi` 工具链，并确保它们在 `PATH` 中，然后从固件目录运行：

```sh
cmake --preset Debug
cmake --build --preset Debug
```

输出在 `STM32F405-FOC/build/Debug/`。本机专用的 `CMakeUserPresets.json`、`.vscode/` 设置和编译结果已排除在 Git 之外。现有本机 VS Code 操作说明见 [`vscode-build.md`](STM32F405-FOC/docs/vscode-build.md)。

## 上板状态

当前固件包含自动对齐和 60 rpm 速度环；持续运行尚未完成上板验收。接线、供电及调试状态见 [`m0-torque-bring-up.md`](STM32F405-FOC/docs/m0-torque-bring-up.md)。`.ioc` 仍是原始模板，重新生成代码前请先阅读 [`vscode-build.md`](STM32F405-FOC/docs/vscode-build.md) 中的说明。

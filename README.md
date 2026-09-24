# STM32F405 有感 FOC 工程

固件位于 [`STM32F405-FOC/`](STM32F405-FOC/)。本仓库已以 `F405_FOC_Fork` 的实现替换原有控制程序，参考版本为 `2ac684d`。控制对象是使用 **MT6835 磁编码器**、Motor 1 三相桥和两路相电流采样的 STM32F405 板卡；原有 M0/TLE5012B 程序可从 Git 历史中查看。

**从这里开始阅读：**[《从零读懂这个 FOC 工程》](STM32F405-FOC/docs/FOC-从零读懂这个工程.md)。文档解释控制原理、每个项目文件的作用、实时调用链、命令、保护、构建和验证。

## 构建与验证

固件目录中的 `CMakePresets.json` 提供 Debug 和 Release。将 CMake、Ninja、`arm-none-eabi` 工具链加入 `PATH`，在该目录执行：

```sh
cmake --preset Release
cmake --build --preset Release
```

输出为 `build/Release/405_FOC.elf`、`.hex` 和 `.bin`。主机测试命令见 [`tests/README.md`](STM32F405-FOC/tests/README.md)。本机设置、构建产物和 Python 缓存不纳入 Git。

## 硬件前提

当前代码按参考工程的具体板卡和电机编写：TIM8 驱动 M1，SPI3/PA0 连接 MT6835，B/C 相电流接 PC3/PC2，母线采样接 PA6，电机按 **7 极对**计算。`0.52°` 编码器误差补偿、电流增益、保护阈值和 Flash 校准记录也来自参考台架。**编译通过不代表本机板卡已完成接线核对或上板验收**；首次烧录前请先对照[硬件拓扑](STM32F405-FOC/硬件PCB拓扑.md)和教学文档的硬件章节。

# 共用固件源码与 M1 / MT6835 可选构建

此目录以参考项目 `F405_FOC_Fork`（`2ac684d`）为基础，现同时承载两种配置的**共用应用、FOC、USB/UART/CAN 代码**。从这里单独构建时默认选择 Motor 1 + MT6835、20 kHz；从[顶层默认工程](../..)构建时设置 `FOC_BOARD=M0`，选择 Motor 0 + TLE5012B、10 kHz。板级配置见 `App/Config/foc_profile.h`，使用与扩展步骤见[统一固件指南](../../docs/unified-firmware.md)。

在本目录安装好 CMake、Ninja、ARM GCC 后构建：

```sh
cmake --preset Debug
cmake --build --preset Debug
```

输出是 `build/Debug/405_FOC.elf`、`.hex` 和 `.bin`。Release 同理使用 `Release` 预设。主机测试见 [`tests/README.md`](tests/README.md)。从[初学者教程](docs/FOC-从零读懂这个工程.md)了解公式、实时调用链、状态机与保护，再用[逐文件导览](docs/文件导览.md)查每个文件的职责。

M1 方案依赖 SPI3/PA0 的 MT6835、PC3/PC2 的 B/C 相电流、PA6 母线采样、7 极对电机及对应的 TIM8 六路栅极接线。现有参考项目的板上记录不等于本轮新固件已重新验收。

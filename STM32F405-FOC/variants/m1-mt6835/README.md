# 共用固件源码与参考组合构建入口

此目录以参考项目 `F405_FOC_Fork`（`2ac684d`）为基础，现承载全部组合的**共用应用、FOC、USB/UART/CAN 代码**。从这里单独构建时默认选择 M1 功率接口、MT6835、原参考电机 `REFERENCE_24V`；该电机的准确型号待确认。从[顶层默认工程](../..)构建则默认选择 M0 接口、TLE5012B、ZH3620-1。四个独立选项见[统一固件指南](../../docs/unified-firmware.md)。

在本目录安装好 CMake、Ninja、ARM GCC 后构建：

```sh
cmake --preset Debug
cmake --build --preset Debug
```

输出是 `build/Debug/405_FOC.elf`、`.hex` 和 `.bin`。Release 同理使用 `Release` 预设。主机测试见 [`tests/README.md`](tests/README.md)。从[初学者教程](docs/FOC-从零读懂这个工程.md)了解公式、实时调用链、状态机与保护，再用[逐文件导览](docs/文件导览.md)查每个文件的职责。

M1 接口使用 TIM8、PC3/PC2 的 B/C 相电流与 PA6 母线采样；默认 MT6835 接 SPI3/PA0。换电机型号、编码器或功率接口后必须核对硬件连接并重新 `cal`。现有参考项目的板上记录不等于新组合已重新验收。

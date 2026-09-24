# M1 / MT6835 可选固件

这是参考项目 `F405_FOC_Fork`（`2ac684d`）的独立副本：Motor 1 三相桥、MT6835 磁编码器、20 kHz 电流环、1 kHz 速度/位置环。仓库的**默认固件**位于[上两级目录](../..)，使用 M0/TLE5012B。此处的 App、Core、Drivers、USB、测试和工具按 M1 方案组合，不能与默认方案交叉编译或烧录。

在本目录安装好 CMake、Ninja、ARM GCC 后构建：

```sh
cmake --preset Debug
cmake --build --preset Debug
```

输出是 `build/Debug/405_FOC.elf`、`.hex` 和 `.bin`。Release 同理使用 `Release` 预设。主机测试见 [`tests/README.md`](tests/README.md)。从[初学者教程](docs/FOC-从零读懂这个工程.md)了解公式、实时调用链、状态机与保护，再用[逐文件导览](docs/文件导览.md)查每个文件的职责。

M1 方案依赖 SPI3/PA0 的 MT6835、PC3/PC2 的 B/C 相电流、PA6 母线采样、7 极对电机及对应的 TIM8 六路栅极接线。参考项目的板上记录不等于当前硬件已重新验收。

# STM32F405 五连杆轮腿控制

当前硬件为两路龙邱L4225、TLE5012B和7∶1减速箱。固件、文档和测试均位于 [STM32F405-FOC/](STM32F405-FOC/README.md)。

进入该目录后使用 `cmake --preset Debug-dual-leg`、`cmake --build --preset Debug-dual-leg`。日常控制见 [VOFA轮腿操作](STM32F405-FOC/docs/wheel-leg-quickstart.md)，已通过的运动路径及数据见 [最近实测](STM32F405-FOC/docs/fast-reversal-fix-20261016.md)。

`exports/` 为VOFA生成包和本机备份，不再版本管理；源配置保留在 `STM32F405-FOC/docs/vofa/`。单电机兼容构建与自动整定工具仍保留。

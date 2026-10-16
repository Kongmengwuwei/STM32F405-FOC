# STM32F405 五连杆轮腿 FOC

当前实测配置：双路 M0/M1、龙邱 L4225-6S420-A、TLE5012B、7∶1减速、12 V。使用 **Debug-dual-leg**，关节范围0°～110°，电流指令上限10 A，快速档120电机RPM；保留180 RPM及ADC/PWM保护。

```powershell
cmake --preset Debug-dual-leg
cmake --build --preset Debug-dual-leg
python variants/m1-mt6835/tests/run_tests.py --cc gcc
```

需要 CMake、Ninja、ARM GCC；主机回归使用可在本机执行的 GCC、Python、numpy、pyserial。构建输出为 `build/dual-leg/Debug/STM32F405-FOC.elf/.bin/.hex`。本地工具链路径可写入不入库的 `CMakeUserPresets.json`。

| 目的 | 入口 |
|---|---|
| 上电、恢复位置、定点控制 | [VOFA日常操作](docs/wheel-leg-quickstart.md) |
| 接线、关节命令和诊断 | [双路接口](docs/dual-leg.md)、[硬件映射](docs/hardware-map.md) |
| 足端坐标、正逆解与标定 | [五连杆控制](docs/fivebar-foot-control.md) |
| 了解调度与代码职责 | [架构](docs/architecture.md)、[源码导览](variants/m1-mt6835/docs/文件导览.md) |
| 查看已通过的路径与响应 | [最近实测](docs/fast-reversal-fix-20261016.md) |
| 回归测试和实机工具 | [测试](variants/m1-mt6835/tests/README.md)、[工具](variants/m1-mt6835/tools/bench/README.md) |
| 单电机兼容配置 | [统一固件选项](docs/unified-firmware.md)、[单电机VOFA](docs/vofa-longqiu.md) |
| 自适配与抗齿槽 | [R/L整定](docs/motor-autotune.md)、[磁编扫描](docs/motor-sensor-autocal.md)、[抗齿槽](docs/anticogging.md) |

上电不自动转动。原零点附近0°～40°内静止、支撑好机构后，在VOFA确认恢复位置；不是将当前位置设为零。进入合法行程后即可控制。同次通电连续跟踪有效时，不需要重复恢复；停止输出后需支撑重力负载。

源码仍集中在 `variants/m1-mt6835/`，目录名沿用历史。默认 `Debug/Release` 继续提供单电机M1兼容构建，轮腿请选择上面的专用预设。CubeMX、HAL/CMSIS和USB源码保留；不要用 `.ioc` 覆盖手写双路初始化。VOFA生成包在仓库外层 `exports/`，可用 `python docs/vofa/configure_wheel_leg.py` 重新生成，不再重复入库。

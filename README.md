# STM32F405 有感 FOC：龙邱电机分支

当前组合为 **M1 / 龙邱 L4225-6S420-A / TLE5012B / 12 V 空载 / 安装 4 / WARN**。编码器仍使用 SPI3：PC10=SCK、PC11=MISO、PC12=MOSI、PA0=CS0。

程序支持电流、速度、位置控制，极对数/方向/电角零点扫描，R/L 辨识、电流 PI 自整定，以及 720 点抗齿槽转矩前馈。有效校准和整定参数从 Flash 加载；抗齿槽表启动时默认关闭，`cog on` 开启、`cog off` 关闭。日常结束发送 `stop`。

电流环 10 kHz、普通外环 1 kHz、24 通道 USB 总览 2.5 kHz。速度/位置输出硬限 ±6 A；WARN 的诊断阈值不等于停机阈值，单次运动目标会持续保持。详细参数以源码和 Flash 整定记录为准。

唯一共用固件在 `STM32F405-FOC/variants/m1-mt6835/`。该目录名是来源历史；M0、MT6835 和旧电机配置仍可通过构建选项选择。进入 `STM32F405-FOC/`：

```powershell
cmake --preset Debug
cmake --build --preset Debug
python variants/m1-mt6835/tests/run_tests.py --cc gcc
```

默认输出 `build/longqiu/Debug/STM32F405-FOC.elf`，另有 `.bin` 和 `.hex`；本机 `Debug-local` 使用已安装的工具链。配置默认值集中在 `cmake/foc-defaults.cmake`，本机编辑器配置不入 Git。

- [工程导航](STM32F405-FOC/README.md)、[架构与代码分工](STM32F405-FOC/docs/architecture.md)
- [自动整定](STM32F405-FOC/docs/motor-autotune.md)、[抗齿槽补偿与实测](STM32F405-FOC/docs/anticogging.md)
- [动态响应实测](STM32F405-FOC/docs/longqiu-dynamic-response-2026-10-10.md)、[VOFA 配置](STM32F405-FOC/docs/vofa-longqiu.md)
- [本次清理、验证和后续优化](STM32F405-FOC/docs/code-cleanup-2026-10-10.md)

历史测试按日期保留，其配置与结论只适用于对应安装和负载。当前低速补偿已经过 12 V 空载验收，带载和长期温升尚未验收。

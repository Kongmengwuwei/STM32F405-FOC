# 当前主机测试与采集工具

龙邱分支新增 `test_longqiu.c`：使用真实 `foc.c` 和 `control.c`，检查新电机校准身份、正反方向扫描、失败扫描、15° 起始角扣除、WARN/TRIP 电压饱和、外环限流、Flash 保存恢复采样和 stop。编译选择 `FOC_PORT_M1`、`FOC_ENCODER_TLE5012B`、`FOC_MOTOR_L4225_6S420_A`、`FOC_LOAD_FREE_SHAFT`、`FOC_INSTALLATION_ID=2`，分别设置 `FOC_PROTECTION_TRIP=0/1`；包含目录用 App/Config、App/FOC、App/Control、App/Hardware/bsp。硬件 ADC/SPI 时序仍需上板测试。

链接真实 `foc.c` 或 `app.c` 的测试需同时链接 `App/FOC/motor_ident.c` 和 `App/FOC/sensor_cal.c`；使用 `foc_stub.c` 的纯外环测试无需它们。自动适配证据见[方案与实测](../../../docs/motor-autotune.md)。

本目录只保留适用于现行电流环和 USB 协议的测试。主机测试使用真实的部分 `App/` 源码，硬件接口由夹具替代；测试通过不能证明真实 ADC 时序、栅极波形或绝对电流精度。固件默认参数见[应用层说明](../App/README.md)，实机短测证据见[定位提速](../../../docs/position-speedup-and-foc-roadmap-2026-09-28.md)。

| 文件 | 主要验证内容 |
|---|---|
| `test_control_tune_scale.c` | 外环增益不随电流 Kp 比值改变，磁编方向映射保持正确 |
| `test_current_acceptance.py` | 电流阶跃验收、噪声底和低信噪比、迟缓/偏置/振荡/丢帧/故障拒绝 |
| `test_sensor_cal.c` | 1～64 极对、正反方向、跨零、失步/卡轴/非整数/回程拒绝、运行时电角度和硬中止 |
| `test_motor_ident.c` | 独立离散 RL 对象、低电感、多电压/频率、逆变器偏置、双次不一致拒绝、R/L 与 PI、记录校验 |
| `test_foc_adaptive.c` | 真实 FOC 的运行时 PI、12/18/24 V 正反电流跟踪、默认/回退/重校准及故障锁存保护 |
| `test_m0_unified.c` | 默认 M0 组合的校准、命令、三种模式、总览遥测及 WARN 行为 |
| `test_app_usb.c`、`test_justfloat.c` | 命令解析、帧格式、旧组兼容与异常输入 |
| `test_longqiu_dynamic.c` | 龙邱真实外环的连续 ±300 rpm 反向、参考斜坡、计时回绕、校准方向、模式切换及 ±6 A 上限（实物响应另测） |
| `test_control_pid.c`、`test_control_observer.c`、`foc_stub.c` | 外环响应、模式切换、观察器和抗饱和 |
| `test_current_window.c`、`test_pwm_zero_offset.c` | M0/M1 采样窗口和 PWM 等占空比校零 |
| `test_foc_recalibration.c`、`test_motor_record.c`、`test_profile_matrix.c` | 校准流程、Flash 身份和配置矩阵 |
| `test_mt6835_crc.c`、`test_usb_queue.c`、`usb_stubs/` | 编码器帧校验、USB 队列/背压 |
| `test_current_probe.py`、`test_response_probe.py` | PC 采集切组、按设备时间计算运动响应 |
| `capture_usb.py` | 实机 USB 连续性记录；打开 COM 前先断开 VOFA |

本次还覆盖双向平均零点对齿槽偏置的消除、辨识定子坐标冻结、DC 整定期有界运动与正式采集期的严格运动拒绝。`test_foc_adaptive.c` 另覆盖全圆起始角选择最近 60° 均衡方向（调整不超过 30°），并验证 ±0.2 Id 的 q 注入不会使任一相跨零。

在 `variants/m1-mt6835/` 目录，先建 `build/`，再用主机 GCC 运行核心测试，例如：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -O2 -DFOC_PORT_M0 -DFOC_ENCODER_TLE5012B -DFOC_MOTOR_ZH3620_1 -DFOC_INSTALLATION_ID=1 -I App/Control -I App/FOC -I App/Protocols/JustFloat -I App/Hardware/bsp -I App/Hardware/encoder -I App/Config tests/test_m0_unified.c App/Control/app.c App/Control/control.c App/FOC/foc.c App/FOC/motor_ident.c App/FOC/sensor_cal.c -lm -o build/test_m0_unified.exe
./build/test_m0_unified.exe
python tests/test_current_probe.py
python tests/test_response_probe.py
python tools/bench/selftest.py
```

`test_m0_unified.c` 的 `FOC_LOAD_PLASTIC_ARM` 为默认参数；显式加 `-DFOC_LOAD_FREE_SHAFT` 可以检查裸轴配置。完整固件编译用项目根目录的 `cmake --build --preset Debug-local`。台架采集和响应分析命令见[bench 工具说明](../tools/bench/README.md)。

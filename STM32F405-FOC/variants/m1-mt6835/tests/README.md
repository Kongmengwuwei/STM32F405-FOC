# 当前主机测试与采集工具

本目录只保留适用于现行电流环和 USB 协议的测试。主机测试使用真实的部分 `App/` 源码，硬件接口由夹具替代；测试通过不能证明真实 ADC 时序、栅极波形或绝对电流精度。固件默认参数见[应用层说明](../App/README.md)，实机短测证据见[定位提速](../../../docs/position-speedup-and-foc-roadmap-2026-09-28.md)。

| 文件 | 主要验证内容 |
|---|---|
| `test_m0_unified.c` | 默认 M0 组合的校准、命令、三种模式、总览遥测及 WARN 行为 |
| `test_app_usb.c`、`test_justfloat.c` | 命令解析、帧格式、旧组兼容与异常输入 |
| `test_control_pid.c`、`test_control_observer.c`、`foc_stub.c` | 外环响应、模式切换、观察器和抗饱和 |
| `test_current_window.c`、`test_pwm_zero_offset.c` | M0/M1 采样窗口和 PWM 等占空比校零 |
| `test_foc_recalibration.c`、`test_motor_record.c`、`test_profile_matrix.c` | 校准流程、Flash 身份和配置矩阵 |
| `test_mt6835_crc.c`、`test_usb_queue.c`、`usb_stubs/` | 编码器帧校验、USB 队列/背压 |
| `test_current_probe.py`、`test_response_probe.py` | PC 采集切组、按设备时间计算运动响应 |
| `capture_usb.py` | 实机 USB 连续性记录；打开 COM 前先断开 VOFA |

在 `variants/m1-mt6835/` 目录，先建 `build/`，再用主机 GCC 运行核心测试，例如：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -O2 -DFOC_PORT_M0 -DFOC_ENCODER_TLE5012B -DFOC_MOTOR_ZH3620_1 -DFOC_INSTALLATION_ID=1 -I App/Control -I App/FOC -I App/Protocols/JustFloat -I App/Hardware/bsp -I App/Hardware/encoder -I App/Config tests/test_m0_unified.c App/Control/app.c App/Control/control.c App/FOC/foc.c -lm -o build/test_m0_unified.exe
./build/test_m0_unified.exe
python tests/test_current_probe.py
python tests/test_response_probe.py
python tools/bench/selftest.py
```

`test_m0_unified.c` 的 `FOC_LOAD_PLASTIC_ARM` 为默认参数；显式加 `-DFOC_LOAD_FREE_SHAFT` 可以检查裸轴配置。完整固件编译用项目根目录的 `cmake --build --preset Debug-local`。台架采集和响应分析命令见[bench 工具说明](../tools/bench/README.md)。

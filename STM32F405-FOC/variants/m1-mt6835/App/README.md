# 共用 FOC 应用层

这套 `App/` 源码服务所有功率接口、编码器和电机组合。顶层 CMake 默认选择 `M0 + TLE5012B + ZH3620_1 + PLASTIC_ARM + WARN`；M1/MT6835 和参考电机使用同一应用和控制算法，但单独选择驱动与参数。项目总入口见[根目录说明](../../../README.md)，各项配置和校准见[统一固件指南](../../../docs/unified-firmware.md)。

## 数据怎样走

1. `Core/Src/main.c` 初始化板级外设后调用 `app_init()`。M0 的 ADC 中断、M1 的 ADC DMA 回调在固定 PWM 相位取得 B/C 电流，随后读取编码器和母线。
2. `app_sample()` 把一次有效采样交给 `foc_step()`：由 B/C 算 A 相，经 Clarke/Park 得到 Id/Iq；双 PI 根据目标算 Ud/Uq，逆 Park 与 SVPWM 生成三相占空比。实际电压缩放会反馈给积分抗饱和。
3. `control_step()` 每 1 ms 更新速度/位置外环。速度模式由速度 PI 产生 Iq 目标；位置模式先按距离、速度上限与刹车距离产生速度目标，再由速度 PI 产生 Iq。电流模式直接使用用户的 Iq 目标。
4. 主循环解析 USB CDC/USART2 命令，处理校准 Flash 保存。USB 遥测默认 `send 6` 为 24 个普通 float，M0 2.5 kHz；控制采样仍 10 kHz。CAN1 有底层收发，但应用未定义 CAN 控制协议。

## 源码分工

| 目录/文件 | 职责 |
|---|---|
| `Config/foc_port.h` | M0/M1 功率接口、采样窗口与诊断阈值 |
| `Config/foc_encoder_profile.h` | TLE5012B/MT6835 型号选择 |
| `Config/foc_motor.h` | 电机资料、极对数、电流与外环参数；ZH3620-1 的带臂/裸轴独立档案 |
| `Config/foc_profile.h`、`foc_policy.h`、`foc_telemetry.h` | 组合身份、WARN/TRIP、遥测分频 |
| `Control/app.c`、`telemetry.h` | 命令、采样调度、USB/UART 遥测与告警发布 |
| `Control/control.c` | 模式切换、速度/位置外环与轨迹 |
| `FOC/foc.c`、`sine_table.inc` | 零偏、校准、坐标变换、电流 PI、SVPWM、状态机 |
| `Hardware/bsp/bsp_adc_m0.c`、`bsp_adc.c` | M0 顺序/可选同步采样、M1 双 ADC DMA |
| `Hardware/bsp/bsp_motor.c`、`bsp_motor_record.h` | PWM 提交与关断、Flash 校准记录 |
| `Hardware/encoder/`、`Hardware/tle5012b/`、`Hardware/mt6835/` | 编码器统一接口、芯片协议与 SPI 读数 |
| `Hardware/bsp/bsp_usb.c`、`bsp_uart.c`、`bsp_can.c` | 通信队列和物理接口 |
| `Protocols/JustFloat/justfloat.h` | 数值帧尾标记与封装 |

每个 `.h` 文件给其他模块提供类型、常量或函数声明；对应 `.c` 实现行为。完整的初学者解释与逐文件索引见[教程](../docs/FOC-从零读懂这个工程.md)和[文件导览](../docs/文件导览.md)。

## 当前默认参数和使用

ZH3620-1 为 14 极、7 极对。M0 电流环 10 kHz、ADC1 顺序采 B/C、28 周期、TIM1 触发点 8200；电流 PI `Kp=0.20 V/A`、`Ki=192 V/(A·s)`、回算 `1200/s`。默认塑料臂配置：速度 PI `0.18/0.48`、目标变化率 4500 rpm/s；位置 P `12 rpm/°`、加速/刹车 `4000/3200 rpm/s`、接近上限 150 rpm。裸轴另有 `FREE_SHAFT` 参数。当前选择依据见[电流采样与调参](../../../docs/current-sampling-noise.md)和[定位提速](../../../docs/position-speedup-and-foc-roadmap-2026-09-28.md)。

首次安装上电只静止测零偏，不自动转动；成功执行 `cal` 后才接受目标。`Iq <A>`、`rpm <rpm>`、`pos <deg>` 会选择对应模式；`zero` 定义位置零点；`stop` 关闭功率输出；`clear` 清告警。默认 WARN 只记录有限目标超出诊断阈值，单次目标持续有效；无效角度或无法计算时仍会关闭输出。连接 VOFA、通道表和命令操作见[VOFA 指南](../../../docs/vofa-quickstart.md)。

目前实际验证范围是约 12 V、这一台电机和可双向连续旋转的塑料臂的短时运动。板载电流绝对比例、栅极波形与长期温升尚未独立验收。换电机、编码器、相线或机械负载后须核对参数和接线，并按安装身份重新校准。

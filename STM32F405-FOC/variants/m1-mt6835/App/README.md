# 共用 FOC 应用层

当前默认为 **M1 + TLE5012B + L4225_6S420_A + FREE_SHAFT + 安装 4 + WARN**，12 V 空载。所有接口/电机组合共用本目录代码，型号参数由 `Config/` 选择，硬件驱动由 CMake 选择。

## 实时调用链

1. `Core/Src/main.c` 初始化外设并调用 `app_init()`；恢复校准、电流参数和有效补偿表，随后开始静止电流零偏估计。补偿表启动默认关闭。
2. PWM 固定相位触发 ADC，M0 中断或 M1 双 ADC DMA 回调读取磁编；有效采样交给 `app_sample()`。
3. `foc_step()` 做角度更新、Clarke/Park、电流 PI、耦合前馈、限幅和 SVPWM，先写下一周期的 PWM 预装载。
4. `foc_outer_step()` 调用 `control_step_with_angle()`。正常速度/位置外环为 1 kHz，抗齿槽静态采集位置 PID 为采样频率 10 kHz；输出作用于后续电流采样。
5. 每次采样积累遥测窗口；USB 总览 `send 6` 再分频到 2.5 kHz。主循环处理命令、USB 队列和停机 Flash 保存，CAN 暂无应用控制协议。

## 文件职责

| 文件 | 职责 |
| --- | --- |
| `Config/foc_port.h` | M0/M1 定时器、采样频率、ADC 窗口和板级阈值 |
| `Config/foc_motor.h` | 电机及负载资料、初始 PI 和外环增益 |
| `Config/foc_encoder_profile.h` | 磁编型号和角度修正配置 |
| `Config/foc_profile.h`、`foc_policy.h`、`foc_telemetry.h` | 组合身份、WARN/TRIP 策略和遥测分频 |
| `Control/app.c`、`telemetry.h` | 命令事务、采样调度、遥测、保存和告警发布 |
| `Control/control.c` | 目标保持、速度/位置外环、轨迹和静态采集 PID |
| `Control/anticogging.c` | 720 点补偿表、插值、上传校验、启用/退出和安装绑定 |
| `FOC/foc.c`、`sine_table.inc` | 坐标变换、电流环、SVPWM、校准和状态机 |
| `FOC/motor_ident.c` | R/L 辨识、拟合检查及 SI 单位电流 PI 计算 |
| `FOC/sensor_cal.c` | 极对数、磁编方向、电角零点扫描和质量检查 |
| `Hardware/bsp/bsp_motor.c`、`bsp_motor_record.h` | PWM 提交/关断、时序统计和 Flash 记录 |
| `Hardware/bsp/bsp_adc_m0.c`、`bsp_adc.c` | M0 顺序/可选同步 ADC，M1 双 ADC DMA |
| `Hardware/encoder/`、`tle5012b/`、`mt6835/` | 磁编统一接口和芯片协议 |
| `Hardware/bsp/bsp_usb.c`、`bsp_uart.c`、`bsp_can.c` | 通信物理层和队列 |
| `Protocols/JustFloat/justfloat.h` | 帧封装 |

电流 PI 参数用 SI 单位保存，对采样频率的离散换算在参数恢复/应用/回滚/失效时更新，电流 ISR 使用缓存系数。其他模块只读 `foc_parameters`；修改必须调用 FOC 参数 API，前台运行时变更须锁中断。

## 使用入口

- [自动辨识与电流环自整定](../../../docs/motor-autotune.md)
- [抗齿槽采集、启用与实测](../../../docs/anticogging.md)
- [VOFA 命令与通道](../../../docs/vofa-quickstart.md)
- [当前架构](../../../docs/architecture.md)、[回归入口](../tests/README.md)

上电不会自动转动。有效校准与零偏就绪后，`Iq`、`rpm`、`pos` 选择对应模式；`zero` 重定义相对位置零点，`stop` 关闭输出。默认 WARN 单次目标持续有效，断开 USB 不会停机；自动辨识和抗齿槽采集独立执行更严格的中止条件。Flash 校准重写会使旧补偿表失效。

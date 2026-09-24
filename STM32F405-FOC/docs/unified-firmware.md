# 统一固件：使用、结构与扩展

## 两种固件有什么共同点

在控制算法看来，两种配置都提供一次采样的机械角度、B/C 相电压、母线电压和三相 PWM 输出能力。`app.c` 不知道编码器型号，`foc.c` 不知道电机插在 M0 还是 M1。USB CDC、UART、CAN 驱动和 `Iq`、`rpm`、`pos`、`cal`、`zero`、`stop`、`clear`、`send` 命令也只实现一次。

差异由编译选项 `FOC_BOARD` 决定：顶层 `STM32F405-FOC/` 固定默认 M0；子目录 `variants/m1-mt6835/` 默认 M1。输出文件名不同，烧录前核对。顶层遗留的 `App/` 和 `Core/` 保留历史试验源码，但当前不会参与编译。统一实现目前放在原 M1 子目录，是因为这里已包含 USB、UART、CAN 与 CubeMX 外设基础；它现在同时为 M0 编译。

| 文件 | 职责 |
|---|---|
| `App/Config/foc_profile.h` | 极对数、采样频率、电流/转速/母线范围、校准电压、环路参数和板卡编号 |
| `App/Control/app.c` | 初始化、命令解析、四组 USB 遥测、UART 遥测、故障清除、Flash 保存调度 |
| `App/Control/control.c` | 1 kHz 速度与位置外环；速度/位置命令 200 ms 看门狗 |
| `App/FOC/foc.c` | 电流零偏、编码器角度与电角度、电流 PI、SVPWM、对齐与故障状态机 |
| `App/Hardware/encoder/bsp_encoder.h` | 统一角度接口：有效角度、原始角度和采样延迟 |
| `bsp_encoder_tle5012b.c` | M0 的 SPI3 模式 1 及 TLE5012B CRC/状态检查，失败输出 NaN |
| `bsp_encoder_mt6835.c`、`mt6835/` | M1 的 SPI3 DMA 与 MT6835 CRC/状态检查 |
| `App/Hardware/bsp/bsp_adc_m0.c` | M0 TIM1 → ADC1 注入采 B/C，相同周期 ADC2 采母线 |
| `App/Hardware/bsp/bsp_adc.c` | M1 TIM8 → 双 ADC DMA 采 B/C 与母线 |
| `App/Hardware/bsp/bsp_motor.c` | 按配置驱动 TIM1/TIM8 六路 PWM、定时检查、关断、带板卡编号的 Flash 校准记录 |
| `App/Hardware/bsp/bsp_usb.c`、`bsp_uart.c`、`bsp_can.c` | 通信传输层；CAN 目前只提供收发 API，没有电机命令协议 |
| `Core/Src/stm32f4xx_it.c` | 把不同采样中断汇合到同一个 `app_sample()` |

新增编码器时，实现 `bsp_encoder.h` 的接口，让坏帧返回 NaN；在 CMake 里只替换编码器源文件和 SPI 初始化参数。新增电机或板卡时，先测极对数、电阻、电感、磁链、电流比例和可用母线范围，再在 `foc_profile.h` 增加配置，并核对 PWM 引脚、ADC 通道与采样时序。换电机或重新安装编码器时要修改 `FOC_MOTOR_ID` 并重新校准：Flash 记录同时核对板卡、安装编号和极对数。修改 `Core/` 中 CubeMX 生成区域前，应先同步 `.ioc`；当前 `.ioc` 尚不能一键重生成两种配置。

## 构建与连接

默认 M0：在 `STM32F405-FOC/` 运行 `cmake --preset Debug` 和 `cmake --build --preset Debug`；本机 VS Code 可用 `Debug-local`。ELF 是 `build/Debug/STM32F405-FOC.elf`。可选 M1：进入 `variants/m1-mt6835/`，用同样的 `Debug` 预设，ELF 是 `build/Debug/405_FOC.elf`。VS Code 默认烧录任务只对应 M0。

M0 使用 Motor0 三相口，TLE5012B 在 SPI3/PA0；M1 使用 Motor1 三相口及 MT6835，编码器同样走 SPI3/PA0。PA1 是共享总线另一片选，会保持高电平。两者都需要 ST-Link 烧录，USB PA11/PA12 用于 CDC 虚拟串口，USART2 PA2/PA3 可接独立 USB 转 TTL。切换固件后要重新核对三相接线、编码器安装、供电和校准。

## USB 上位机

烧录后原生 USB 应枚举为 CDC COM 口，产品名分别为 `FOC M0 TLE5012B` 或 `FOC M1 MT6835`。在 VOFA+ 选择 Serial/串口、对应 COM、2000000 baud、8N1、无流控，协议选 JustFloat 并开启 DTR。CDC 的波特率是主机接口设置，不决定 USB 总线速率。若 M0 无法枚举，先检查板上 8 MHz HSE、USB 线路和 48 MHz 时钟配置；M0 的新 USB 路径尚未实机验收。

USB 每帧 12 个小端 float32，末尾是 JustFloat `+Inf`，共 52 字节。M0 每秒 10,000 帧（约 520 kB/s）；M1 每秒 20,000 帧（约 1.04 MB/s）。第 0、1 通道打包时间/状态及序号/组号，**不能直接当普通浮点曲线**。`send 3` 选择外环组，通道 6 是转速、8 是目标转速、9 是实测 Iq。其他通道及单位见 [数据通道表](../variants/m1-mt6835/tools/bench/README.md)。持续记录需主机持续读取；VOFA 的绘图刷新率不能替代完整性检查。

需要验证帧是否连续时，在 M0 下运行 `python tests/capture_usb.py COM端口 --sample-hz 10000 --seconds 30`（从共用固件目录执行）；M1 可省略采样率选项。完整的台架工况工具目前仍使用 M1 的默认 20 kHz 时序和目标范围，**不要直接用其 `run` 命令驱动 M0**。

文本命令以真正的 CR、LF 或 CRLF 结束，不发送字面量反斜线。USB 波形流不插入文字 ACK；`hello` 的文字回应只从 UART 发出。

| 命令 | 用途 | M0 范围 | M1 范围 |
|---|---|---|---|
| `send 0` 至 `send 3` | 选择遥测组 | 相同 | 相同 |
| `cal` | 静止待机时对齐并保存电角度零点 | 首次必须显式发送 | 首次无记录时原固件可自动对齐 |
| `Iq 0.20` | 转矩电流目标 | ±0.30 A | ±5 A |
| `rpm 10` | 速度目标 | ±100 RPM | ±9400 RPM |
| `pos 90` | 绝对多圈位置目标，单位° | ±1e6° | ±1e6° |
| `zero` | 静止时定义当前位置为 0° | 相同 | 相同 |
| `stop` | 立即关断，取消斜坡与外环 | 相同 | 相同 |
| `clear` | 只清除已经消失的故障，不自动转动 | 相同 | 相同 |

速度与位置模式要求上位机在 200 ms 内重复发送目标；建议每 50–100 ms 重发。手动在 VOFA+ 发一次后自动停机是看门狗行为。转矩模式没有该续发要求。`Iq 0` 在待机时不会启动；`rpm 0` 和 `pos 0` 是实际闭环运行请求。

## M0 首次调试顺序与验收边界

M0 上电先保持栅极关断，约 200 ms 静止检查后再用 2048 组样本估计电流零偏。此时即使尚无 Flash 校准记录也不会自动带电机转动。确认编码器、ADC 和母线数据有效后发送 `cal`；对齐成功才保存本配置的零点并接受转矩/速度/位置目标。任何编码器坏帧、母线异常、采样超时或电流超限都会锁存故障并关断。

M0 的极对数目前暂按 7，ADC VDDA 暂按 3.13 V，标称分流 1 mΩ、增益 20；速度/位置和对齐参数是保守起点，尚未完成对应实物调参。旧 M0 固件曾完成短时 0.2 A 电流闭环，但旧版自动 60 rpm 未成功；这些数据不能直接证明新统一固件的中断时序。上板应先用限流电源与自由轴进行 USB 枚举、10 kHz 周期、编码器 CRC、ADC 零偏、六路栅极/死区示波、对齐，再逐步验证低转矩、速度和位置。软件电流保护不能代替硬件短路保护。

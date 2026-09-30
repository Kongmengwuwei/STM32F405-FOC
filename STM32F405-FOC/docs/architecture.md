# 当前 FOC 架构

顶层默认双路构建使用 `variants/m1-mt6835/dual/`，两路独立运行同一套 M0 控制算法，硬件同步采样细节见[双路控制指南](dual-motor.md)。本页下述接口二选一架构对应 `FOC_DUAL=OFF` 的单路固件。

可执行固件共用 `variants/m1-mt6835/` 内的 `App/`、`Core/` 和 USB 中间件。目录名 `m1-mt6835` 是来源历史，并不限制现在能构建的组合。顶层旧 M0 试验代码已经移除，避免误读成另一套可运行固件。

四个选择有不同含义：`FOC_PORT` 指三相功率接口及其 PWM/ADC 路径；`FOC_ENCODER` 指机械角度芯片及 SPI 帧格式；`FOC_MOTOR` 指电机本体参数；`FOC_INSTALLATION_ID` 指磁铁、编码器与相线的具体安装。M0/M1 不是电机名称。

| 层 | 主要文件 | 职责 |
|---|---|---|
| 接口 | `App/Config/foc_port.h`、`bsp_motor.c`、`bsp_adc*.c` | M0/TIM1/ADC1 注入 10 kHz 或 M1/TIM8/双 ADC DMA 20 kHz，电流和母线采样、PWM 安全关断 |
| 编码器 | `App/Config/foc_encoder_profile.h`、`bsp_encoder_*.c`、`Core/Src/spi.c` | TLE5012B 的 16 位模式 1 或 MT6835 的 8 位模式 3，角度有效性检查 |
| 电机 | `App/Config/foc_motor.h` | 型号、极对数、资料参数与保守控制限值；当前 ZH3620-1 及型号待确认的参考电机 |
| 组合 | `App/Config/foc_profile.h` | 综合接口与电机限值，绑定 Flash 校准身份 |
| 共用算法 | `App/Control/app.c`、`control.c`、`App/FOC/foc.c` | 命令、遥测、位置/速度/电流环、SVPWM、对齐与故障状态机 |

M0 的 ADC1 中断或 M1 的 ADC DMA 半传输中断开始读取编码器。TLE5012B 同步完成后进入 `app_sample()`；MT6835 在 SPI DMA 完成后进入 `app_sample()`。因此接口与编码器可交叉构建，算法层不需要按具体型号复制。当前已经编译四种接口/编码器组合；交叉组合的 SPI/ADC 最坏耗时与实物接线仍需实测。

每次采样的数据流是：PWM 固定相位触发 ADC → 读取编码器和母线 → `foc_step()` 计算电角度、电流 PI 与 SVPWM → 写入下一周期的 PWM 预装载 → USB/UART 遥测。USB CDC 或 USART2 命令进入共用解析器。CAN1 尚无控制命令协议。

校准记录 v3 绑定功率接口、电机型号、编码器型号、安装编号及极对数。v1/v2 没有足够信息区分新组合，因此不会自动复用。所有组合上电只做静止电流零偏估计，首次需显式 `cal`；随后才允许转矩、速度与位置命令。具体构建和使用见[统一固件指南](unified-firmware.md)及[VOFA 操作](vofa-quickstart.md)。

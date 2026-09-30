# F405 双路驱动板硬件核对

顶层固件现在默认同时控制两轴；`m0` 驱动板上 M0 三相接口及原有电流采样，编码器使用 SPI1 的 PB3/PB4/PB5、CS1/PA1；`m1` 驱动板上 M1 三相接口及原有电流采样，编码器使用 SPI3 的 PC10/PC11/PC12、CS0/PA0。本页表格的 M0/M1 均指板上丝印；原有“当前默认实物 M0”描述记录的是改动前已验证的台架接线。详见[双路控制指南](dual-motor.md)。

以下依据《ODrive原理图_F405.pdf》（下称“原理图”）、《驱动板405使用说明.pdf》（下称“使用说明”）和《SimpleFOC开箱测试.pdf》（下称“开箱测试”）。这些是板卡资料，不代表已经核对实物焊接版本。页码按 PDF 页面计。

板上有 M0、M1 两组三相功率桥，另有 AUX 半桥。两组三相均由 STM32F405 的高级定时器驱动，各通过 EG2134 和六只 MOS 管连接电机；每组 B/C 相的低侧分流器供 ADC 采样。直流母线先到功率桥和母线电容，再经板上降压得到约 12 V 栅极驱动电源、5 V、3.3 V 逻辑电源。此前已验证的台架组合是 **M0 三相 + TLE5012B 的 SPI3/PA0 + ZH3620-1**；双路固件已改为本页开头的 SPI 对应关系。历史参考工程的 M1/MT6835 接线不能照搬。原理图图片见 [第 1 页](schematic/odrive-1.png)、[第 2 页](schematic/odrive-2.png)、[第 3 页](schematic/odrive-3.png)。更细的电源、接口及引脚索引仍保留在[参考板硬件拓扑](../variants/m1-mt6835/硬件PCB拓扑.md)；其中接线示例针对旧 M1/MT6835 组合，应以本页和实际焊接版本为准。

## 控制与采样

| 信号 | M0 | M1 | 来源 |
| --- | --- | --- | --- |
| 三相高侧 PWM | PA8/PA9/PA10，TIM1 CH1/2/3 | PC6/PC7/PC8，TIM8 CH1/2/3 | 原理图 p1 |
| 三相低侧 PWM | PB13/PB14/PB15，TIM1 CH1N/2N/3N | PA7/PB0/PB1，TIM8 CH1N/2N/3N | 原理图 p1 |
| B/C 相电流放大输出 | PC0/PC1，ADC10/11 | PC3/PC2，ADC13/12 | 原理图 p1–3 |
| ABZ 编码器 A/B/Z | PB4/PB5/PC9，TIM3 CH1/2 | PB6/PB7/PC15，TIM4 CH1/2 | 原理图 p1 |

两相采样分别来自 B/C 低侧 1 mΩ 分流电阻。运放输出以 1.65 V 为偏置，图纸标称增益 20；具体电流极性和比例仍须在实物上校准（原理图 p2–3；使用说明 p1）。

J4 编码器接口：M1 占 1–6 脚，M0 占 7–12 脚；每组依次为 3.3 V、5 V、A、B、Z、GND。A/B/Z 有 3.3 kΩ 上拉到 3.3 V，图纸未显示电平转换（原理图 p1）。

其他传感器接口：AS5600 使用 A/B 作为 SCL/SDA，因此 M0 的 PB4/PB5 需软件 I²C，M1 的 PB6/PB7 可用 I2C1（使用说明 p2；开箱测试 p3）。SPI 编码器共享 PC10/PC11/PC12，片选 CS0=PA0、CS1=PA1；相关 GPIO 经过可选焊桥，须核实焊桥位置（原理图 p1；使用说明 p2；开箱测试 p4–5）。

当前实物反馈已改为 MENC15A，用户确认内置芯片为 TLE5012B。改动前只读固件按 PC10=SCK、PC11=MISO、PC12=MOSI、PA0=CS 配置 SPI3 模式 1，实测角度帧和安全字 CRC 正常；改动后两路接线见本页开头。MENC15A 模块标注的 SPC/DSI/SDO 分别对应 SCK/MOSI/MISO。用户提供的当前模块照片把电源接口标为 `5V`，实际两路也均接板上 5 V；[逐飞科技公开驱动](https://gitee.com/seekfree/CYT2BL3_Brushless_Driver_Project/blob/master/Seekfree_CYT2BL3_Double_Foc_Project/libraries/zf_device/zf_device_menc15a.c)给另一版本模块的 VCC 标为 3.3 V，不应直接用于当前实物接线。

GPIO3/4 是另一组串口信号，对应 PA2/USART2_TX 与 PA3/USART2_RX，用于 USB 转 TTL 调试（原理图 p1；使用说明 p2；开箱测试 p6）。[AS5600 官方数据手册](https://look.ams-osram.com/m/7059eac7531a86fd/original/AS5600-DS000365.pdf)列出 I²C、模拟/PWM 输出，没有原生 UART。若实际模块连接在 GPIO3/4，必须另有串口转换电路和协议。

开箱测试 p4–5 以 AS5047P 演示 SPI 接线，p10 还列出 TLE5012B 和 MT6701 作为支持示例；这三种芯片不能仅凭“SPI 磁编码器”名称共用同一帧格式。

母线电压采样 VBUS_S 接 PA6/ADC6，分压器为 39 kΩ 与 2.2 kΩ，理想比例 2.2/41.2 ≈ 0.053398（原理图 p1）。HSE 晶振标为 8 MHz（原理图 p1），已将 `.ioc` 和 HAL 配置中的 HSE_VALUE 从 25 MHz 更正为 8 MHz。当前运行时仍只使用 16 MHz HSI；启用 HSE 前须核对实物晶振。

## 上电前待确认

- 使用说明 p1–2 给出 60 V 设计上限、建议 8–36 V，开箱测试 p1/p6 则写 12–56 V。首次调试应按较保守的说明和实际器件额定值选择电源。
- 图中 EG2134 栅极驱动器使用六路 PWM 输入，未看到独立使能或故障反馈线；PB12/TIM1 BKIN 也未连接（原理图 p1–3）。不能假定硬件有自动过流关断。
- [EG2134 厂商资料](https://egmicro.com/products/detail/?name=EG2134)说明 HIN/LIN 高电平有效、输入内建下拉且无使能脚，因此按原理图型号设计的固件启动时将 12 路控制输入设置为低电平。实物芯片型号仍须核对。
- 原理图 p2–3 提醒器件型号仅供参考，以实际焊接件为准。电流量程、驱动器型号、供电极限、相序及采样极性均需实物核验。

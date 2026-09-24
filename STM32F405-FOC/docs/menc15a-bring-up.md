# M0 TLE5012B（MENC15A 模块）只读验收

用户确认模块接在 M0/CS0（PA0），并明确芯片为英飞凌 TLE5012B。主板使用 SPI3：PC10=SCK、PC11=MISO、PC12=MOSI。模块 SPC/DSI/SDO 对应 SCK/MOSI/MISO。当前固件只读位置，不配置 PWM；启动与错误处理均将驱动输入保持低电平。

## 协议与实现

用户提供的逐飞 `zf_device_menc15a.c/.h` 示例采用 SPI 模式 0、16 位字，并发送 `0x8021` 后读取一个返回字。实物芯片确定后，按[英飞凌 TLE5012B 数据手册](https://www.infineon.com/assets/row/public/documents/24/49/infineon-tle5012b-exxxx-datasheet-en.pdf?fileId=db3a304334fac4c601350f31c43c433f)和[英飞凌 SSC 接口说明](https://community.infineon.com/t5/%E5%8D%9A%E5%AE%A2/TLE5012B-%E8%AF%A6%E8%A7%A3%E7%B3%BB%E5%88%97%E4%B8%80%E4%B9%8BSSC-SPI%E9%80%9A%E8%AE%AF%E6%8E%A5%E5%8F%A3/ba-p/1041095)改为 **SPI 模式 1（CPOL=0，CPHA=1）**、16 位字、软件片选。CS 拉低后发送命令 `0x8021`，留出命令与数据之间的规定间隔，再读取角度数据字和安全字，最后 CS 拉高。发送读取时的填充值为 `0xFFFF`。角度为数据字低 15 位，每机械圈 32768 计数；安全字包含 CRC、传感器编号、复位、系统、接口和角度有效状态。

板卡传输代码位于 `App/Src/board_spi_encoder.c`；TLE5012B CRC 和安全字检查位于 `App/Src/tle5012b_protocol.c`；`App/Src/menc15a_device.c` 保留原工程 `menc15a_init()` 和 `menc15a_get_absolute_data(menc15a_1_module)` 接口。现在 `menc15a_last_transport_ok` 仅表示 SPI 传输完成，`menc15a_last_read_ok` 表示 CRC 与全部安全位及 M0 传感器响应标识通过。只有后者为真时才更新公开的绝对角度和增量；首帧及失败后的首帧增量为零。上层使用角度时必须检查 `menc15a_last_read_ok`。M1 尚未接入驱动。

## 上板观测

2026-09-23 经 VS Code STM32 扩展附带的 CubeProgrammer 和 ST-Link 构建、烧录、回读校验成功，目标 3.13 V。

模式 1 的实测帧示例：命令 `0x8021`、角度数据 `0x995A`、安全字 `0x3EB4`；按命令和数据计算的 CRC 为 `0xB4`，与安全字一致。首次安全字记录了复位/系统历史状态。只读固件随后读取一次 STAT 寄存器 `0x00` 以取得并清除锁存记录：一次上电首次值 `0x8005`，其中 `S_RST` 和 `S_VR` 置位；后续读数 `0x8000`。后续角度安全字如 `0xFE80`，CRC、传感器响应、复位、系统、接口和角度有效位全部通过，CRC 与传感器错误计数均为 0。`S_VR` 表示曾发生电源监测事件，这次已经清除；仅凭这份历史记录不能确定原因，若后续再次出现，须排查传感器供电。

用户在模式 1 固件运行时缓慢单方向手转 M0 轴，低 15 位角度覆盖 **59–32757**，最大相邻 2 ms 位移 **104 计数**，超过 1024 计数的跳变 **0 次**，累计约 **−37008 计数**。累计量略多于一机械圈，可能是实际手转超过一圈或首尾未对齐；这次试验证实 15 位角度连续和跨零点正常，但还没有精确标定一圈机械角度与电角度比例。后续安全校验版固件静止运行时传输、CRC 和角度状态均有效，角度跟踪器状态为 `FOC_ANGLE_VALID`，故障计数为 0。

此前模式 0 下多次看到 `0x4000`–`0x7FFF`，低 14 位似乎走完一圈且低 15 位出现半圈跳变。**这是时钟相位与器件要求不符造成的错误解帧，不是芯片只有 14 位分辨率。**原先的 14 位结论作废，当前固件不再含 14 位控制路径。

## 调试变量

| 变量 | 含义 |
| --- | --- |
| `g_menc15a_transport_ok` | 最近一次底层 SPI 传输是否完成 |
| `g_tle5012b_sample_valid` | 最近一次角度帧 CRC、安全状态和 M0 响应标识是否全部通过 |
| `g_menc15a_angle_reply` / `g_menc15a_safety_reply` | 最近一次原始角度字和安全字 |
| `g_menc15a_candidate_angle` | 原始角度字的低 15 位，仅供诊断；无效帧时不得用于控制 |
| `g_tle5012b_crc_error_count` / `g_tle5012b_sensor_error_count` | CRC 与其他安全状态错误次数 |
| `g_tle5012b_initial_stat_*` / `g_tle5012b_followup_stat_*` | 启动和稍后读取的 STAT 原始字及 CRC 结果 |
| `g_menc15a_angle_status` / `g_menc15a_tracker_fault_count` | 角度跟踪器状态与锁存故障次数 |
| `g_menc15a_net_counts` / `g_menc15a_max_abs_step` / `g_menc15a_large_step_count` | 仅基于有效帧计算的手转连续性诊断 |

进入电流闭环前还需验证传感器断线时安全状态、磁场裕量、机械方向和 7 极对对应关系，并完成 PWM 同步相电流采样及电流限值。当前绝不自动输出转矩。

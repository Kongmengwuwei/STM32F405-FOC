# STM32F405 有感 FOC：龙邱电机分支

本分支默认驱动 **龙邱 L4225-6S420-A（420KV）+ M1 功率口 + 原 TLE5012B 磁编**，用户确认供电为 **12 V**。编码器仍用 SPI3：PC10=SCK、PC11=MISO、PC12=MOSI、PA0=CS0。M1 切换三相功率输出与电流采样，磁编接口保持原接线。

| 配置 | 本分支默认值 |
|---|---|
| `FOC_PORT` | `M1`，TIM8，10 kHz 电流环 |
| `FOC_ENCODER` | `TLE5012B`，SPI3/PA0 |
| `FOC_MOTOR` | `L4225_6S420_A`，14 极 / 7 极对、12 槽 |
| `FOC_LOAD_PROFILE` | `FREE_SHAFT`，已完成短时空载电流和速度测试 |
| `FOC_INSTALLATION_ID` | `2`，当前装配已校准并保存 |
| `FOC_PROTECTION` | `WARN`，沿用单次命令保持和阈值告警 |

图片标注工作电压 11.1–24 V、最大电流 13.5 A、转矩 0.6–1.1 N·m。13.5 A 没有被当作首次 Iq 目标。新电机普通运行的 d/q 电压矢量幅值暂限为母线的 10%（12 V 时 1.2 V），在 WARN 下也生效；速度/位置外环输出限制为 ±2 A，WARN 下也生效；校准采用 3 A 目标、1.5 V 电压上限、15° 起始电角度。电流诊断阈值为 1 A、相电流诊断阈值为 4 A，**WARN 不按这些阈值关断，也不会在断开 USB 后停止**。绕组参数未知，电感和磁链前馈关闭。

在 `STM32F405-FOC/` 用 `cmake --preset Debug`、`cmake --build --preset Debug` 构建；本机可用 `Debug-local`。输出在 `build/longqiu/Debug/STM32F405-FOC.elf`，另有 `.bin` 和 `.hex`。本机 VS Code 构建、烧录和调试任务已指向该组合和目录；`.vscode/` 与 `CMakeUserPresets.json` 按现有规则仅保留在本机。

接线、校准、首次小电流试转见[龙邱电机首测说明](STM32F405-FOC/docs/longqiu-first-test.md)。已烧录并校验，校准保存及软件复位复用、±0.1 A 跟踪、±30 rpm 空载短测通过；后续调参使 ±30 rpm 末段均值改善为 +29.69/−29.89 rpm，位置 ±1/5/10/30/60° 短测末段误差约 0.03°以内，见[控制调试](STM32F405-FOC/docs/longqiu-control-tuning-2026-10-09.md)。低速纹波、带载、外部测量基准和长期温升尚未验收，见[实测记录](STM32F405-FOC/docs/longqiu-bring-up-2026-10-09.md)。历史 ZH3620-1/塑料臂测试结果不代表这台新电机的性能。其他入口见[工程说明](STM32F405-FOC/README.md)、[统一固件](STM32F405-FOC/docs/unified-firmware.md)、[硬件映射](STM32F405-FOC/docs/hardware-map.md)和[VOFA 操作](STM32F405-FOC/docs/vofa-quickstart.md)。

# L4225-6S420-A：M1 / TLE5012B / 12 V

用户确认电机为图中 L4225-6S420-A，420KV、12 槽、14 极（7 极对），实际 12 V，磁铁安装在新轴上且轴空载自由转动。资料工作电压 11.1–24 V、最大电流 13.5 A、转矩 0.6–1.1 N·m；未给绕组参数和连续相电流额定值。2026-10-09 已完成烧录和空载短测，见[实测记录](longqiu-bring-up-2026-10-09.md)。

2026-10-10 已继续完成连续换速、反向和定位提速，当前参数及实测范围见[动态响应记录](longqiu-dynamic-response-2026-10-10.md)。

## 接线与身份

三相接 M1：高侧 PC6/PC7/PC8，低侧 PA7/PB0/PB1，由 TIM8 驱动；B/C 电流采样为 PC3/PC2（ADC13/12），双 ADC 同步 DMA。M1/TLE5012B 使用 10 kHz 电流环，总览组 6 为 2.5 kHz / 24 通道，旧组 0–5 为 5 kHz。M1/MT6835 仍为 20 kHz，未用 M0 关闭。

磁编保持 TLE5012B / SPI3：PC10=SCK、PC11=MISO、PC12=MOSI、PA0=CS0，模式 1、16 位帧。功率口换 M1 不改变片选。

安装编号为 2，Flash 校准身份 `0x1312`（M1 / 新电机编号 3 / TLE5012B / 安装 2）。当前装配已保存零点，软件复位后可直接运行，无需每次上电校准。换磁铁位置或改三相接线后递增安装编号并重新校准。上电只静止估计电流零偏，不自动转动。

## 构建与烧录

在 `STM32F405-FOC/` 目录运行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

本机可用 `Debug-local`，生成 `build/longqiu/Debug/STM32F405-FOC.elf`、`.bin`、`.hex`。配置应为 `port=M1, encoder=TLE5012B, motor=L4225_6S420_A, load=FREE_SHAFT, installation=2, protection=WARN`。

本机 VS Code 构建、烧录和 F5 调试路径已更新。USB 产品名为 `FOC L4225-6S420-A TLE5012B M1`。工具路径按现有规则在忽略的 `.vscode/` 和 `CMakeUserPresets.json`。已使用 ST-Link 下载并校验，程序下载保留 sector 11 的校准记录；整片擦除会丢失记录。

## 操作与重新校准

1. VOFA 选择当前 COM 口（本次 COM8）、JustFloat、1000000、8N1，开启 DTR；命令附真实换行。发送 `stop`、`send 6`，观察待机状态 0、母线、电流和角度。
2. 当前接线可复用已保存零点。需重新校准时让轴自由转动，停止手动转轴后发送 `cal`。每方向机械行程约 360/7=51.43°，扫描总计约 10 s，再保存 Flash；至少等待 14 s 后检查。保存时 PWM 已关断，单 bank Flash 操作暂停 USB 约 1 s，并重置采样时间戳；这个时间戳跳变不是运行中丢帧。
3. 校准使用 3 A 目标、1.5 V 电压上限、15° 起始电角度，保留行程、回位误差和静止角度离散度检查。检查 `foc.calibrated`、Flash 记录或小电流目标进入 RUN；只回待机不足以证明成功。失败时查看 ALIGNMENT/FLASH 告警和接线。
4. 短时 `Iq 0.10` 后发送 `stop`；反向使用 `Iq -0.10` 后再次 `stop`。±0.1 A 跟踪已实测。后续速度与位置调试见[控制调试报告](longqiu-control-tuning-2026-10-09.md)：±10/30/60 rpm 各 3 s 的均值已接近目标；±1/5/10/30/60° 和回零短测的末段反馈误差均约 0.03°以内，但低速纹波仍明显。

WARN 模式下单次目标持续保持，关闭串口或拔 USB 不会停机；`Iq 0` 只给零转矩，`stop` 才关 PWM。5 A 目标、10 A 相电流、600 rpm 速度模式、600 rpm 电流模式均为诊断阈值，WARN 不据此拒绝或关断。旧版 1 A 阈值曾在低速克服阻力时记录 CURRENT（1024）；最终动态响应复测累计告警为零。

新电机实际软件限幅：运行 `sqrt(Ud²+Uq²)` 不超过母线 35%（12 V 时 4.2 V），速度/位置外环输出和积分限 ±6 A，WARN/TRIP 均执行，TRIP 还有更严的 5 A 限制。直接 Iq 命令沿用原 WARN 策略。软件限幅不是硬件电流保护，也不能证明连续负载能力。

## 参数与验证范围

`App/Config/foc_motor.h`：电流 PI 0.20 / 80/s、抗饱和 1200/s；速度低速 PI 0.08 / 0.40、滤波 0.5 ms，高速 PI 0.04 / 0.80、滤波 2 ms，在参考速度 60–150 rpm 间平滑插值，速度参考斜坡 15000 rpm/s；位置模式速度 P 低速 0.06、高速 0.04，位置 P 12、逼近速度 240 rpm、加速 10000 rpm/s、刹车 4800 rpm/s。绕组 R/L/磁链未知，前馈关闭。

真实板上验证 PWM 提交、SPI 读数、校准保存复用、小电流跟踪和短时双向低速旋转。M1/TLE5012B 从 20 kHz 改为 10 kHz 解决 PWM 写入窗口不足。已做小步进及双向位置短测；尚未完成外部电流/角度基准测量、带载、长期温升和额定功率测试。

可重复的短测工具为 `variants/m1-mt6835/tools/bench/longqiu_probe.py`，例如 `python tools/bench/longqiu_probe.py --stage speed --rpm 30 --out build/bench-longqiu`（从 variant 目录运行）。工具依赖 pyserial/numpy，COM 默认 COM8；运行前断开 VOFA。每段定时发送 stop 并检查响应，通信关闭也执行 stop；工具仍不能替代硬件保护。

# tools：电机测试数据系统

本工具集的 `bench.py` 与 `frames.f32` 使用兼容遥测组 `send 0`～`send 5`，每帧 52 字节。**固件上电默认 `send 6` 是 24 通道、100 字节/帧，M0 为 2.5 kHz**；需要看全部目标/实际值时使用[VOFA 总览](../../../../docs/vofa-quickstart.md)，不要用下文 52 字节解析器直接解析总览。工具会按所需组号向固件发送 `send X`。

电流环的最新带载比较见[电流采样与调参](../../../../docs/current-sampling-noise.md)，当前 ZH3620-1 Kp=0.20、Ki=192/s。`current_probe.py` 的参数只是记录已烧录的增益；切组时会先排空旧帧，再检查序号连续性。

新增 `overview_probe.py --port COM8 --out build/overview`：先断开 VOFA 串口，在约 12 V 空载条件下验证组 6 的转速、位置、电流目标与实际值；退出会发送 `stop` 并排空接收。该工具有自己的实验边界检查，不修改固件保护策略。[VOFA 总览说明](../../../../docs/vofa-quickstart.md)记录了显示修正、实测数据和仍存在的问题。

**当前策略（2026-09-27）：** 固件默认 WARN，各型号均取消目标范围拒绝、通信超时关断和阈值跳闸，`send 4` 为数值诊断组。下文旧限幅与看门狗描述对应 TRIP/历史版本；详见[台架告警模式](../../../../docs/bench-warning-mode.md)。主机 `bench.py run` 的参考电机工况库和自有 `--iq-limit/--rpm-limit/--pos-limit` 检查独立于固件，不用于默认 ZH3620-1 的 VOFA 手动电流测试。

本目录保存**主机端工具**。固件接口与命令见 [App/README.md](../../App/README.md)，
主机回归测试见 [tests/README.md](../../tests/README.md)，硬件接线见
[硬件映射](../../../../docs/hardware-map.md)。

## bench/：一键电机台架

从仓库根目录用 `download/bench.cmd`，或直接调用 Python：

```sh
download\bench.cmd list                 # 列出串口，FOC 板前有 * 标记
download\bench.cmd discover             # 探活：帧率、字节率、帧尾校验、连续性
download\bench.cmd run --dry-run        # 只打印工况清单与预估时长，不发命令
download\bench.cmd run                  # 全量工况（需 --yes 或交互确认）
download\bench.cmd run --mode speed --groups 3 --per-cel 5
download\bench.cmd report %TEMP%\foc_bench\<时间戳>
download\bench.cmd chain  %TEMP%\foc_bench\<时间戳> step_pos_360
```

`run` 默认写入 `%TEMP%\foc_bench\<时间戳>\`（可用 `--out` 改）。每个
`(工况, 日志组)` 一个压缩包：`<工况>_g<组>_r<重复>.zip`，内含

| 文件 | 内容 |
|---|---|
| `meta.json` | 工况名、模式、组号、目标值、命令时间线、抖动、限制、git 版本、列名 |
| `frames.f32` | 原始帧流，**每帧 12 个小端 float32 + 4 字节帧尾 = 52 字节** |

`frames.f32` 可直接用 numpy 读：

```python
import numpy as np, json, zipfile
with zipfile.ZipFile("step_pos_360_g3_r0.zip") as z:
    meta = json.loads(z.read("meta.json"))
    data = np.frombuffer(z.read("frames.f32"), dtype="<f4").reshape(-1, 13)[:, :12]
```

或让库代劳（解压到临时目录并给出连续性统计）：

```python
import sys; sys.path.insert(0, "tools/bench")  # 从固件根目录运行
import benchlib
table, meta = benchlib.load(r"...\step_pos_360_g3_r0.zip")
print(meta["continuity"])
```

## 当前帧格式（控制频率与 USB 帧率独立）

`send X` 选择遥测组。USB 默认 1000000/8N1/DTR，divider=2；M0 电流环 10 kHz、USB 5 kHz/260 kB/s，M1 电流环 20 kHz、USB 10 kHz/520 kB/s。每帧 **12 个小端 float32 + 帧尾 `00 00 80 7F` = 52 字节**。控制频率不因遥测分频改变。

| 下标 | 内容 |
|---|---|
| 0 | 低 24 位 = `t_u24`（TIM5 微秒，16.78 s 回绕）；高 8 位 = 状态字：bit0..2 状态、bit3..6 故障、bit7 为 PWM 已开启 |
| 1 | 低 24 位 = `seq`（按电流环频率计数）；高 8 位 = 日志组号 |
| 2..11 | 该组的 10 个数据通道（见下表） |

**时间与丢帧**：`t_u24` 与 `seq` 是权威采样身份，与组号无关；组号切换不重置它们。
相邻帧应满足 `seq` 差 divider，时间差为 `1e6*divider/sample_hz` µs，入口抖动容许 ±5 µs。默认 M0 为差 2 / 200 µs，M1 为差 2 / 100 µs。`seq` 仍计数全部控制周期，回绕时间 M0 27.96 min、M1 13.98 min。
新归档写入 `sample_hz`、`usb_divider`，`benchlib.load()` 据此检验丢帧；缺少这些字段的历史包按 M1 20 kHz/divider=1 读取。`run` 工况仍针对参考电机，默认 sample_hz=20000/divider=2，不要直接对 ZH3620-1 执行。 `discover` 的速率是按帧头推算值，均匀丢帧仍可能误导推算；严格校验需给出构建参数。

状态字：状态 `0..7` = IDLE/PRECHARGE/CALIBRATE/SAVE/RUN/FAULT/OFFSET/PWM_ZERO；
故障 `0..14` = OK/SENSOR/ADC/TIMING/WINDOW/ALIGNMENT/FLASH/UART/BUS/ZERO/CURRENT/SPEED/POSITION/NUMERIC/VOLTAGE；
bit7=1 表示 PWM 已开启；OFF/PRECHARGE 由状态区分。

### 原四组通道（第 4 组诊断另见下表）

| 下标 | 组 0 raw | 组 1 current | 组 2 voltage | 组 3 control |
|---|---|---|---|---|
| 2 | `adc_raw_b` | `ib` A | `ud` V | `iq_ref` A |
| 3 | `adc_raw_c` | `ic` A | `uq` V | `iq_ref_cmd` A |
| 4 | `adc_raw_bus` | `elec_deg` ° | `ccr_a` | `pos_deg` ° |
| 5 | `v_b` V | `iq_ref` A | `ccr_b` | `pos_tgt` ° |
| 6 | `v_c` V | `integral_d` V | `ccr_c` | `rpm` |
| 7 | `v_bus` V | `integral_q` V | `duty_a` | `rpm_encoder` |
| 8 | `angle_raw_deg` ° | `ud` V | `duty_b` | `rpm_tgt` |
| 9 | `sample_us` | `uq` V | `duty_c` | `iq` A |
| 10 | `bus_v_nominal` V | `b_offset` V | `edge_limit_v` V | `mode` |
| 11 | `b_offset` V | `c_offset` V | `vec_limit_v` V | `bus_v` V |

| 下标 | 组 4 diagnostics |
|---|---|
| 2 / 3 | `iq_ref` / `iq_ref_cmd` A |
| 4 / 5 | 告警位掩码 / 最近新增告警编号 |
| 6 / 7 | 普通数值状态 / `rpm_encoder` |
| 8 / 9 | 拒绝命令累计数 / 实际 `iq` A |
| 10 / 11 | 当前停止原因（0=无） / 母线 V |

- **组 0**：ADC 原始码值与编码器**未修正**角度（二阶谐波补偿前的真值）。
  原始码值不是为了标定增益（软件无法自标定），而是把标定自由度留到将来：
  一旦有电流探头或已知负载，可以用同一批历史数据重算，不必重做实验。
  换算（`App/Hardware/bsp/bsp_adc.c`）：`v_b = adc_raw_b * 3.3/4095`，
  `v_bus = adc_raw_bus * (3.3/4095) * (41.2/2.2)`，
  `i = (v - offset) * 50`（标称，未标定）。
- **组 1**：PI 积分器与输出是电流环内部状态，主机无法从别处反算。
- **组 2**：`ccr_*` 是本周期实际生效的量化值（uint16 精确），电压余量对应
  `foc_modulate` 的采样窗口上限与 `Vbus/√3` 线性上限。
- **组 3**：`pos_deg`/`rpm` 是编码器 Ground Truth，进入无感 FOC 后仍然采集，
  用于与估计角度/速度对比。`mode`：0=Torque、1=Speed、2=Position。

同一工况在 4 个组上分别跑，`chain` 按每段采集开始后的样本序号并列导出宽表 CSV。
各次运行的全局 `seq` 不相同，这种并列仅供重复性比较，**不能视作同一物理瞬间**；
目前组 0 的电流与组 2 的 PWM 不同时采集，尚不能做严格的逐样本无感 FOC 回放。

## 命令

| 命令 | 含义 |
|---|---|
| `Iq <A>` | Torque 模式，目标 Iq，±5 A、最多两位小数 |
| `rpm <v>` | Speed 模式，目标转速，±9400 RPM |
| `pos <deg>` | Position 模式，**绝对**多圈机械角度，±1e6° |
| `zero` | 把当前位置定义为 0°（需停机且静止） |
| `stop` | 立即关断 |
| `clear` | 清已消失的故障，不自动启动 |
| `cal` | 停机重校准 |
| `send X` | 切换日志组，X = 0..5；默认 M0 5 kHz、M1 10 kHz（不改变电机状态） |
| `hello` | **仅 UART** 回 `#FOC 1.1 <状态字>`；不发到 USB 二进制流，避免破坏分帧 |

三条模式命令**命令即切模式**，不需要额外的 mode 命令。速度环/位置环是 1 kHz
外环，最终输出 Iq 参考，底层仍是现有 20 kHz Id/Iq 电流环。

默认 WARN 单次目标保持，超时只记告警，Iq 无斜坡；TRIP 才保留看门狗、限制和斜坡。历史工况脚本仍用 100 ms 重发及参考电机幅值，只适合已验证的对应台架。当前 ZH3620-1 使用下面的小电流工具。

### 电流环短测

```sh
python tools/bench/current_probe.py COM8 --out build/bench_debug/current --label trial --steps --kp 0.20 --ki 24
```

`--steps` 明确启用运动；省略时仅记录待机。工具预先 stop、检查待机母线与数据，依次发送 ±0.05/0.10/0.20 A 的短阶跃，每轮最后 stop，异常退出也尝试 stop；不发 cal/rpm/pos，不改固件限制。`--kp/--ki` 仅记载已烧录参数，不在线修改。默认 USB divider=2；全速对照固件对应 `--usb-divider 1`。`--repeat 1` 可减少重复。

`.f32` 保存精确原帧，JSON 保存命令时间、状态、连续性和统计。计时使用帧头，不用主机到达间隔；稳态窗口剔除阶跃后 30 ms。组 5 的原始记录在 divider=2 下已是两点窗口平均，不能称为未滤波 10 kHz 电流。

### 第 5 组电流窗口

| 通道 | 内容 |
|---|---|
| I2 / I3 | 最新 Iq 目标 / 命令 |
| I4 | 窗口平均 Id |
| I5 / I6 | 窗口平均 Ud / Uq |
| I7 | 窗口平均转速 |
| I8 | 窗口最小实际/请求电压比例，1=未饱和；停机为 0 |
| I9 | 窗口平均 Iq，沿用电流图绑定 |
| I10 / I11 | 累计告警 / 窗口平均母线 |

窗口在目标或状态边沿清空，最多包含 divider 次连续采样；时间戳取末端，命令取最新值。divider=1 是单样本原值。组 0–4 仍取末端单样本；组 5 的平均仅用于记录，电流 PI 没有加反馈滤波。平均不能证明物理纹波变小，也不是完整抗混叠滤波器。

## 安全性

- 主机侧默认 `--iq-limit 0.4 A`、`--rpm-limit 1000`、`--pos-limit 1200°`，
  超限的工况在**发命令之前**就会被拒绝并打印 `SKIP`。
- 每个 (工况, 组) 采集结束后立刻检查归档：出现故障帧或配置帧率下连续性丢失即
  终止整轮，不静默重试。
- 每次切换工况前执行固定前置序列（`stop` → 静置 → 位置模式 `zero`），
  保证同一工况跨组、跨重复的输入一致。
- 电机**无负载**固定台架；母线允许范围与电机额定范围不同，扩大幅值前先确认供电。

## 分析配方

- **电流/速度/位置闭环**：用组 3 的 `iq_ref_cmd`/`rpm_tgt`/`pos_tgt` 与
  `rpm`/`pos_deg` 算跟随误差、超调、静差、低速纹波。
- **Rs/Ld/Lq/磁链辨识**：组 0 的 `adc_raw_b/c/bus` + 组 1 的 `elec_deg`、
  `ud/uq`、组 2 的 `duty_*`；先在 `send 0` 下用静止注入或低速旋转取样。
- **机械参数（负载/惯量/摩擦）**：`send 0` 下施加已知 `Iq` 阶跃，用组 0 的
  未修正角度差分得快照速度，拟合加速段斜率。
- **无感 FOC 离线回放**：组 0（Vbus、原始电流码、真实角度）+ 组 2（实际施加的
  `duty`/`ccr`）足以在 PC 上重放电压与电流；组 3 提供目标轨迹与 Ground Truth 转速。
- **角度估计对比**：任何估计器都用组 0 的 `angle_raw_deg` 与组 3 的 `pos_deg`/
  `rpm` 作基准，注意前者是补偿前的真值。

未标定项：相电流绝对精度与增益、编码器内部测量延迟、`FOC_EDGE_LIMIT` 隐含的
模拟建立时间。不要把这些数字当成已标定事实。

## 外环快速响应实验

`response_probe.py` 在 COM8 上重复速度反向和位置换目标，保存每批原始 `.f32` 与 JSON。仅在已完成校准、确认负载可自由双向旋转且供电条件符合实验范围时使用；退出或越过主机实验边界会发送 stop。参数支持 `--kind speed/position/both`、`--repeat`、`--dwell`、`--rpm`、`--deg`、`--max-iq-ref`、`--max-position`。后两个只是 PC 实验中止边界，不改变固件限制。例如：

```sh
python tools/bench/response_probe.py --label response --out build/response --kind position --repeat 2 --dwell 0.4 --deg 15
```

时间从遥测目标变化计算，不从电脑写串口时间计算；位置要求 ±0.5° 连续保持 100 ms。短等待期间末值统计只取最后四分之一，避免把运动过程算成稳态。最新塑料臂带载参数、完整指标及同固件空载复测协议见[带载快速响应记录](../../../../docs/loaded-arm-fast-tuning-2026-09-28.md)。

## 解析器自检

不接硬件也可以验证解析器：

```sh
python tools/bench/selftest.py
```

覆盖帧字节往返、拆分/粘连读取、错位重同步、丢帧检测、损坏拒绝、通道表、
归档往返与工况限值校验。

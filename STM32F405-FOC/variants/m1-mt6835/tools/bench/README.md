# 主机工具

从工程根目录运行下列命令。Python依赖numpy、pyserial；运动学工具另需主机GCC，绘图需matplotlib。实机串口与VOFA不能同时占用，退出运动会关闭输出，需支撑机械腿。工具默认查询；只有明确运动选项才驱动。

## 当前双路轮腿

```powershell
# 只读检查：不会启动、回零或清故障
python variants/m1-mt6835/tools/bench/restore_position_probe.py

# 人工确认两轴在原上止点向内0°～40°、静止支撑后恢复位置
python variants/m1-mt6835/tools/bench/restore_position_probe.py --restore

# 使用当前实测几何离线解算，不访问电机
python variants/m1-mt6835/tools/bench/fivebar_tool.py --config docs/data/fivebar-current-geometry-20261016.json ik 0 160

# 只读板端状态和几何
python variants/m1-mt6835/tools/bench/foot_vertical_probe.py
```

`fivebar_tool.py` 提供 `ik/fk/workspace/export/move`；`move` 会持续发送目标并驱动，`export`只读取标定。详见[足端接口](../../../../docs/fivebar-foot-control.md)。`leg_teach.py` 仅用于重新记录物理边界，日常恢复不需要示教。

`foot_vertical_probe.py --drive --max-rpm 120` 从支撑好的合法姿态小幅起步，再验证x=0、y100～200的10/45/60/90/120 RPM与途中反向。它读取板端当前几何，按每路自身参考监督，不假设两轴等角；不会重启、回零、清故障或修改标定。必须先确认机构起始姿态、到该路径及全路径机械净空。失败立即stop，保存CSV和首次故障快照，并读取门极模式验证双路关闭。默认输出在忽略目录 `build/dual-leg/wheel-leg-ui/`。

已删除固定旧几何、旧同步假设和旧回零流程的重复试验脚本。日常操作优先[VOFA轮腿面板](../../../../docs/wheel-leg-quickstart.md)，最新实测见[报告](../../../../docs/fast-reversal-fix-20261016.md)。

`send 7`为24个JustFloat通道、末通道5007：故障、时间ms、q0/q1、FOC转速0/1、外环转速0/1、Iq0/1、Iq参考0/1、关节参考0/1、足端目标x/y、位置控制目标0/1、独立速度RMS0/1、母线、状态位、采样计数、5007。状态位1=圈数有效、2=足端所有权、4=关节终点、8=初始化成功。stop不覆盖首次故障快照；它不等于独立的预触发峰值采集。

## 保留的单电机工具

以下使用单路固件协议；不能直接用于已装车的双路轮腿。

| 工具 | 用途 |
|---|---|
| `autotune.py`、`current_acceptance.py` | R/L辨识、PI候选验收和保存；[操作](../../../../docs/motor-autotune.md) |
| `anticogging.py` | 裸轴抗齿槽采集/对比；[说明](../../../../docs/anticogging.md) |
| `longqiu_dynamic.py`、`longqiu_probe.py`、`longqiu_tune.py` | 龙邱单路小幅和连续反向 |
| `current_probe.py`、`response_probe.py`、`overview_probe.py`、`outer_probe.py` | 电流、外环和24通道总览采集 |
| `loaded_arm_probe.py`、`analyze_loaded_arm.py` | 旧塑料臂工况的采集和离线分析，需对应单路配置 |
| `bench.py`、`benchlib.py`、`selftest.py` | 12通道兼容帧、归档、离线解析自检 |

单路兼容组0～5为12个float加帧尾（52字节）；单路总览组6和双路组为24个float加帧尾（100字节），不能混用解析器。采样周期和遥测分频应以对应构建配置为准。通道定义见[单路总览](../../../../docs/vofa-quickstart.md)，主机测试入口见[测试说明](../../tests/README.md)。

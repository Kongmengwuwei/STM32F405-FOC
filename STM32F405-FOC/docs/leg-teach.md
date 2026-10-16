# 机械基准与行程维护

日常上电使用[恢复位置按钮](wheel-leg-quickstart.md)，不用重新示教或定义零位。本页用于改变安装后重新测量机械边界。

当前采用可靠的上方机械边界为q=0°，向行程内回转110°为另一侧软件边界；两端不要求杆件垂直。M0磁编随正关节角递增，M1递减，减速比7∶1。

| 项目 | M0 | M1 |
|---|---:|---:|
| 保存原止点单圈参考 | 11.220408° | 302.910885° |
| 关节正方向 | +1 | −1 |
| 驱动范围 | 0°～110° | 0°～110° |
| 足端终点范围 | 2°～108° | 2°～108° |

实际编译参考为 `dual/leg_reference_config.h`；[止点更新记录](data/leg-home-reference-20261014-refresh.json)保留用户确认与采样范围。更早的[两端原始测量](data/leg-boundaries-20261014.json)及同目录 `.A.npy`、`.B.npy`、`.reference.npy` 用于追溯，不能覆盖当前参考。原始测得总行程约113.60°／115.65°，最终按用户选择缩为110°。

机械基准与FOC电角度零点不同。修改前者会影响关节坐标及几何绑定，不能通过任意姿态下重新home解决回零失败。当前几何保存在Flash，备份见[data/fivebar-current-geometry-20261016.json](data/fivebar-current-geometry-20261016.json)。

## 重新采集

必须先支撑并关闭两路输出，VOFA释放串口。示教只跟踪手动运动，不驱动；开始前确认要重新测量原边界，而非执行日常恢复。

```powershell
python variants/m1-mt6835/tools/bench/leg_teach.py --begin --record build/leg-boundaries-new.json
# 保持上电连续采样，人工缓慢转到另一侧边界
python variants/m1-mt6835/tools/bench/leg_teach.py --read --record build/leg-boundaries-new.json
python variants/m1-mt6835/tools/bench/leg_teach.py --finish --record build/leg-boundaries-new.json
```

`leg teach`期间拒绝位置、保持、校准、置零和清故障；桥保持关闭。采样中断、CRC失败或不合法跳变使本轮记录失效，不能拼接中断前后的圈数。结束采集不自动修改编译基准或启用运动；应核对原始圈数、边界与7∶1比例，再明确更新基准、重标主动杆方向，并按小幅到全程验证。

当前已验证运动路径见[最近实测](fast-reversal-fix-20261016.md)。磁编位于电机轴，齿隙和受力变形仍可能使磁编推算角与杆件真实角度不同。

"""Generate VOFA 1.3.x gimbal presets from the checked-in native export."""
import copy
import json
from pathlib import Path

DOCS = Path(__file__).resolve().parents[1]


def write(name, data):
    (DOCS / name).write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


base = json.loads((DOCS / "vofa-overview.config.json").read_text(encoding="utf-8"))
tab_base = json.loads((DOCS / "vofa-overview.tabview.json").read_text(encoding="utf-8"))
names = [f"M{motor} {label}" for motor in range(2) for label in
         ["目标转速 rpm", "实际转速 rpm", "目标角度 °", "实际角度 °", "目标 Iq A", "实际 Iq A",
          "实际 Id A", "母线电压 V", "控制模式", "运行状态", "故障码", "告警位"]]
colors = ["#ea8c28", "#007fc4", "#ea8c28", "#007fc4", "#aa55aa", "#00805f"] * 4
selected = {2, 3, 14, 15}
channels = [{"is_draw": i in selected, "color": colors[i], "scale": 1, "yoffset": 0,
             "xoffset": 0, "decimal": 3 if i % 12 in (2, 3, 4, 5, 6) else 0 if i % 12 >= 8 else 2,
             "value": 0, "name": name} for i, name in enumerate(names)]
channels[14]["color"] = "#aa55aa"
channels[15]["color"] = "#00805f"
write("vofa-gimbal.channels.json", {"protocol": "JustFloat", "sample_hz": 1000, "frame_bytes": 100, "channels": names})
# VOFA 1.3.10 reads CSV headers using the Windows local code page.
(DOCS / "vofa-gimbal.datas.csv").write_bytes((",".join(names) + "\r\n" + ",".join(["0"] * 24) + "\r\n").encode("gbk"))

chart = copy.deepcopy(tab_base["ctx"]["tabs"][0]["widgets"][1])
chart["ctx"]["."]["ctx"] = {".": {"x": 10, "y": 685, "width": 2780, "height": 760}}
chart["ctx"]["axis_x_wave"]["ctx"]["."] = {"max_value": 14999, "min_value": 0,
    "left_index": 0, "right_index": 14999, "unit_text": "ms", "decimal": 0}
chart["ctx"]["axis_y_wave"]["ctx"]["."] = {"top_value": 100, "bottom_value": -100, "bar_index": 5, "decimal": 2}
chart["ctx"]["rbw"]["ctx"]["."]["lines"] = [2, 3, 14, 15]
tab = {"type": "tabview", "vnumber": 100, "ctx": {"tabs": [{"name": "双轴云台 · 角度控制", "widgets": [
    {"path": "GimbalControl", "type": "outside", "ctx": {".": {"ctx": {".": {"x": 10, "y": 10, "width": 2780, "height": 650}}}}}, chart]}]}}
write("vofa-gimbal.tabview.json", tab)
window = dict(tab["ctx"], is_top=False, x=0, y=0, width=3840, height=2064,
              currentIndex=0, lastIndex=0, file_url=(DOCS / "vofa-gimbal.tabview.json").as_posix())
write("vofa-gimbal.tabviews.json", {"type": "tabviews", "vnumber": 100, "ctx": [window]})


def command(name, content, intro):
    return {"name": name, "intro_on": True, "intro": intro, "hex_on": False,
            "loop_on": False, "loop_ms": 100, "loop_count": 1,
            "cmd_hex": " ".join(f"{b:02X}" for b in (content + "\r\n").encode("ascii")),
            "is_group": False, "down": False, "subCmds": []}


commands = [command("关闭两轴输出", "stop", "关闭驱动；竖直轴会失去支撑力"),
            command("两轴回软件零位", "gimbal pos 0 0", "启动两轴并保持软件零位"),
            command("M0 保持当前位置", "m0 hold", "启动水平轴位置保持"),
            command("M1 保持当前位置", "m1 hold", "启动竖直轴位置保持"),
            command("关闭 M0", "m0 stop", "只关闭水平轴"),
            command("关闭 M1", "m1 stop", "只关闭竖直轴"),
            command("清除故障", "clear", "先停机并排除故障，不会自动启动")]
group = {"name": "双轴云台", "intro_on": True, "intro": "任意角度使用画布输入框，或发送框：gimbal pos 10 -10",
         "hex_on": False, "loop_on": False, "loop_ms": 100, "loop_count": 1, "cmd_hex": "",
         "is_group": True, "down": True, "subCmds": commands}
write("vofa-gimbal.cmds.json", {"type": "cmds", "vnumber": 100, "ctx": group})
config = copy.deepcopy(base)
config["ctx"]["defaultAppTheme"]["ctx"]["lColors"] = [c["color"] for c in channels]
wave = config["ctx"]["wave_view"]["ctx"]
wave["settingsPanel"]["ctx"]["."]["settings_ctx"] = channels
wave["left_panel"]["ctx"]["command_panel"]["ctx"] = {}
serial = wave["left_panel"]["ctx"]["pal"]["ctx"]
serial["protocol_combo"] = "JustFloat"
serial["serial"].update(port="COM8", baud="1000000", dtr=True, rts=False)
tx = wave["tx_rx_zone"]["ctx"]
tx["."].update(loop_on=False, send_hex_on=False, cmd_hex="73 74 6F 70", enter_to_send=False)
tx["send_record_model"]["ctx"] = []
bar = wave["color_bar"]["ctx"]
bar["."].update(delta_t=1, left_index=0, right_index=14999, size=15000)
bar["."]["buffer_max_size_input.text"] = "15000"
bar["buffer_max_size_input"]["text"] = "15000"
write("vofa-gimbal.config.json", config)
print("Generated gimbal layout, config, channels and commands; no automatic motion.")

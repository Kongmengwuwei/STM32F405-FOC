"""Build/install the current M1 single-motor VOFA+ 1.3.10 configuration.

Uses the checked-in single-motor layout. Installation backs up existing settings
and the overview widget; run only after exiting VOFA+.
"""
import argparse
import csv
import json
import shutil
from datetime import datetime
from pathlib import Path


def read(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def command(name, text, intro):
    # In native cmds JSON, loop_on exposes the sending control. A count of 1
    # sends once when clicked; false makes the entry "invisible" in VOFA 1.3.10.
    # This is independent of the main send box's automatic-repeat switch.
    return dict(name=name, intro_on=True, intro=intro, hex_on=False,
                loop_on=True, loop_ms=100, loop_count=1,
                cmd_hex=" ".join(f"{b:02X}" for b in (text + "\r\n").encode("ascii")),
                is_group=False, down=False, subCmds=[])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--install", action="store_true")
    parser.add_argument("--vofa-dir", type=Path, default=Path("D:/APP/vofa+/x64"))
    parser.add_argument("--context-dir", type=Path,
                        default=Path("C:/Users/kongmeng/AppData/Local/vofa+/100/context"))
    args = parser.parse_args()
    docs = Path(__file__).resolve().parent.parent
    repo = docs.parent.parent
    output = repo / "exports/vofa-longqiu-single-motor-20261010"
    config = read(docs / "vofa-overview.config.json")
    wave = config["ctx"]["wave_view"]["ctx"]
    serial = wave["left_panel"]["ctx"]["pal"]["ctx"]
    serial["serial"].update(port=args.port, baud="1000000", dtr=True, rts=False)
    serial.update(protocol_combo="JustFloat", link_type_combo=0)
    tx = wave["tx_rx_zone"]["ctx"]
    tx["."].update(loop_on=False, loop_count=1, send_file=False,
                   send_hex_on=False, cmd_hex="73 74 6F 70")  # stop; append LF
    tx["append_combo"]["currentIndex"] = 1
    tx["send_record_model"]["ctx"] = [
        dict(hex_on=False, cmd=c) for c in ["stop", "send 6", "zero", "hold",
                                           "rpm 10", "rpm -10", "pos 5", "pos 0"]]
    # Clear persisted telemetry so disconnected screens never show old tests.
    channels = wave["settingsPanel"]["ctx"]["."]["settings_ctx"]
    decimals = [2, 2, 3, 3, 3, 3, 3, 3, 2, 3, 3, 0, 0, 0, 0, 0, 0, 3, 2, 2, 3, 3, 0, 0]
    for ch, name, decimal in zip(channels, read(docs / "vofa-overview.channels.json")["channels"], decimals):
        ch.update(value=0, name=name, decimal=decimal, scale=1, yoffset=0, xoffset=0)
    wave["color_bar"]["ctx"]["."].update(delta_t=0.4, size=25000,
                                             left_index=20000, right_index=24999)
    # Retain the proven 3840 x 2064 / globalScale=2 layout for this display.
    tabview = read(docs / "vofa-overview.tabview.json")
    tabview["ctx"]["tabs"][0]["name"] = "龙邱 M1 · 单电机（12 V）"
    widgets = tabview["ctx"]["tabs"][0]["widgets"]
    limits = [(400, -400), (75, -75), (8, -8)]
    bindings = [[0, 1, 18], [2, 3], [4, 5, 6, 7]]
    for widget, (top, bottom), lines in zip(widgets[1:], limits, bindings):
        assert widget["ctx"]["rbw"]["ctx"]["."]["lines"] == lines
        widget["ctx"]["axis_y_wave"]["ctx"]["."].update(top_value=top, bottom_value=bottom)
        widget["ctx"]["."]["ctx"]["."].update(y=820, height=650)
    widgets.append(dict(path="LongqiuControl", type="outside",
                        ctx={".": {"ctx": {".": dict(x=10, y=590, width=2780, height=210)}}}))
    tabviews = dict(type="tabviews", vnumber=100, ctx=[tabview["ctx"]])
    cmds = [command("停机 · 关闭输出", "stop", "关闭 M1 PWM；零速度或零位置命令不等于停机"),
            command("开启 24 通道总览", "send 6", "选择当前界面对应的数据组，不启动电机"),
            command("当前位置设为零点", "zero", "先停机；将当前磁编角度设为软件零点"),
            command("保持当前位置", "hold", "启动位置闭环，保持点击时的当前位置"),
            command("清除历史告警", "clear", "只清诊断，不会停止正在运行的电机")]
    for rpm in [10, -10, 30, -30, 60, -60, 150, -150, 300, -300, 0]:
        cmds.append(command(f"速度 {rpm:+d} rpm", f"rpm {rpm}", "启动速度闭环；单次目标持续有效，用 stop 结束"))
    for deg in [1, -1, 5, -5, 10, -10, 30, -30, 60, -60, 0]:
        cmds.append(command(f"位置 {deg:+d}°", f"pos {deg}", "相对于软件零点的绝对目标；到达后保持，用 stop 结束"))
    root = dict(name="龙邱 M1 · 12V 单电机", intro_on=True,
                intro="空载调试；命令只发一次；断开 USB 不会停机，结束用 stop。",
                hex_on=False, loop_on=True, loop_ms=100, loop_count=1,
                cmd_hex="", is_group=True, down=True, subCmds=cmds)
    assert len(channels) == 24
    for item in cmds:
        assert item["loop_count"] == 1 and bytes.fromhex(item["cmd_hex"]).endswith(b"\r\n")
    for filename, value in [("vofa+.config.json", config), ("vofa+.tabviews.json", tabviews),
                            ("vofa+.cmds.json", dict(type="cmds", vnumber=100, ctx=root))]:
        write(output / "context" / filename, value)
    write(output / "vofa-longqiu.tabview.json", tabview)
    shutil.copy2(docs / "vofa-overview.channels.json", output / "vofa-longqiu.channels.json")
    widget_out = output / "plugins/widgets/FOCOverview/FOCOverview.qml"
    widget_out.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(docs / "vofa/widgets/FOCOverview/FOCOverview.qml", widget_out)
    control_out = output / "plugins/widgets/LongqiuControl/LongqiuControl.qml"
    control_out.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(docs / "vofa/widgets/LongqiuControl/LongqiuControl.qml", control_out)
    shutil.copy2(docs / "vofa-longqiu.md", output / "使用说明.md")
    # VOFA restores this CSV after the JSON, including its old channel names.
    # VOFA needs one row to restore header names; zeros are startup placeholders.
    with (output / "vofa+.datas.csv").open("w", encoding="gbk", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow([ch["name"] for ch in channels])
        writer.writerow([0] * 24)
    print(f"Package: {output}")
    if args.install:
        if not args.context_dir.is_dir() or not (args.vofa_dir / "vofa+.exe").is_file():
            raise RuntimeError("VOFA installation/context directory does not exist")
        backup = output / "backup" / datetime.now().strftime("%Y%m%d-%H%M%S")
        shutil.copytree(args.context_dir, backup / "context")
        data_file = args.context_dir.parent.parent / "vofa+.datas.csv"
        if data_file.exists():
            shutil.copy2(data_file, backup / "vofa+.datas.csv")
        installed_widget = args.vofa_dir / "plugins/widgets/FOCOverview/FOCOverview.qml"
        if installed_widget.exists():
            (backup / "plugins/widgets/FOCOverview").mkdir(parents=True)
            shutil.copy2(installed_widget, backup / "plugins/widgets/FOCOverview/FOCOverview.qml")
        installed_widget.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(widget_out, installed_widget)
        installed_control = args.vofa_dir / "plugins/widgets/LongqiuControl/LongqiuControl.qml"
        if installed_control.exists():
            (backup / "plugins/widgets/LongqiuControl").mkdir(parents=True)
            shutil.copy2(installed_control, backup / "plugins/widgets/LongqiuControl/LongqiuControl.qml")
        installed_control.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(control_out, installed_control)
        shutil.copy2(output / "vofa+.datas.csv", data_file)
        for source in (output / "context").glob("*.json"):
            target = args.context_dir / source.name
            shutil.copy2(source, target)
            assert target.read_bytes() == source.read_bytes()
        print(f"Installed: {args.context_dir}\nBackup: {backup}")


if __name__ == "__main__":
    main()

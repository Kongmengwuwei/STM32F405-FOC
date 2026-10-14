"""Generate the dual wheel-leg VOFA package; install only with VOFA closed."""

import argparse
import csv
import json
import shutil
from datetime import datetime
from pathlib import Path
from configure_longqiu import read, write, command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--install", action="store_true")
    parser.add_argument("--vofa-dir", type=Path, default=Path("D:/APP/vofa+/x64"))
    parser.add_argument(
        "--context-dir",
        type=Path,
        default=Path("C:/Users/kongmeng/AppData/Local/vofa+/100/context"),
    )
    args = parser.parse_args()
    docs = Path(__file__).resolve().parent.parent
    output = docs.parent.parent / "exports/vofa-dual-leg"
    config = read(docs / "vofa-overview.config.json")
    wave = config["ctx"]["wave_view"]["ctx"]
    serial = wave["left_panel"]["ctx"]["pal"]["ctx"]
    serial["serial"].update(port=args.port, baud="1000000", dtr=True, rts=False)
    serial.update(protocol_combo="JustFloat", link_type_combo=0)
    tx = wave["tx_rx_zone"]["ctx"]
    tx["."].update(
        loop_on=False,
        loop_count=1,
        send_file=False,
        send_hex_on=False,
        cmd_hex="73 74 6F 70",
    )
    tx["append_combo"]["currentIndex"] = 1
    tx["send_record_model"]["ctx"] = [
        dict(hex_on=False, cmd=c)
        for c in ["stop", "send 0", "clear", "limit 10", "leg home", "leg hold"]
    ]
    channels = wave["settingsPanel"]["ctx"]["."]["settings_ctx"]
    names = read(docs / "vofa-dual-leg.channels.json")["channels"]
    assert len(channels) == len(names) == 24
    for i, (channel, name) in enumerate(zip(channels, names)):
        channel.update(
            value=0,
            name=name,
            decimal=0 if i % 12 >= 8 else 3,
            scale=1,
            yoffset=0,
            xoffset=0,
        )
    wave["color_bar"]["ctx"]["."].update(
        delta_t=1, size=25000, left_index=20000, right_index=24999
    )
    view = read(docs / "vofa-overview.tabview.json")
    tab = view["ctx"]["tabs"][0]
    tab["name"] = "轮腿 M0 / M1 · 7:1"
    tab["widgets"][0]["path"] = "DualLegOverview"
    for widget, lines, limits in zip(
        tab["widgets"][1:],
        [[0, 1, 12, 13], [2, 3, 14, 15], [4, 5, 6, 16, 17, 18]],
        [(30, -30), (115, -5), (11, -11)],
    ):
        widget["ctx"]["rbw"]["ctx"]["."]["lines"] = lines
        widget["ctx"]["axis_y_wave"]["ctx"]["."].update(
            top_value=limits[0], bottom_value=limits[1]
        )
        widget["ctx"]["."]["ctx"]["."].update(y=820, height=650)
    tab["widgets"].append(
        dict(
            path="DualLegControl",
            type="outside",
            ctx={".": {"ctx": {".": dict(x=10, y=590, width=2780, height=210)}}},
        )
    )
    commands = [
        command("停机 · 关闭两路", "stop", "关闭两路 PWM；停机需支撑机械腿"),
        command("双路总览", "send 0", "24 通道双路总览，不启动电机"),
        command("清除故障", "clear", "确认姿态后清除故障；不自动恢复驱动"),
        command("已在可靠边界 → 回零", "leg home", "先将两路都放到可靠机械边界；回零后不自动启动"),
        command("上限 10 A", "limit 10", "只在两路停机时修改；最大 10 A，硬过流11 A"),
    ]
    root = dict(
        name="轮腿双路控制",
        intro_on=True,
        intro="用控制面板持续刷新位置；先在可靠边界回零；同步目标0～110°；500 ms失联关闭两路。",
        hex_on=False,
        loop_on=True,
        loop_ms=100,
        loop_count=1,
        cmd_hex="",
        is_group=True,
        down=True,
        subCmds=commands,
    )
    write(output / "context/vofa+.config.json", config)
    write(
        output / "context/vofa+.tabviews.json",
        dict(type="tabviews", vnumber=100, ctx=[view["ctx"]]),
    )
    write(output / "context/vofa+.cmds.json", dict(type="cmds", vnumber=100, ctx=root))
    write(output / "vofa-dual-leg.tabview.json", view)
    shutil.copy2(
        docs / "vofa-dual-leg.channels.json", output / "vofa-dual-leg.channels.json"
    )
    for name in ["DualLegOverview", "DualLegControl"]:
        dest = output / f"plugins/widgets/{name}/{name}.qml"
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(docs / f"vofa/widgets/{name}/{name}.qml", dest)
    with (output / "vofa+.datas.csv").open("w", encoding="gbk", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(names)
        writer.writerow([0] * 24)
    print(f"Generated: {output}")
    if args.install:
        if not args.context_dir.is_dir() or not (args.vofa_dir / "vofa+.exe").is_file():
            raise RuntimeError("VOFA installation/context directory does not exist")
        backup = output / "backup" / datetime.now().strftime("%Y%m%d-%H%M%S")
        shutil.copytree(args.context_dir, backup / "context")
        data_file = args.context_dir.parent.parent / "vofa+.datas.csv"
        if data_file.exists():
            shutil.copy2(data_file, backup / "vofa+.datas.csv")
        for source in (output / "context").glob("*.json"):
            shutil.copy2(source, args.context_dir / source.name)
        shutil.copy2(output / "vofa+.datas.csv", data_file)
        for name in ["DualLegOverview", "DualLegControl"]:
            target = args.vofa_dir / f"plugins/widgets/{name}"
            if target.exists():
                shutil.copytree(target, backup / f"widgets/{name}")
            target.mkdir(parents=True, exist_ok=True)
            shutil.copy2(
                output / f"plugins/widgets/{name}/{name}.qml", target / f"{name}.qml"
            )
        print(f"Installed: {args.context_dir}; backup: {backup}")


if __name__ == "__main__":
    main()

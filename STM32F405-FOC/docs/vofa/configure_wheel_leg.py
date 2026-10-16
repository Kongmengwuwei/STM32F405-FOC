"""Generate/install the unified wheel-leg VOFA panel. Close VOFA before install."""

import argparse, csv, json, shutil, subprocess
from pathlib import Path
from datetime import datetime
from configure_longqiu import read, write, command


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default="COM8")
    p.add_argument("--install", action="store_true")
    p.add_argument("--vofa-dir", type=Path, default=Path("D:/APP/vofa+/x64"))
    p.add_argument(
        "--context-dir",
        type=Path,
        default=Path("C:/Users/kongmeng/AppData/Local/vofa+/100/context"),
    )
    a = p.parse_args()
    docs = Path(__file__).resolve().parent.parent
    out = docs.parent.parent / "exports/vofa-wheel-leg"
    config = read(docs / "vofa-overview.config.json")
    wave = config["ctx"]["wave_view"]["ctx"]
    serial = wave["left_panel"]["ctx"]["pal"]["ctx"]
    serial["serial"].update(port=a.port, baud="1000000", dtr=True, rts=False)
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
        dict(hex_on=False, cmd=c) for c in ["stop", "send 5", "clear"]
    ]
    names = read(docs / "vofa-wheel-leg.channels.json")["channels"]
    settings = wave["settingsPanel"]["ctx"]["."]["settings_ctx"]
    assert len(names) == len(settings) == 24
    for i, (ch, name) in enumerate(zip(settings, names)):
        ch.update(
            value=0,
            name=name,
            decimal=0 if i >= 11 and i not in [17, 18, 21, 22] else 2,
            scale=1,
            yoffset=0,
            xoffset=0,
        )
    view = read(docs / "vofa-overview.tabview.json")
    tab = view["ctx"]["tabs"][0]
    tab["name"] = "轮腿 · 直观控制"
    tab["widgets"] = [
        dict(
            path="WheelLegControl",
            type="outside",
            ctx={".": {"ctx": {".": dict(x=10, y=10, width=1400, height=780)}}},
        )
    ]
    commands = dict(
        name="轮腿控制",
        intro_on=True,
        intro="连接后使用面板按钮，不需要手输位置循环。",
        hex_on=False,
        loop_on=False,
        loop_ms=100,
        loop_count=1,
        cmd_hex="",
        is_group=True,
        down=True,
        subCmds=[
            command("停止输出", "stop", "关闭两路输出，需支撑腿部"),
            command("刷新面板", "send 5", "只读状态查询"),
            command("清除故障", "clear", "排除原因后清除，不自动启动"),
        ],
    )
    write(out / "context/vofa+.config.json", config)
    write(
        out / "context/vofa+.tabviews.json",
        dict(type="tabviews", vnumber=100, ctx=[view["ctx"]]),
    )
    write(out / "context/vofa+.cmds.json", dict(type="cmds", vnumber=100, ctx=commands))
    write(out / "vofa-wheel-leg.tabview.json", view)
    dest = out / "plugins/widgets/WheelLegControl/WheelLegControl.qml"
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(docs / "vofa/widgets/WheelLegControl/WheelLegControl.qml", dest)
    shutil.copy2(
        docs / "vofa-wheel-leg.channels.json", out / "vofa-wheel-leg.channels.json"
    )
    with (out / "vofa+.datas.csv").open("w", encoding="gbk", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(names)
        writer.writerow([0] * 24)
    print(f"Generated: {out}")
    if a.install:
        result = subprocess.run(
            [
                "powershell",
                "-NoProfile",
                "-Command",
                "@(Get-Process 'vofa+' -ErrorAction SilentlyContinue).Count",
            ],
            capture_output=True,
            text=True,
            check=True,
        )
        if result.stdout.strip() != "0":
            raise RuntimeError("Close VOFA before installation")
        if not a.context_dir.is_dir() or not (a.vofa_dir / "vofa+.exe").is_file():
            raise RuntimeError("VOFA installation/context not found")
        backup = out / "backup" / datetime.now().strftime("%Y%m%d-%H%M%S")
        shutil.copytree(a.context_dir, backup / "context")
        datafile = a.context_dir.parent.parent / "vofa+.datas.csv"
        if datafile.exists():
            shutil.copy2(datafile, backup / "vofa+.datas.csv")
        widget = a.vofa_dir / "plugins/widgets/WheelLegControl"
        if widget.exists():
            shutil.copytree(widget, backup / "widgets/WheelLegControl")
        for source in (out / "context").glob("*.json"):
            shutil.copy2(source, a.context_dir / source.name)
        shutil.copy2(out / "vofa+.datas.csv", datafile)
        widget.mkdir(parents=True, exist_ok=True)
        shutil.copy2(dest, widget / "WheelLegControl.qml")
        print(f"Installed: {a.context_dir}; backup: {backup}")


if __name__ == "__main__":
    main()

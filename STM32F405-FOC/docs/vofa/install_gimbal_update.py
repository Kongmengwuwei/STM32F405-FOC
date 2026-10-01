"""Install the gimbal widget and update its existing VOFA layout in place."""
import json
import re
import shutil
from pathlib import Path

docs = Path(__file__).resolve().parents[1]
project = docs.parent
context = Path("C:/Users/kongmeng/AppData/Local/vofa+/100/context")
widget = Path("D:/APP/vofa+/x64/plugins/widgets/GimbalControl/GimbalControl.qml")
backup = project / "build/gimbal-20261001/slider-speedup/vofa-backup"
backup.mkdir(parents=True, exist_ok=True)
targets = [context / f"vofa+.{name}.json" for name in ("config", "tabviews", "cmds")]
for target in targets + [widget]:
    destination = backup / target.name
    if target.exists() and not destination.exists():
        shutil.copy2(target, destination)


def channel_names(value):
    if isinstance(value, dict):
        name = value.get("name")
        if isinstance(name, str) and re.match(r"^M[01] ", name):
            axis = int(name[1])
            suffix = re.sub(r"^(水平|俯仰)( ·)?\s*", "", name[3:])
            value["name"] = f"M{axis} {'俯仰' if axis == 0 else '水平'} {suffix}"
        for child in value.values():
            channel_names(child)
    elif isinstance(value, list):
        for child in value:
            channel_names(child)


config = json.loads(targets[0].read_text(encoding="utf-8"))
channel_names(config)
tabs = json.loads(targets[1].read_text(encoding="utf-8"))
changed = 0
for window in tabs["ctx"]:
    for tab in window.get("tabs", []):
        if not any(w.get("path") == "GimbalControl" for w in tab.get("widgets", [])):
            continue
        for entry in tab["widgets"]:
            if entry.get("path") == "GimbalControl":
                entry["ctx"] = {".": {"ctx": {".": {"x": 0, "y": 0, "width": 2780, "height": 850}}}}
                changed += 1
            elif entry.get("path") == "WaveChart":
                entry["ctx"]["."]["ctx"]["."].update(y=885)
assert changed, "Existing gimbal layout was not found; no configuration was written"
commands = json.loads(targets[2].read_text(encoding="utf-8"))
known = {x["name"]: x for x in json.loads((docs / "vofa-gimbal.cmds.json").read_text(encoding="utf-8"))["ctx"]["subCmds"]}


def command_descriptions(value):
    if isinstance(value, dict):
        if value.get("name") in known:
            value["intro"] = known[value["name"]]["intro"]
        for child in value.values():
            command_descriptions(child)
    elif isinstance(value, list):
        for child in value:
            command_descriptions(child)


command_descriptions(commands)
for path, data in zip(targets, (config, tabs, commands)):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
shutil.copy2(docs / "vofa/widgets/GimbalControl/GimbalControl.qml", widget)
print(f"Updated {changed} gimbal panel(s); backup: {backup}")

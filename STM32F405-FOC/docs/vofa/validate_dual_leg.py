"""Offscreen Qt QA for dual-leg panel, without serial/hardware access."""

import os

os.environ["QT_QPA_PLATFORM"] = "offscreen"
os.environ["QT_QUICK_BACKEND"] = "software"
import json, time
from pathlib import Path
from PyQt5.QtCore import QObject, pyqtSignal, pyqtProperty, pyqtSlot, QUrl, QTimer
from PyQt5.QtGui import QGuiApplication
from PyQt5.QtQuick import QQuickView
from PyQt5.QtTest import QTest

HERE = Path(__file__).resolve().parent
OUT = HERE.parents[1] / "build/dual-leg/workspace/vofa-qa"
OUT.mkdir(parents=True, exist_ok=True)
MOD = OUT / "MyModules"
MOD.mkdir(exist_ok=True)
(MOD / "qmldir").write_text(
    "module MyModules\nResizableRectangle 1.0 ResizableRectangle.qml\nMyMenu 1.0 MyMenu.qml\nChMenu 1.0 ChMenu.qml\n"
)
(MOD / "ResizableRectangle.qml").write_text(
    "import QtQuick 2.12\nRectangle {property real minimumWidth:0;property real minimumHeight:0;function get_ctx(){return {}}\nfunction __set_ctx__(a,b){}}"
)
(MOD / "MyMenu.qml").write_text("import QtQuick 2.12\nItem {}")
(MOD / "ChMenu.qml").write_text(
    "import QtQuick 2.12\nQtObject {property int index:0;property var bind_obj:sys_manager.values[index]}"
)


class Value(QObject):
    changed = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.v = 0.0

    @pyqtProperty(float, notify=changed)
    def value(self):
        return self.v

    @value.setter
    def value(self, v):
        self.v = v
        self.changed.emit()


class Manager(QObject):
    changed = pyqtSignal()
    need_update = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.on = False
        self.items = [Value() for _ in range(24)]
        self.sent = []

    @pyqtProperty(bool, notify=changed)
    def connected(self):
        return self.on

    @pyqtProperty("QVariantList", constant=True)
    def values(self):
        return self.items

    @pyqtSlot(str)
    def send_string(self, s):
        self.sent.append(s.strip())
        if s.startswith("leg pos"):
            self.items[9].value = 4
            self.items[21].value = 4
        if s.startswith("stop"):
            self.items[9].value = 0
            self.items[21].value = 0


app = QGuiApplication([])
manager = Manager()
view = QQuickView()
engine = view.engine()
engine.addImportPath(str(OUT))
view.rootContext().setContextProperty("sys_manager", manager)
view.setSource(
    QUrl.fromLocalFile(str(HERE / "widgets/DualLegControl/DualLegControl.qml"))
)
assert view.status() == QQuickView.Ready, [e.toString() for e in view.errors()]
root = view.rootObject()
engine.globalObject().setProperty("testRoot", engine.newQObject(root))


def js(s):
    v = engine.evaluate(s)
    assert not v.isError(), v.toString()
    return v.toVariant()


manager.on = True
manager.changed.emit()
manager.need_update.emit()
js("testRoot.apply()")
assert not root.property("active") and not manager.sent
manager.items[11].value = 4194304
manager.items[23].value = 4194304
root.setProperty("lastDataMs", time.time() * 1000 - 1000)
js("testRoot.apply()")
assert not manager.sent
manager.need_update.emit()
target = root.findChild(QObject, "jointTarget")
assert target
for invalid in ["-1", "110.01", "NaN"]:
    target.setProperty("text", invalid)
    js("testRoot.apply()")
    assert not root.property("active")
current = root.findChild(QObject, "currentLimit")
assert current
current.setProperty("text", "10.01")
before = len(manager.sent)
js("testRoot.setCurrentLimit()")
assert len(manager.sent)==before
current.setProperty("text", "10")
js("testRoot.setCurrentLimit()")
assert manager.sent[-1]=="limit 10.00"
rate = root.findChild(QObject, "motionRate")
assert rate
rate.setProperty("currentIndex",2)
timer = QTimer()
timer.timeout.connect(manager.need_update.emit)
timer.start(50)
target.setProperty("text", "108")
js("testRoot.apply()")
QTest.qWait(250)
assert root.property("active") and manager.sent.count("leg pos 108.00 108.00") >= 3
assert "leg rate 45" in manager.sent
manager.items[23].value = 1
QTest.qWait(100)
assert not root.property("active") and manager.sent[-1] == "stop"
manager.items[23].value = 4194304
manager.need_update.emit()
ctx = js("testRoot.get_widget_ctx()")
assert "active" not in json.dumps(ctx) and "target" not in json.dumps(ctx)
js("testRoot.set_widget_ctx({ctx:{}})")
assert not root.property("active")
manager.items[3].value = 12.3
manager.items[15].value = 12.4
js("testRoot.holdHere()")
QTest.qWait(70)
assert "leg pos 12.30 12.40" in manager.sent
js("testRoot.stop()")
view.resize(2780, 210)
view.show()
QTest.qWait(100)
image = view.grabWindow()
if not image.isNull():
    image.save(str(OUT / "control.png"))
overview = QQuickView()
overview.engine().addImportPath(str(OUT))
overview.rootContext().setContextProperty("sys_manager", manager)
overview.setSource(
    QUrl.fromLocalFile(str(HERE / "widgets/DualLegOverview/DualLegOverview.qml"))
)
assert overview.status() == QQuickView.Ready, [e.toString() for e in overview.errors()]
result = {
    "qml_creation": True,
    "current_limit_10A_and_overrange_rejected": True,
    "rate_45_requested_with_position": True,
    "unhomed_start_rejected": True,
    "stale_start_rejected": True,
    "bounds_0_110": True,
    "synchronous_108_keepalive": True,
    "diagnostic_group_stops_control": True,
    "geometry_only_persistence": True,
    "context_restore_disabled": True,
    "hold_current_angles": True,
    "rendered_control": str(OUT / "control.png") if not image.isNull() else None,
    "rendering_unavailable_in_offscreen_platform": image.isNull(),
    "commands": manager.sent,
}
(OUT / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
print(json.dumps(result))

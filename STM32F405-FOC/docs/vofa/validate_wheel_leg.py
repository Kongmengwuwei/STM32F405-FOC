"""Offscreen integration/render QA for the unified VOFA panel; no COM/hardware."""

import os

os.environ["QT_QPA_PLATFORM"] = "offscreen"
os.environ["QT_QUICK_BACKEND"] = "software"
import json, time
from pathlib import Path
from PyQt5.QtCore import QObject, pyqtSignal, pyqtProperty, pyqtSlot, QUrl, QTimer
from PyQt5.QtGui import QGuiApplication, QFontDatabase, QFont
from PyQt5.QtQuick import QQuickView
from PyQt5.QtTest import QTest

HERE = Path(__file__).resolve().parent
OUT = HERE.parents[1] / "build/dual-leg/wheel-leg-ui-qa"
MOD = OUT / "MyModules"
MOD.mkdir(parents=True, exist_ok=True)
(MOD / "qmldir").write_text(
    "module MyModules\nResizableRectangle 1.0 ResizableRectangle.qml\nMyMenu 1.0 MyMenu.qml\nChMenu 1.0 ChMenu.qml\n",
    encoding="utf-8",
)
(MOD / "ResizableRectangle.qml").write_text(
    "import QtQuick 2.12\nRectangle {property real minimumWidth:0;property real minimumHeight:0;function __set_ctx__(a,b){}}",
    encoding="utf-8",
)
(MOD / "MyMenu.qml").write_text("import QtQuick 2.12\nItem {}", encoding="utf-8")
(MOD / "ChMenu.qml").write_text(
    "import QtQuick 2.12\nQtObject {property int index:0;property var bind_obj:sys_manager.values[index]}",
    encoding="utf-8",
)


class Value(QObject):
    changed = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.v = 0

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
        self.on = True
        self.items = [Value() for _ in range(24)]
        self.sent = []
        self.reject = False
        self.respond = True
        self.home_rejection = 0
        self.delay_ms = 0
        self.setup_rejection = ""
        self.save_delay_ms = 0
        self.inflight = 0
        self.setup_overlap = False

    @pyqtProperty(bool, notify=changed)
    def connected(self):
        return self.on

    @pyqtProperty("QVariantList", constant=True)
    def values(self):
        return self.items

    @pyqtSlot(str)
    def send_string(self, text):
        for line in text.strip().splitlines():
            self.sent.append(line)
            if not self.respond:
                continue
            if line.startswith("send "):
                self.need_update.emit()
                continue
            if self.inflight and (line.startswith("limit ") or line.startswith("leg rate ") or line.startswith("foot speed ")):
                self.setup_overlap = True
            self.inflight += 1
            if self.delay_ms:
                QTimer.singleShot(self.delay_ms, lambda command=line: self.accept(command))
            else:
                self.accept(line)

    def accept(self, line):
        self.inflight -= 1
        self.items[16].value += 1
        self.items[15].value = 0
        if self.setup_rejection and line.startswith(self.setup_rejection):
            self.items[15].value = 5
        elif line.startswith("leg pos "):
            if self.reject:
                self.items[15].value = 6
            else:
                q0, q1 = map(float, line.split()[2:])
                self.items[2].value = q0
                self.items[3].value = q1
                self.items[11].value = self.items[12].value = 4
        elif line.startswith("foot pos "):
            if self.reject:
                self.items[15].value = 6
                self.items[19].value = 4
            else:
                x, y = map(float, line.split()[2:])
                self.items[6].value = x
                self.items[7].value = y
                self.items[14].value = int(self.items[14].value) | 32
                self.items[11].value = self.items[12].value = 4
        elif line == "stop":
            self.items[11].value = self.items[12].value = 0
            self.items[14].value = int(self.items[14].value) & ~32
        elif line == "leg restore":
            self.items[15].value = self.home_rejection
            if not self.home_rejection:
                self.items[14].value = int(self.items[14].value) | 1 | 512
        elif line == "foot cal -90 -90":
            self.items[14].value = int(self.items[14].value) | 8
        elif line in ("foot calibrate", "foot save"):
            if self.save_delay_ms:
                self.items[14].value = (int(self.items[14].value) | 8 | 64) & ~1 & ~16 & ~512
                QTimer.singleShot(self.save_delay_ms, self.finish_save)
            else:
                self.finish_save()
        self.need_update.emit()

    def finish_save(self):
        self.items[14].value = (int(self.items[14].value) | 8 | 16) & ~1 & ~512 & ~64
        self.need_update.emit()

    def row(self, values):
        values=dict(values)
        if 14 in values:values[14] |= 1024
        for k, v in values.items():
            self.items[k].value = v
        self.need_update.emit()


app = QGuiApplication([])
for font in ["C:/Windows/Fonts/msyh.ttc", "C:/Windows/Fonts/segoeui.ttf"]:
    if Path(font).is_file():
        QFontDatabase.addApplicationFont(font)
app.setFont(QFont("Microsoft YaHei", 10))
manager = Manager()
view = QQuickView()
view.engine().addImportPath(str(OUT))
view.rootContext().setContextProperty("sys_manager", manager)
view.setSource(
    QUrl.fromLocalFile(str(HERE / "widgets/WheelLegControl/WheelLegControl.qml"))
)
assert view.status() == QQuickView.Ready, [e.toString() for e in view.errors()]
root = view.rootObject()
engine = view.engine()
engine.globalObject().setProperty("testRoot", engine.newQObject(root))


def js(code):
    result = engine.evaluate(code)
    assert not result.isError(), result.toString()
    return result.toVariant()


def child(name):
    item = root.findChild(QObject, name)
    assert item, name
    return item


manager.row({23: 1})
QTest.qWait(70)
manager.sent.clear()
js("testRoot.applyJoint()")
assert not root.property("active") and not manager.sent
manager.row({23: 5005, 14: 262, 10: 12, 21: 20, 22: 160})
QTest.qWait(70)
js("testRoot.applyJoint()")
assert not root.property("active")
manager.row({14: 775})
QTest.qWait(70)
js("testRoot.applyFoot()")
assert not root.property("active")
# Keep sample progress/freshness alive while testing the 50 ms heartbeat.
timer = QTimer()


def tick():
    manager.items[20].value += 10
    manager.need_update.emit()


timer.timeout.connect(tick)
timer.start(30)
# Refresh is a data query, not a command ACK; successful data must clear waiting.
js("testRoot.refresh()")
QTest.qWait(70)
assert not root.property("refreshPending") and "状态已更新" in root.property("message")
manager.row({14: 262, 11: 0, 12: 0})
# Previous group-5 firmware has no near-zero capability; do not send restore.
manager.items[14].value = 262
manager.need_update.emit()
QTest.qWait(70)
manager.sent.clear()
js("testRoot.setHome()")
assert not manager.sent and "更新固件" in root.property("message")
manager.row({14:262})
manager.home_rejection = 19
js("testRoot.setHome()")
QTest.qWait(70)
assert root.property("actionPending") == "" and "M0读数" in root.property("message")
manager.home_rejection = 0
manager.row({0:7.25,1:8.5,14:262})
js("testRoot.setHome()")
QTest.qWait(70)
assert "位置恢复成功" in root.property("message") and manager.items[0].value==7.25 and manager.items[1].value==8.5
count=manager.sent.count("leg restore")
js("testRoot.setHome()")
assert manager.sent.count("leg restore")==count and "跟踪仍有效" in root.property("message")
manager.row({0:3.06,1:-.81,14:799,21:23.97,22:-201.39})
QTest.qWait(70)
assert root.property("poseKnown") and js("testRoot.displayPose!==null")
assert not root.property("modelValid") and not child("runButton").property("enabled")
assert "移入" in child("startHelp").property("text")
manager.row({14:1024|2048|4096,13:1,11:5,12:5,0:float("nan"),1:float("nan")})
QTest.qWait(70)
assert root.property("startupFailed") and "M0磁编" in root.property("status")
assert not root.property("poseKnown") and not child("runButton").property("enabled")
assert "M0磁编" in child("startHelp").property("text")
manager.row({13:0,11:0,12:0,0:-.73,1:3,14:927})
QTest.qWait(70)
manager.sent.clear();js("testRoot.applyJoint()")
assert not root.property("active") and not manager.sent and "移入" in root.property("message")
manager.row({0:7.25,1:8.5,14:262})
# A connected port without returning data/ACK cannot leave an endless spinner.
timer.stop()
manager.respond = False
js("testRoot.setHome()")
QTest.qWait(1650)
assert root.property("actionPending") == "" and "未收到位置恢复" in root.property("message")
js("testRoot.refresh()")
QTest.qWait(1650)
assert not root.property("refreshPending") and "未收到板端状态" in root.property("message")
manager.respond = True
manager.row({14: 775, 11: 0, 12: 0})
timer.start(30)
QTest.qWait(70)
assert child("motionRate").property("currentIndex")==2
child("jointTarget").setProperty("text", "10")
js("testRoot.applyJoint()")
QTest.qWait(70)
assert not root.property("active") and "先标定" in root.property("message")
child("motionRate").setProperty("currentIndex",0)
js("testRoot.applyJoint()")
QTest.qWait(230)
assert root.property("active") and manager.sent.count("leg pos 10.00 10.00") >= 3
assert manager.sent[-1] == "leg pos 10.00 10.00"
# Rejected update retains the previously accepted target, without restarting.
manager.reject = True
child("jointTarget").setProperty("text", "20")
js("testRoot.applyJoint()")
QTest.qWait(70)
manager.reject = False
QTest.qWait(70)
assert root.property("active") and manager.sent[-1] == "leg pos 10.00 10.00"
assert "原目标" in root.property("message")
# An external stop must not be undone by the heartbeat.
manager.row({11: 0, 12: 0})
QTest.qWait(70)
assert not root.property("active") and manager.sent[-1] == "stop"
js("testRoot.setHome()")
QTest.qWait(70)
assert "位置跟踪仍有效" in root.property("message")
js("testRoot.calibrateSave()")
QTest.qWait(70)
assert "foot calibrate" in manager.sent
assert "已保存" in root.property("message")
js("testRoot.setHome()")
QTest.qWait(70)
manager.row({14: 927, 0: 60, 1: 60, 4: 0, 5: 160, 11: 0, 12: 0})
child("motionRate").setProperty("currentIndex",2)
child("controlMode").setProperty("currentIndex", 1)
child("footX").setProperty("text", "0")
child("footY").setProperty("text", "160")
js("testRoot.applyFoot()")
QTest.qWait(320)
assert root.property("active") and manager.sent.count("foot pos 0.00 160.00") >= 3
assert "leg rate 120" in manager.sent and "foot speed 400" in manager.sent
manager.row({13: 11})
QTest.qWait(70)
assert not root.property("active") and manager.sent[-1] == "stop"
manager.row({13: 0, 14: 927, 11: 0, 12: 0})
QTest.qWait(70)
js("testRoot.applyFoot()")
QTest.qWait(100)
timer.stop()
QTest.qWait(400)
assert not root.property("active") and manager.sent[-1] == "stop"
# Real transport returns ACKs after the click handler, unlike a synchronous
# fake. Setup must finish serially; any setup rejection must prevent movement.
manager.row({13:0,14:927,11:0,12:0})
timer.start(30)
child("controlMode").setProperty("currentIndex",0)
manager.sent.clear(); manager.delay_ms=40; manager.setup_rejection="limit "
QTest.qWait(70)
js("testRoot.applyJoint()")
QTest.qWait(150)
assert not root.property("active") and not any(s.startswith("leg pos ") for s in manager.sent)
assert "电流上限" in root.property("message")
manager.setup_rejection="";manager.sent.clear()
js("testRoot.applyJoint()")
QTest.qWait(360)
assert root.property("active") and not manager.setup_overlap
assert manager.sent[:3]==["limit 10.00","leg rate 120","leg pos 20.00 20.00"]
js("testRoot.stop()")
QTest.qWait(120)
manager.delay_ms=0
# Flash can take more than 1.5 seconds. No competing query or move may be sent
# while an accepted calibration is waiting for save completion.
manager.row({13:0,14:927,11:0,12:0})
manager.save_delay_ms=2400;manager.sent.clear()
js("testRoot.calibrateSave()")
QTest.qWait(1700)
assert root.property("actionPending")=="save" and root.property("busy")
js("testRoot.applyJoint()")
assert manager.sent==["foot calibrate"]
QTest.qWait(1000)
assert root.property("actionPending")=="" and "已保存" in root.property("message")
manager.save_delay_ms=0
# Context restore never revives targets or output.
js("testRoot.set_widget_ctx({ctx:{}})")
ctx = js("testRoot.get_widget_ctx()")
assert "active" not in json.dumps(ctx) and "commandText" not in json.dumps(ctx)
manager.row({13: 0, 14: 927, 11: 0, 12: 0, 0: 60, 1: 60, 4: 0, 5: 160})
timer.start(30)
root.setProperty("message", "位置已恢复。选择目标，再点击移动即可。")
view.resize(1400, 780)
view.show()
QTest.qWait(180)
image = view.grabWindow()
if not image.isNull():
    image.save(str(OUT / "panel.png"))
else:
    path = (OUT / "panel.png").as_posix()
    assert js(
        "testRoot.grabToImage(function(result){result.saveToFile("
        + json.dumps(path)
        + ");})"
    )
    QTest.qWait(300)
    assert (OUT / "panel.png").is_file(), "Panel render unavailable"
result = {
    "qml_ready": True,
    "fast_default_120rpm_400mm_s": True,
    "fast_joint_requires_geometry": True,
    "legacy_firmware_blocked": True,
    "unhomed_and_uncalibrated_feedback": True,
    "heartbeat_and_ack": True,
    "rejected_target_keeps_old": True,
    "manual_stop_no_restart": True,
    "near_zero_restore_preserves_angle": True,
    "missing_restore_capability_blocks_command": True,
    "repeat_restore_preserves_tracking": True,
    "outside_drive_range_blocks_start": True,
    "outside_drive_range_still_shows_pose": True,
    "startup_encoder_failure_explained": True,
    "refresh_success_and_timeout": True,
    "home_rejection_and_missing_ack_recovery": True,
    "calibrate_and_save": True,
    "async_setup_ack_serialized": True,
    "setup_rejection_blocks_motor_start": True,
    "long_flash_save_waits_without_conflicting_commands": True,
    "fault_and_stale_stop": True,
    "disabled_context_restore": True,
    "rendered": str(OUT / "panel.png"),
}
(OUT / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
# Destroy QML while its Python context and serial mock are still alive.
# Otherwise Component.onDestruction can call a deleted context at interpreter exit.
timer.stop()
view.setSource(QUrl())
view.close()
app.processEvents()
from PyQt5 import sip
sip.delete(view)
print(json.dumps(result))

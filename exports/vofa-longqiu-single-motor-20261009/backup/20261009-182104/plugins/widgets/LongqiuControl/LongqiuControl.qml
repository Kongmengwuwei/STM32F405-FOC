import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import MyModules 1.0

ResizableRectangle {
    id: root
    property string path: "LongqiuControl"
    width: 2780; height: 210
    minimumWidth: 2200; minimumHeight: 190
    color: "#f3f6fa"
    property double nowMs: Date.now()
    property double lastDataMs: 0
    readonly property bool fresh: sys_manager.connected && nowMs - lastDataMs < 1000
    property string message: "目标只发送一次；结束用停机，断开 USB 不会停机。"
    Timer { interval: 250; running: true; repeat: true; onTriggered: root.nowMs = Date.now() }
    Connections { target: sys_manager; onNeed_update: root.lastDataMs = Date.now() }
    function valid(input) {
        return /^[+-]?[0-9]+(\.[0-9]{1,3})?$/.test(input.text)
            && isFinite(Number(input.text)) && Math.abs(Number(input.text)) <= 60;
    }
    function send(line) {
        if (!sys_manager.connected) { message = "串口未连接"; return; }
        sys_manager.send_string(line + "\r\n");
        message = "已发送 " + line + "；请对照目标和运行状态确认接受。";
    }
    ColumnLayout {
        anchors.fill: parent; anchors.margins: 12; spacing: 8
        RowLayout {
            Layout.fillWidth: true
            Text { text: "龙邱 M1 · L4225-6S420-A · 12 V"; font.pixelSize: 28; font.bold: true; color: "#155786" }
            Text { text: !sys_manager.connected ? "未连接" : root.fresh ? "遥测正常" : "等待遥测"; font.pixelSize: 24; color: root.fresh ? "#16835d" : "#bd7624" }
            Item { Layout.fillWidth: true }
            Button { text: "总览 send 6"; font.pixelSize: 24; enabled: sys_manager.connected; onClicked: root.send("send 6") }
            Button { text: "当前位置设零"; font.pixelSize: 24; enabled: root.fresh; onClicked: root.send("zero") }
            Button { text: "保持当前位置"; font.pixelSize: 24; enabled: root.fresh; onClicked: root.send("hold") }
            Button { text: "停机 · 关闭输出"; font.pixelSize: 26; enabled: sys_manager.connected; onClicked: root.send("stop"); palette.buttonText: "#c93939" }
        }
        RowLayout {
            Layout.fillWidth: true
            Text { text: "速度 rpm"; font.pixelSize: 24; color: "#155786" }
            TextField { id: speed; text: "10"; Layout.preferredWidth: 210; font.pixelSize: 26; selectByMouse: true; inputMethodHints: Qt.ImhFormattedNumbersOnly }
            Button { text: "发送速度"; font.pixelSize: 24; enabled: root.fresh && root.valid(speed); onClicked: root.send("rpm " + Number(speed.text).toFixed(3)) }
            Text { text: "位置 °"; font.pixelSize: 24; color: "#155786" }
            TextField { id: position; text: "5"; Layout.preferredWidth: 210; font.pixelSize: 26; selectByMouse: true; inputMethodHints: Qt.ImhFormattedNumbersOnly }
            Button { text: "发送位置"; font.pixelSize: 24; enabled: root.fresh && root.valid(position); onClicked: root.send("pos " + Number(position.text).toFixed(3)) }
            Button { text: "回软件零点"; font.pixelSize: 24; enabled: root.fresh; onClicked: root.send("pos 0") }
            Item { Layout.fillWidth: true }
            Text { text: "速度 ±60 rpm / 位置 ±60° · 空载调试"; font.pixelSize: 24; color: "#53657a" }
        }
        Text { text: root.message; font.pixelSize: 24; color: "#53657a"; Layout.fillWidth: true; elide: Text.ElideRight }
    }
    function get_widget_ctx() { return {"path": path, "ctx": {".": {"ctx": get_ctx()}}}; }
    function set_widget_ctx(value) { __set_ctx__(root, value.ctx); }
}

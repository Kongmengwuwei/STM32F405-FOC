import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import MyModules 1.0

ResizableRectangle {
    id: root
    property string path: "DualLegControl"
    width: 2780; height: 210
    minimumWidth: 2400; minimumHeight: 190
    color: "#f3f6fa"
    property bool active: false
    property double lastDataMs: 0
    property double nowMs: Date.now()
    property double startMs: 0
    property double target0: 0
    property double target1: 0
    property string message: "首次启动或基准失效需在可靠止点回零；基准有效可从当前位置控制。建议 2～108°，停机需支撑。"
    readonly property bool fresh: sys_manager.connected && nowMs - lastDataMs < 700
    MyMenu { ChMenu { id: state0; index: 9 } }
    MyMenu { ChMenu { id: state1; index: 21 } }
    MyMenu { ChMenu { id: fault0; index: 10 } }
    MyMenu { ChMenu { id: fault1; index: 22 } }
    MyMenu { ChMenu { id: tag; index: 23 } }
    MyMenu { ChMenu { id: flags0; index: 11 } }
    MyMenu { ChMenu { id: actual0; index: 3 } }
    MyMenu { ChMenu { id: actual1; index: 15 } }
    readonly property bool homed: (Math.round(value(flags0)) & 4194304) !== 0
        && (Math.round(value(tag)) & 4194304) !== 0
    function value(channel) { return channel.bind_obj ? Number(channel.bind_obj.value) : NaN; }
    function healthy() {
        return sys_manager.connected && Date.now() - lastDataMs < 700
            && (Math.round(value(tag)) & 1) === 0
            && value(fault0) === 0 && value(fault1) === 0;
    }
    function ready() {
        return healthy() && homed && value(actual0) >= -1 && value(actual0) <= 111
            && value(actual1) >= -1 && value(actual1) <= 111 && (value(state0) === 0 || value(state0) === 4)
            && (value(state1) === 0 || value(state1) === 4);
    }
    function valid(input, maximum) {
        return /^[+-]?[0-9]+(\.[0-9]{1,2})?$/.test(input.text)
            && isFinite(Number(input.text)) && Number(input.text) >= 0 && Number(input.text) <= maximum;
    }
    function send(line) { if (sys_manager.connected) sys_manager.send_string(line + "\r\n"); }
    function stop() { active = false; send("stop"); }
    function command() { send("leg pos " + target0.toFixed(2) + " " + target1.toFixed(2)); }
    function holdHere() {
        if (!ready()) return;
        target0 = value(actual0); target1 = value(actual1);
        if (target0 < 0 || target0 > 110 || target1 < 0 || target1 > 110) return;
        startMs = Date.now(); active = true; command();
        message = "两路保持当前姿态；同步目标只在点击更新时发送。";
    }
    function setCurrentLimit() {
        if (active || !fresh || !valid(current, 10) || Number(current.text)<0.2
            || value(state0)!==0 || value(state1)!==0) return;
        send("limit " + Number(current.text).toFixed(2));
        message="上限已发送；范围 0.2～10 A，硬过流 11 A。";
    }
    function apply() {
        if (!ready() || !valid(angle0, 110)) return;
        target0 = Number(angle0.text); target1 = target0;
        send("leg rate " + [10,30,45][motionRate.currentIndex]);
        if (!active) startMs = Date.now();
        active = true; command();
        message = "双路保持中；修改角度后点击更新目标，结束点击停机。";
    }
    Timer { interval: 100; repeat: true; running: true; onTriggered: root.nowMs = Date.now() }
    Connections { target: sys_manager; onNeed_update: root.lastDataMs = Date.now() }
    Timer {
        interval: 50; repeat: true; running: root.active
        onTriggered: {
            if (!root.healthy()) { root.stop(); root.message = "遥测中断或故障，已停机。"; return; }
            if (Date.now() - root.startMs > 1000 && (root.value(state0) !== 4 || root.value(state1) !== 4)) {
                root.stop(); root.message = "位置命令未接受；请检查两路校准和行程。"; return;
            }
            root.command();
        }
    }
    Component.onDestruction: root.stop()
    ColumnLayout {
        anchors.fill: parent; anchors.margins: 12; spacing: 8
        RowLayout {
            Text { text: "轮腿 M0 / M1 · 12 V · 7:1"; font.family: "Microsoft YaHei UI"; font.pixelSize: 28; font.bold: true; color: "#155786" }
            Text { text: root.active ? "双路位置保持" : root.fresh ? (root.homed ? (root.ready() ? "已回零 / 就绪" : "已回零 / 检查姿态") : "未回零") : "未连接 / 等待遥测"; font.pixelSize: 24; color: root.active ? "#16835d" : "#53657a" }
            Item { Layout.fillWidth: true }
            Button { text: "双路总览"; font.pixelSize: 24; onClicked: { root.stop(); root.send("send 0"); } }
            Button { text: "已在可靠边界 → 回零"; font.pixelSize: 24;
                enabled: !root.active && root.healthy() && root.value(state0) === 0 && root.value(state1) === 0;
                onClicked: { root.send("leg home"); root.message="回零请求已发送；必须已将两路放到可靠机械边界。"; } }
            Button { text: "清除故障"; font.pixelSize: 24; enabled: !root.active && sys_manager.connected; onClicked: root.send("clear") }
            Button { text: "停机 · 关闭两路"; font.pixelSize: 26; onClicked: { root.stop(); root.message = "两路已关闭；请支撑机械腿。"; } palette.buttonText: "#c93939" }
        }
        RowLayout {
            Text { text: "同步目标 °"; font.pixelSize: 24 }
            TextField { id: angle0; objectName: "jointTarget"; text: "2"; Layout.preferredWidth: 160; font.pixelSize: 26; selectByMouse: true }
            Text { text: "轨迹速度"; font.pixelSize: 24 }
            ComboBox { id: motionRate; objectName: "motionRate"; model: ["8.6°/s", "25.7°/s", "38.6°/s"]; currentIndex: 2; Layout.preferredWidth: 180; font.pixelSize: 24 }
            Button { text: root.active ? "更新同步目标" : "开始同步运动"; font.pixelSize: 24; enabled: root.fresh && root.ready() && root.valid(angle0,110); onClicked: root.apply() }
            Button { text: "原位保持"; font.pixelSize: 24; enabled: root.fresh && root.ready(); onClicked: root.holdHere() }
            Button { text: "回到 2°"; font.pixelSize: 24; enabled: root.fresh && root.ready(); onClicked: { angle0.text="2"; root.apply(); } }
            Item { Layout.fillWidth: true }
            Text { text: "每路电流上限 A"; font.pixelSize: 24 }
            TextField { id: current; objectName: "currentLimit"; text: "10"; Layout.preferredWidth: 140; font.pixelSize: 26; selectByMouse: true }
            Button { text: "设置上限"; font.pixelSize: 24; enabled: !root.active && root.fresh && root.valid(current,10) && Number(current.text)>=0.2 && root.value(state0)===0 && root.value(state1)===0; onClicked: root.setCurrentLimit() }
        }
        Text { text: root.message; font.pixelSize: 24; color: "#53657a"; Layout.fillWidth: true; elide: Text.ElideRight }
    }
    // Persist geometry only; drive enable and targets never survive a restart.
    function get_widget_ctx() { return {"path": path, "ctx": {".": {"ctx": {".": {"x": x, "y": y, "width": width, "height": height}}}}}; }
    function set_widget_ctx(value) { root.stop(); __set_ctx__(root, value.ctx); root.stop(); }
}

import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import MyModules 1.0

ResizableRectangle {
    id: root
    property string path: "GimbalControl"
    width: 2780
    height: 850
    minimumWidth: 1800
    minimumHeight: 800
    color: "#f3f6fa"
    property double lastDataMs: 0
    property double nowMs: Date.now()
    property bool fresh: sys_manager.connected && lastDataMs > 0 && nowMs - lastDataMs < 1500
    property string message: "拖动滑条即可调角；输入框需点击发送；连接不会启动电机"
    readonly property double m0Limit: 1000000
    readonly property double m1Limit: 85
    readonly property double m0SliderLimit: m0Limit > 85 ? 180 : 85
    readonly property double m1SliderLimit: m1Limit > 85 ? 180 : 85
    property double pendingM0: NaN
    property double pendingM1: NaN
    property var labels: ["目标转速 rpm", "实际转速 rpm", "目标角度 °", "实际角度 °",
        "目标 Iq A", "实际 Iq A", "实际 Id A", "母线电压 V", "控制模式", "运行状态", "故障码", "告警位"]
    property var decimals: [2,2,3,3,3,3,3,2,0,0,0,0]
    Timer { interval: 250; running: true; repeat: true; onTriggered: root.nowMs = Date.now() }
    Connections { target: sys_manager; onNeed_update: root.lastDataMs = Date.now() }
    onFreshChanged: if (!fresh) clearPending()
    Timer { id: sliderSendTimer; interval: 40; onTriggered: root.flushPending() }
    function clearPending() {
        pendingM0 = NaN; pendingM1 = NaN;
        if (sliderSendTimer) sliderSendTimer.stop();
    }
    function queueAngle(axis, value) {
        if (!fresh) return;
        if (axis === 0) pendingM0 = value; else pendingM1 = value;
        if (!sliderSendTimer.running) sliderSendTimer.start();
    }
    function flushPending() {
        var a = pendingM0, b = pendingM1;
        clearPending();
        if (!fresh) return;
        if (isFinite(a) && isFinite(b)) send("gimbal pos " + a.toFixed(2) + " " + b.toFixed(2));
        else if (isFinite(a)) send("m0 pos " + a.toFixed(2));
        else if (isFinite(b)) send("m1 pos " + b.toFixed(2));
    }
    function valid(input, limit) {
        return /^[+-]?[0-9]+(\.[0-9]{1,2})?$/.test(input.text) &&
            isFinite(Number(input.text)) && Math.abs(Number(input.text)) <= limit;
    }
    function send(line) {
        clearPending();
        if (!sys_manager.connected) { message = "串口未连接"; return; }
        sys_manager.send_string(line + "\r\n");
        message = "已发送 " + line + "；请对照目标角度确认接受";
    }
    function angle(input) { return Number(input.text).toFixed(2); }
    function format(index, obj) {
        if (!fresh || !obj) return "—";
        var value = Number(obj.value), local = index % 12;
        if (!isFinite(value)) return "无效";
        if (local === 8) return ["电流", "速度", "位置"][Math.round(value)] || String(value);
        if (local === 9) return ["待机", "预充电", "校准", "保存", "运行", "故障", "校零", "PWM校零"][Math.round(value)] || String(value);
        return value.toFixed(decimals[local]);
    }
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 12
        RowLayout {
            Layout.fillWidth: true
            Text { text: "双轴云台"; font.family: "Microsoft YaHei UI"; font.pixelSize: 32; font.bold: true; color: "#155786" }
            Text { text: !sys_manager.connected ? "未连接" : root.fresh ? "遥测正常" : "等待 / 遥测中断"; font.family: "Microsoft YaHei UI"; font.pixelSize: 24; color: root.fresh ? "#16835d" : "#bd7624" }
            Item { Layout.fillWidth: true }
            Button { text: "关闭两轴输出"; font.family: "Microsoft YaHei UI"; font.pixelSize: 26; enabled: sys_manager.connected; onClicked: root.send("stop"); palette.buttonText: "#c93939" }
        }
        RowLayout {
            Layout.fillWidth: true
            Text { text: "M0 俯仰 °"; font.family: "Microsoft YaHei UI"; font.pixelSize: 26; color: "#155786" }
            TextField { id: yaw; text: "0"; Layout.preferredWidth: 230; font.family: "Microsoft YaHei UI"; font.pixelSize: 28; selectByMouse: true; inputMethodHints: Qt.ImhFormattedNumbersOnly }
            Button { text: "发送 M0"; font.family: "Microsoft YaHei UI"; font.pixelSize: 24; enabled: root.fresh && root.valid(yaw, root.m0Limit); onClicked: root.send("m0 pos " + root.angle(yaw)) }
            Text { text: "M1 水平 °"; font.family: "Microsoft YaHei UI"; font.pixelSize: 26; color: "#155786" }
            TextField { id: pitch; text: "0"; Layout.preferredWidth: 210; font.family: "Microsoft YaHei UI"; font.pixelSize: 28; selectByMouse: true; inputMethodHints: Qt.ImhFormattedNumbersOnly }
            Button { text: "发送 M1"; font.family: "Microsoft YaHei UI"; font.pixelSize: 24; enabled: root.fresh && root.valid(pitch, root.m1Limit); onClicked: root.send("m1 pos " + root.angle(pitch)) }
            Button { text: "同时发送两轴"; font.family: "Microsoft YaHei UI"; font.pixelSize: 24; enabled: root.fresh && root.valid(yaw, root.m0Limit) && root.valid(pitch, root.m1Limit); onClicked: root.send("gimbal pos " + root.angle(yaw) + " " + root.angle(pitch)) }
            Item { Layout.fillWidth: true }
            Button { text: "两轴回软件零位"; font.family: "Microsoft YaHei UI"; font.pixelSize: 24; enabled: root.fresh; onClicked: root.send("gimbal pos 0 0") }
        }
        RowLayout {
            Layout.fillWidth: true
            Text { text: "俯仰 M0  −" + root.m0SliderLimit + "°"; font.family: "Microsoft YaHei UI"; font.pixelSize: 24; color: "#155786" }
            Slider {
                id: m0Slider; Layout.fillWidth: true
                from: -root.m0SliderLimit; to: root.m0SliderLimit; stepSize: 0.1
                value: root.valid(yaw, root.m0Limit) ? Number(yaw.text) : 0
                enabled: root.fresh
                onMoved: { yaw.text = value.toFixed(2); root.queueAngle(0, value); }
                onPressedChanged: if (!pressed) root.flushPending()
            }
            Text { text: "+" + root.m0SliderLimit + "°"; font.family: "Microsoft YaHei UI"; font.pixelSize: 24; color: "#155786" }
            Button { text: "M0 回零"; font.family: "Microsoft YaHei UI"; font.pixelSize: 24; enabled: root.fresh; onClicked: { yaw.text = "0"; root.send("m0 pos 0"); } }
        }
        RowLayout {
            Layout.fillWidth: true
            Text { text: "水平 M1  −" + root.m1SliderLimit + "°"; font.family: "Microsoft YaHei UI"; font.pixelSize: 24; color: "#155786" }
            Slider {
                id: m1Slider; Layout.fillWidth: true
                from: -root.m1SliderLimit; to: root.m1SliderLimit; stepSize: 0.1
                value: root.valid(pitch, root.m1Limit) ? Number(pitch.text) : 0
                enabled: root.fresh
                onMoved: { pitch.text = value.toFixed(2); root.queueAngle(1, value); }
                onPressedChanged: if (!pressed) root.flushPending()
            }
            Text { text: "+" + root.m1SliderLimit + "°"; font.family: "Microsoft YaHei UI"; font.pixelSize: 24; color: "#155786" }
            Button { text: "M1 回零"; font.family: "Microsoft YaHei UI"; font.pixelSize: 24; enabled: root.fresh; onClicked: { pitch.text = "0"; root.send("m1 pos 0"); } }
        }
        Text { text: root.message; font.family: "Microsoft YaHei UI"; font.pixelSize: 24; color: "#53657a"; Layout.fillWidth: true; elide: Text.ElideRight }
        GridLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            columns: 6
            columnSpacing: 12
            rowSpacing: 12
            Repeater {
                model: 24
                Rectangle {
                    id: card
                    property int channelIndex: index
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    color: "white"
                    radius: 8
                    border.color: "#dce4ed"
                    MyMenu { ChMenu { id: channel; index: card.channelIndex } }
                    Text {
                        x: 14; y: 10; width: parent.width - 28
                        text: (card.channelIndex < 12 ? "M0 俯仰 · " : "M1 水平 · ") + root.labels[card.channelIndex % 12]
                        color: "#53657a"; font.family: "Microsoft YaHei UI"; font.pixelSize: 24; elide: Text.ElideRight
                    }
                    Text {
                        x: 14; y: parent.height * 0.43; width: parent.width - 28
                        text: root.format(card.channelIndex, channel.bind_obj)
                        color: ((card.channelIndex % 12 >= 10) && root.fresh && channel.bind_obj && channel.bind_obj.value !== 0) ? "#c93939" : "#155786"
                        font.family: "Cascadia Mono"; font.pixelSize: 32; font.bold: true; elide: Text.ElideRight
                    }
                }
            }
        }
        Text {
            text: (root.m0Limit > 85 ? "M0：可多圈" : "M0：±85°目标 / ±90°保护") + "    " + (root.m1Limit > 85 ? "M1：可多圈" : "M1：±85°目标 / ±90°保护") + "    停机或断电后俯仰轴可能下落"
            font.family: "Microsoft YaHei UI"; font.pixelSize: 24; color: "#53657a"; Layout.fillWidth: true; elide: Text.ElideRight
        }
    }
    // Targets are intentionally not persisted: loading a layout never sends.
    function get_widget_ctx() {
        return {"path": path, "ctx": {".": {"ctx": {".": {"x": x, "y": y, "width": width, "height": height}}}}};
    }
    function set_widget_ctx(value) { clearPending(); __set_ctx__(root, value.ctx); clearPending(); }
}

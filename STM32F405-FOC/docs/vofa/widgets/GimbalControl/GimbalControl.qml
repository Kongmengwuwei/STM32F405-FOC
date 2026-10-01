import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import MyModules 1.0

ResizableRectangle {
    id: root
    property string path: "GimbalControl"
    width: 2780
    height: 650
    minimumWidth: 1800
    minimumHeight: 600
    color: "#f3f6fa"
    property double lastDataMs: 0
    property double nowMs: Date.now()
    property bool fresh: sys_manager.connected && lastDataMs > 0 && nowMs - lastDataMs < 1500
    property string message: "输入机械角度后点击发送；连接不会启动电机"
    property var labels: ["目标转速 rpm", "实际转速 rpm", "目标角度 °", "实际角度 °",
        "目标 Iq A", "实际 Iq A", "实际 Id A", "母线电压 V", "控制模式", "运行状态", "故障码", "告警位"]
    property var decimals: [2,2,3,3,3,3,3,2,0,0,0,0]
    Timer { interval: 250; running: true; repeat: true; onTriggered: root.nowMs = Date.now() }
    Connections { target: sys_manager; onNeed_update: root.lastDataMs = Date.now() }
    function valid(input, limit) {
        return /^[+-]?[0-9]+(\.[0-9]{1,2})?$/.test(input.text) &&
            isFinite(Number(input.text)) && Math.abs(Number(input.text)) <= limit;
    }
    function send(line) {
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
            Text { text: "M0 水平 °"; font.family: "Microsoft YaHei UI"; font.pixelSize: 26; color: "#155786" }
            TextField { id: yaw; text: "0"; Layout.preferredWidth: 230; font.family: "Microsoft YaHei UI"; font.pixelSize: 28; selectByMouse: true; inputMethodHints: Qt.ImhFormattedNumbersOnly }
            Button { text: "发送 M0"; font.family: "Microsoft YaHei UI"; font.pixelSize: 24; enabled: root.fresh && root.valid(yaw, 1000000); onClicked: root.send("m0 pos " + root.angle(yaw)) }
            Text { text: "M1 俯仰 °"; font.family: "Microsoft YaHei UI"; font.pixelSize: 26; color: "#155786" }
            TextField { id: pitch; text: "0"; Layout.preferredWidth: 210; font.family: "Microsoft YaHei UI"; font.pixelSize: 28; selectByMouse: true; inputMethodHints: Qt.ImhFormattedNumbersOnly }
            Button { text: "发送 M1"; font.family: "Microsoft YaHei UI"; font.pixelSize: 24; enabled: root.fresh && root.valid(pitch, 85); onClicked: root.send("m1 pos " + root.angle(pitch)) }
            Button { text: "同时发送两轴"; font.family: "Microsoft YaHei UI"; font.pixelSize: 24; enabled: root.fresh && root.valid(yaw, 1000000) && root.valid(pitch, 85); onClicked: root.send("gimbal pos " + root.angle(yaw) + " " + root.angle(pitch)) }
            Item { Layout.fillWidth: true }
            Button { text: "两轴回软件零位"; font.family: "Microsoft YaHei UI"; font.pixelSize: 24; enabled: root.fresh; onClicked: root.send("gimbal pos 0 0") }
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
                        text: (card.channelIndex < 12 ? "M0 · " : "M1 · ") + root.labels[card.channelIndex % 12]
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
            text: "M0：连续多圈角度    M1：上电中心 ±85°目标 / ±90°保护    停机或断电后竖直轴可能下落"
            font.family: "Microsoft YaHei UI"; font.pixelSize: 24; color: "#53657a"; Layout.fillWidth: true; elide: Text.ElideRight
        }
    }
    // Targets are intentionally not persisted: loading a layout never sends.
    function get_widget_ctx() { return {"path": path, "ctx": {".": {"ctx": get_ctx()}}}; }
    function set_widget_ctx(value) { __set_ctx__(root, value.ctx); }
}

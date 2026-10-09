import QtQuick 2.12
import QtQuick.Layouts 1.12
import MyModules 1.0

// Display only: this widget never sends motor commands.
ResizableRectangle {
    id: root
    property string path: "FOCOverview"
    color: "#f3f6fa"
    width: 2760
    height: 570
    minimumWidth: 1000
    minimumHeight: 300
    property var names: ["目标转速 rpm", "实际转速 rpm", "目标位置 °", "实际位置 °",
        "目标 Iq A", "实际 Iq A", "目标 Id A", "实际 Id A", "母线电压 V",
        "Ud 电压 V", "Uq 电压 V", "控制模式", "运行状态", "故障编号",
        "告警位掩码", "最近告警", "命令拒绝次数", "电压可用比例",
        "斜坡转速 rpm", "速度环误差 rpm", "位置误差 °", "Iq 误差 A",
        "采样时间 us（24位）", "采样序号（24位）"]
    property var decimals: [2,2,3,3,3,3,3,3,2,3,3,0,0,0,0,0,0,3,2,2,3,3,0,0]
    function format(index, obj) {
        if (!obj || !sys_manager.connected) return "—";
        var v = obj.value;
        if (!isFinite(v)) return "无效";
        if (index === 11) return ["电流", "速度", "位置"][Math.round(v)] || String(v);
        if (index === 12) return ["待机", "预充电", "校准", "保存", "运行", "故障", "校零", "PWM校零"][Math.round(v)] || String(v);
        return Number(v).toFixed(root.decimals[index]);
    }
    GridLayout {
        anchors.fill: parent
        anchors.margins: 12
        anchors.bottomMargin: 48
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
                MyMenu {
                ChMenu {
                    id: channel
                    // Bind the native index; assigning ctx does not select data.
                    index: card.channelIndex
                }
                }
                Text {
                    x: 16; y: 12
                    width: parent.width - 32
                    text: root.names[index]
                    color: "#53657a"
                    font.family: "Microsoft YaHei UI"
                    font.pixelSize: 24
                    elide: Text.ElideRight
                }
                Text {
                    x: 16; y: parent.height * 0.43
                    width: parent.width - 32
                    text: root.format(index, channel.bind_obj)
                    color: ((index === 13 || index === 14) && channel.bind_obj && channel.bind_obj.value !== 0) ? "#c93939" : "#155786"
                    font.family: "Cascadia Mono"
                    font.pixelSize: 32
                    font.bold: true
                    elide: Text.ElideRight
                }
            }
        }
    }
    Row {
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        height: 40
        Repeater {
            model: ["转速：目标 / 实际 / 斜坡参考（rpm）", "位置：目标 / 实际（机械度）", "电流：目标 / 实际 Iq、Id（A）"]
            Text {
                width: root.width / 3
                text: modelData
                color: "#155786"
                font.family: "Microsoft YaHei UI"
                font.pixelSize: 24
                horizontalAlignment: Text.AlignHCenter
            }
        }
    }
    function get_widget_ctx() { return {"path": path, "ctx": {".": {"ctx": get_ctx()}}}; }
    function set_widget_ctx(value) { __set_ctx__(root, value.ctx); }
}

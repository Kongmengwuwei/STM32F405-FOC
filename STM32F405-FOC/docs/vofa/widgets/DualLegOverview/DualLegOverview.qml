import QtQuick 2.12
import QtQuick.Layouts 1.12
import MyModules 1.0

// Display only: this widget never sends motor commands.
ResizableRectangle {
    id: root
    property string path: "DualLegOverview"
    color: "#f3f6fa"
    width: 2760
    height: 570
    minimumWidth: 1000
    minimumHeight: 300
    property var names: ["M0 目标关节转速 rpm", "M0 实际关节转速 rpm", "M0 目标关节角度 °", "M0 实际关节角度 °", "M0 目标 Iq A", "M0 实际 Iq A", "M0 实际 Id A", "M0 母线电压 V", "M0 控制模式", "M0 运行状态", "M0 故障编号", "M0 告警位掩码", "M1 目标关节转速 rpm", "M1 实际关节转速 rpm", "M1 目标关节角度 °", "M1 实际关节角度 °", "M1 目标 Iq A", "M1 实际 Iq A", "M1 实际 Id A", "M1 母线电压 V", "M1 控制模式", "M1 运行状态", "M1 故障编号", "M1 告警位掩码"]
    property var decimals: [2,2,3,3,3,3,3,2,0,0,0,0,2,2,3,3,3,3,3,2,0,0,0,0]
    function format(index, obj) {
        if (!obj || !sys_manager.connected) return "—";
        var v = obj.value;
        if (!isFinite(v)) return "无效";
        if (index === 8 || index === 20) return ["电流", "速度", "位置"][Math.round(v)] || String(v);
        if (index === 9 || index === 21) return ["待机", "预充电", "校准", "保存", "运行", "故障", "校零", "PWM校零"][Math.round(v)] || String(v);
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
                    color: ((index === 10 || index === 11 || index === 22 || index === 23) && channel.bind_obj && channel.bind_obj.value !== 0) ? "#c93939" : "#155786"
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
            model: ["转速：两路目标 / 实际（关节 rpm）", "位置：两路目标 / 实际（关节度）", "电流：两路目标 / 实际 Iq、Id（A）"]
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

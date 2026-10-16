import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import MyModules 1.0

ResizableRectangle {
    id: root
    property string path: "WheelLegControl"
    width: 1400; height: 780; minimumWidth: 1100; minimumHeight: 740
    color: "#f3f6fa"
    readonly property var motorRateProfiles: [10,60,120]
    readonly property var footSpeedProfiles: [20,200,400]
    property bool active: false
    property string commandText: ""
    property string pendingText: ""
    property double pendingSequence: -1
    property double pendingSince: 0
    property double lastDataMs: 0
    property double lastProgressMs: 0
    property double lastSequence: -1
    property double lastQueryMs: 0
    property double receiveCount: 0
    property bool refreshPending: false
    property double refreshSequence: -1
    property double refreshSince: 0
    property double actionSince: 0
    property var stageCommands: []
    property int stageIndex: 0
    property string stageTarget: ""
    property double stageSequence: -1
    property double stageSince: 0
    readonly property bool busy: pendingText!=="" || actionPending!=="" || stageCommands.length>0
    property double nowMs: Date.now()
    property bool observedRun: false
    property string actionPending: ""
    property double actionSequence: -1
    property string message: "连接后，将两杆放在上止点向下0°～40°内，点击恢复腿部位置。"
    property var workspacePoints: []
    property double workspaceZero0: NaN
    property double workspaceZero1: NaN
    Repeater {
        id: bindings; model: 24
        Item { id: bindingItem; property int channelIndex: index; MyMenu { ChMenu { id: ch; index: bindingItem.channelIndex } }
            property var channel: ch.bind_obj }
    }
    function v(index) { var item = bindings.itemAt(index); return item && item.channel ? Number(item.channel.value) : NaN; }
    readonly property bool connected: sys_manager.connected
    readonly property bool fresh: connected && nowMs-lastDataMs<350
    readonly property bool firmware: fresh && v(23)===5005
    readonly property int flags: firmware ? Math.round(v(14)) : 0
    readonly property bool supportsRestore: (flags&1024)!==0
    readonly property bool driveRange: homed && v(0)>=-0.005 && v(1)>=-0.005 && v(0)<=110.005 && v(1)<=110.005
    readonly property bool homed: (flags&1)!==0
    readonly property bool startupFailed: (flags&2048)!==0
    readonly property bool poseKnown: firmware && homed && isFinite(v(0)) && isFinite(v(1)) && isFinite(v(21)) && isFinite(v(22))
    readonly property var displayPose: poseKnown ? solveForward(v(0),v(1)) : null
    readonly property bool modelValid: firmware && (flags&128)!==0 && isFinite(v(4)) && isFinite(v(5))
    function startupReason() {
        var lanes=flags&12288;
        return lanes===12288 ? "两路磁编初始化失败：检查供电/插头，修复后重启" : lanes===4096 ? "M0磁编初始化失败：检查供电及SPI1接线，修复后重启" : lanes===8192 ? "M1磁编初始化失败：检查供电及SPI3接线，修复后重启" : "硬件初始化失败：检查供电及连接后重启";
    }
    readonly property bool calibrated: (flags&8)!==0
    readonly property bool saved: (flags&16)!==0
    readonly property bool saving: (flags&64)!==0
    readonly property bool idle: firmware && v(11)===0 && v(12)===0
    readonly property bool stopped: firmware && (v(11)===0 || v(11)===5) && (v(12)===0 || v(12)===5)
    readonly property bool healthy: firmware && v(13)===0 && (flags&256)!==0
    readonly property bool ready: healthy && (flags&512)!==0 && driveRange && !saving
    readonly property string status: !connected ? "未连接串口" : actionPending==="save" ? "标定保存中，等待板端完成" : !fresh ? "等待遥测" : !firmware ? "板上固件版本不匹配，请烧录新版"
        : startupFailed ? startupReason() : v(13)!==0 ? "故障："+faultName(v(13)) : saving ? "正在保存标定" : !homed ? "位置未识别：请放在上止点附近"
        : (flags&2)===0 ? "电机电角度未校准" : (flags&4)===0 ? "等待电流偏置采集"
        : !driveRange ? "已识别角度：请手动移入0°～110°行程" : !ready ? "暂不能启动：检查当前关节行程和电机是否静止"
        : active ? "定点控制中 · 到达后持续保持" : "位置已识别，可控制关节"
    function faultName(code) { return ["无","磁编","ADC采样","PWM时序","采样窗口","电角度校准","Flash保存","通信超时","供电","电流偏置","过流/堵转","超速","关节越界","数值异常","电压"][Math.round(code)] || String(code); }
    function number(text) { return /^[+-]?[0-9]+(\.[0-9]{1,2})?$/.test(text) && isFinite(Number(text)); }
    function send(line) { if(connected) sys_manager.send_string(line+"\r\n"); }
    function stop() {
        active=false;observedRun=false;commandText="";pendingText="";
        stageCommands=[];stageTarget="";actionPending="";send("stop");
        message="输出已关闭。请支撑腿部，避免重力下落。";
    }
    function rejection() {
        var code=Math.round(v(15));
        var reasons=["已接受","先处理故障","请先在零位附近恢复位置","请先标定主动杆方向",
            "等待电流偏置采集完成","这项操作需要先停止输出","目标/轨迹不合法","指令格式或当前状态不允许",
            "电机尚未静止","电机电角度校准未就绪","回零失败：检查是否贴到原可靠止点并保持静止","正在保存标定","当前状态发生变化","标定姿态与回零不符，请先回零再把两主动杆竖直向下",
            "回零失败：磁编采样尚未就绪或已中断，请检查采样状态",
            "回零失败：M0磁编与记录止点不匹配；若确实贴住原止点，需要重新记录基准",
            "回零失败：M1磁编与记录止点不匹配；若确实贴住原止点，需要重新记录基准",
            "回零失败：两路磁编与记录止点不匹配；请核对物理止点和安装",
            "目标规划超时，已保留原目标；请降低速度或缩短本次步幅",
            "M0读数不在近零识别范围，请放在上止点向下0°～40°内",
            "M1读数不在近零识别范围，请放在上止点向下0°～40°内",
            "两路读数不在近零识别范围，请放在上止点向下0°～40°内",
            "硬件初始化未完成，请检查磁编供电与连接，修复后重启"];
        var text=reasons[code] || "指令被拒绝";
        if(code===6) text+="："+["成功","配置","非有限值","足端不可达","超过关节行程","主动/被动杆接近死点","两被动杆接近死点","装配分支","路径预检","速度/加速度"][Math.round(v(19))];
        return text;
    }
    function goalMatches() {
        var p=pendingText.split(" ");
        if(p[0]==="leg" && p[1]==="pos") return Math.abs(v(2)-Number(p[2]))<.03 && Math.abs(v(3)-Number(p[3]))<.03;
        if(p[0]==="foot" && p[1]==="pos") return (flags&32)!==0 && Math.abs(v(6)-Number(p[2]))<.03 && Math.abs(v(7)-Number(p[3]))<.03;
        return false;
    }
    function request(line) {
        if(!firmware || busy) return;
        pendingSequence=v(16);pendingSince=Date.now();pendingText=line;send(line);
        message="正在确认目标…";
    }
    function stageSend() {
        stageSequence=v(16);stageSince=Date.now();send(stageCommands[stageIndex]);
    }
    function move(line,foot) {
        if(busy) {message="正在确认上一项操作，请稍候。";return;}
        if(!foot && motorRateProfiles[rate.currentIndex]>45 && !calibrated) {message="快速关节控制需要先标定主动杆方向；可选慢速调试。";return;}
        if(active || !idle) {request(line);return;}
        stageTarget=line;stageIndex=0;
        var commands=["limit "+(current.value/10).toFixed(2),"leg rate "+motorRateProfiles[rate.currentIndex]];
        if(foot) commands.push("foot speed "+footSpeedProfiles[rate.currentIndex]);
        stageCommands=commands;message="正在确认电流和速度参数…";stageSend();
    }
    function stagePoll() {
        if(stageCommands.length===0) return;
        if(!healthy || !idle) {
            stageCommands=[];stageTarget="";message="启动条件已变化："+status;return;
        }
        if(v(16)!==stageSequence) {
            if(v(15)!==0) {
                message="设置"+(stageIndex===0?"电流上限":"速度")+"失败："+rejection();
                stageCommands=[];stageTarget="";return;
            }
            stageIndex++;
            if(stageIndex>=stageCommands.length) {
                var target=stageTarget;stageCommands=[];stageTarget="";request(target);
            } else stageSend();
        } else if(Date.now()-stageSince>800) {
            stageCommands=[];stageTarget="";message="参数设置未收到确认，尚未启动电机。";
        }
    }
    function applyJoint() {
        if(!ready || !number(joint.text) || Number(joint.text)<0 || Number(joint.text)>110) {
            message=!ready ? status : "关节目标必须为0～110°，最多两位小数";return;
        }
        var q=Number(joint.text).toFixed(2);move("leg pos "+q+" "+q,false);
    }
    function applyFoot() {
        if(!ready || !calibrated) {message=!calibrated ? "请先完成主动杆方向标定；关节控制仍可使用。" : status;return;}
        if(!number(footX.text) || !number(footY.text)) {message="足端坐标请输入毫米，最多两位小数";return;}
        move("foot pos "+Number(footX.text).toFixed(2)+" "+Number(footY.text).toFixed(2),true);
    }
    function useCurrent() {
        if(!firmware) return;
        if(mode.currentIndex===0) joint.text=((v(0)+v(1))*.5).toFixed(2);
        else if((flags&128)!==0) {footX.text=v(4).toFixed(2);footY.text=v(5).toFixed(2);}
        message="已填入当前位置，点击移动/更新后开始保持。";
    }
    function holdCurrent() {
        if(!ready) return;
        if(mode.currentIndex===0) move("leg pos "+Math.max(0,Math.min(110,v(0))).toFixed(2)+" "+Math.max(0,Math.min(110,v(1))).toFixed(2),false);
        else if(calibrated && (flags&128)!==0) move("foot pos "+v(4).toFixed(2)+" "+v(5).toFixed(2),true);
    }
    function preset(q) {joint.text=q.toFixed(2);if(active && mode.currentIndex===0) applyJoint();}
    function bump(dx,dy) {
        if(!number(footX.text) || !number(footY.text)) return;
        footX.text=(Number(footX.text)+dx).toFixed(2);footY.text=(Number(footY.text)+dy).toFixed(2);
        if(active && mode.currentIndex===1) applyFoot();
        plot.requestPaint();
    }
    function refresh() {
        if(!connected) {message="串口未连接，请先连接COM8。";return;}
        refreshSequence=receiveCount;refreshSince=Date.now();refreshPending=true;
        message="正在读取新版控制状态。";send("send 5");
    }
    onConnectedChanged: {
        lastDataMs=0;lastProgressMs=0;lastSequence=-1;
        refreshPending=false;actionPending="";stageCommands=[];stageTarget="";
        if(!connected) message="串口已断开，请连接COM8。";
    }
    function setHome() {
        if(!firmware || !idle || active || busy) return;
        if(homed) {message="位置跟踪仍有效，无需重复识别；当前角度已保留。";return;}
        if(!supportsRestore) {message="请更新固件后再使用近零位置恢复。";return;}
        actionSince=Date.now();actionSequence=v(16);actionPending="home";send("leg restore");message="正在识别圈数，保留当前位置的实际角度。";
    }
    function calibrateSave() {
        if(!idle || !homed || active || busy) return;
        actionSince=Date.now();actionSequence=v(16);actionPending="save";send("foot calibrate");
        message="正在标定并保存。成功后将两路放在上止点向下0°～40°内，再点击恢复位置。";
    }
    function solveForward(q0,q1) {
        var t0=(v(21)-q0)*Math.PI/180,t1=(v(22)+q1)*Math.PI/180;
        var b0=[35+80*Math.cos(t0),-80*Math.sin(t0)],b1=[-35+80*Math.cos(t1),-80*Math.sin(t1)];
        var dx=b1[0]-b0[0],dy=b1[1]-b0[1],d=Math.sqrt(dx*dx+dy*dy);
        if(d<=0 || d>=290) return null;
        var h=Math.sqrt(145*145-d*d/4);
        return [(b0[0]+b1[0])/2+h*dy/d,(b0[1]+b1[1])/2-h*dx/d,b0,b1];
    }
    function updateWorkspace() {
        if(v(21)===workspaceZero0 && v(22)===workspaceZero1) return;
        workspaceZero0=v(21);workspaceZero1=v(22);var points=[];
        for(var a=2;a<=108;a+=4) for(var b=2;b<=108;b+=4) {
            var p=solveForward(a,b);if(p) points.push(p);
        }
        workspacePoints=points;
    }
    Connections {
        target: sys_manager
        onNeed_update: {
            root.lastDataMs=Date.now();root.receiveCount++;
            if(root.v(23)===5005 && root.v(20)!==root.lastSequence) {
                root.lastSequence=root.v(20);root.lastProgressMs=Date.now();
            }
        }
    }
    Timer {
        interval: 50;repeat:true;running:true
        onTriggered: {
            root.nowMs=Date.now();
            if(root.refreshPending) {
                if(root.receiveCount!==root.refreshSequence && root.fresh) {
                    root.message=root.firmware ? "连接正常，状态已更新："+root.status : "已收到数据，但固件版本或通道配置不匹配。";
                    root.refreshPending=false;
                } else if(root.nowMs-root.refreshSince>1500) {
                    root.message="未收到板端状态：检查COM8、JustFloat和DTR，或断开后重新连接。";
                    root.refreshPending=false;
                }
            }
            if(root.connected && !root.firmware && !root.active && root.pendingText==="" && root.actionPending==="" && root.stageCommands.length===0 && root.nowMs-root.lastQueryMs>1000) {
                root.send("send 5");root.lastQueryMs=root.nowMs;
            }
            if(root.firmware && root.actionPending!=="" && root.v(16)!==root.actionSequence) {
                if(root.v(15)!==0) {root.message=root.rejection();root.actionPending="";}
                else if(root.actionPending==="home" && root.homed) {
                    root.message=root.driveRange ? "位置恢复成功：M0 "+root.v(0).toFixed(2)+"°，M1 "+root.v(1).toFixed(2)+"°。可设置目标。" : "位置恢复成功，当前在软件行程外；请手动稍向行程内移动，再设置目标。";root.actionPending="";
                } else if(root.actionPending==="save" && root.saved && !root.saving) {
                    root.message="标定已保存。请放到上止点向下0°～40°内，再点击恢复位置。";root.actionPending="";
                }
            }
            root.stagePoll();
            if(root.actionPending!=="" && root.nowMs-root.actionSince>(root.actionPending==="save"?10000:1500)) {
                root.message=root.actionPending==="home" ? "未收到位置恢复确认，请刷新检查实际角度与位置状态。" : "未收到标定保存确认。请刷新检查“几何已保存”状态，不要重复标定。";
                root.actionPending="";
            }
            if(root.active || root.pendingText!=="") {
                if(!root.healthy || root.nowMs-root.lastProgressMs>250) {
                    root.stop();root.message="故障或角度采样中断，已停止。";return;
                }
                if(root.pendingText!=="") {
                    if(root.v(16)!==root.pendingSequence) {
                        if(root.v(15)===0 && root.goalMatches()) {
                            if(!root.active) root.observedRun=false;
                            root.commandText=root.pendingText;root.active=true;root.pendingText="";
                            root.message="目标已接受，自动保持；修改目标后点击更新。";
                        } else if(root.v(15)!==0) {
                            root.message=root.rejection()+ (root.active ? "；继续保持原目标。" : "");
                            root.pendingText="";
                        }
                        if(root.pendingText!=="" && root.nowMs-root.pendingSince>300) {
                            root.pendingText="";root.message="位置目标没有生效，请检查板上状态。";
                        }
                    } else if(root.nowMs-root.pendingSince>300) {
                        root.pendingText="";root.message="没有收到目标确认，请检查固件/串口。";
                    }
                }
                if(root.active && root.v(11)===4 && root.v(12)===4) root.observedRun=true;
                if(root.active && root.observedRun && (root.v(11)===0 || root.v(12)===0)) {
                    root.stop();root.message="板上输出已停止，面板不会自动重新开启。";return;
                }
                if(root.active && root.pendingText==="") root.send(root.commandText);
            }
            if(root.firmware) {root.updateWorkspace();plot.requestPaint();}
        }
    }
    ColumnLayout {
        anchors.fill:parent;anchors.margins:16;spacing:12
        RowLayout {
            Text {text:"五连杆腿控制";Layout.preferredWidth:225;font.pixelSize:28;font.bold:true;color:"#155786"}
            Text {text:root.status;font.pixelSize:20;color:root.healthy?"#16835d":"#b54936";Layout.fillWidth:true;elide:Text.ElideRight}
            Button {text:"刷新连接";font.pixelSize:19;onClicked:root.refresh()}
            Button {text:"停止输出";font.pixelSize:23;palette.buttonText:"#c93939";onClicked:root.stop()}
        }
        RowLayout {
            Button {objectName:"homeButton";text:"① 恢复腿部位置";Layout.preferredWidth:245;font.pixelSize:20;enabled:root.idle && !root.active && !root.busy;onClicked:root.homed?root.setHome():homeDialog.open()}
            Button {objectName:"calButton";text:"首次：标定主动杆方向";Layout.preferredWidth:250;font.pixelSize:19;enabled:root.idle && root.homed && !root.active && !root.busy;onClicked:calDialog.open()}
            Button {text:"清除故障";font.pixelSize:19;enabled:root.stopped && !root.startupFailed && !root.active && !root.busy;onClicked:root.send("clear")}
            Item {Layout.fillWidth:true}
            Text {text:root.saved?"几何已标定并保存":root.calibrated?"已标定，尚未保存":"几何未标定：可用关节模式";font.pixelSize:18;color:root.saved?"#16835d":"#b57520"}
        }
        RowLayout {
            Layout.fillWidth:true;Layout.fillHeight:true;spacing:14
            Rectangle {
                Layout.preferredWidth:410;Layout.fillHeight:true;color:"white";radius:10;border.color:"#dce4ed"
                ColumnLayout {
                    anchors.fill:parent;anchors.margins:16;spacing:8
                    ComboBox {id:mode;objectName:"controlMode";Layout.minimumWidth:0;Layout.fillWidth:true;model:["关节角度 · 两路同步","足端位置 · x/y毫米"];font.pixelSize:21;enabled:!root.active && !root.busy}
                    Text {text:"M0："+ (root.homed?root.v(0).toFixed(2):"—")+"°    M1："+(root.homed?root.v(1).toFixed(2):"—")+"°";font.pixelSize:23;color:"#155786"}
                    ColumnLayout {
                        visible:mode.currentIndex===0;Layout.fillWidth:true;spacing:10
                        Text {text:"目标关节角度（°）";font.pixelSize:20;color:"#53657a"}
                        TextField {id:joint;objectName:"jointTarget";text:"10";font.pixelSize:26;Layout.fillWidth:true;selectByMouse:true}
                        Slider {id:jointSlider;from:2;to:108;stepSize:1;value:root.number(joint.text)?Number(joint.text):10;Layout.fillWidth:true;onMoved:joint.text=value.toFixed(2)}
                        RowLayout {Repeater {model:[2,30,60,90,108];Button {text:modelData+"°";font.pixelSize:15;Layout.fillWidth:true;Layout.minimumWidth:0;onClicked:root.preset(modelData)}}}
                        Text {text:"定点控制：输入目标 → 移动 → 到达后持续保持。\n改变目标点更新；停止输出才会释放。";font.pixelSize:15;color:"#53657a";wrapMode:Text.WordWrap;Layout.fillWidth:true}
                    }
                    ColumnLayout {
                        visible:mode.currentIndex===1;Layout.fillWidth:true;spacing:10
                        Text {text:root.calibrated?"目标轮轴位置（mm）":"先点击上方“标定主动杆方向”";font.pixelSize:19;color:root.calibrated?"#53657a":"#b57520";wrapMode:Text.WordWrap;Layout.fillWidth:true}
                        RowLayout {
                            Text {text:"x";font.pixelSize:23}
                            TextField {id:footX;objectName:"footX";implicitWidth:110;Layout.minimumWidth:0;text:"0";font.pixelSize:26;Layout.fillWidth:true;selectByMouse:true;onTextChanged:plot.requestPaint()}
                            Text {text:"y";font.pixelSize:23}
                            TextField {id:footY;objectName:"footY";implicitWidth:110;Layout.minimumWidth:0;text:"160";font.pixelSize:26;Layout.fillWidth:true;selectByMouse:true;onTextChanged:plot.requestPaint()}
                        }
                        RowLayout {
                            Button {text:"← 2mm";Layout.fillWidth:true;Layout.minimumWidth:0;font.pixelSize:14;onClicked:root.bump(-2,0)}
                            Button {text:"→ 2mm";Layout.fillWidth:true;Layout.minimumWidth:0;font.pixelSize:14;onClicked:root.bump(2,0)}
                            Button {text:"↑ 2mm";Layout.fillWidth:true;Layout.minimumWidth:0;font.pixelSize:14;onClicked:root.bump(0,-2)}
                            Button {text:"↓ 2mm";Layout.fillWidth:true;Layout.minimumWidth:0;font.pixelSize:14;onClicked:root.bump(0,2)}
                        }
                        Text {text:"原点：两固定轴中点\nx向右、y向下；绿色点仅表示几何可达域。";font.pixelSize:15;color:"#53657a";Layout.fillWidth:true;wrapMode:Text.WordWrap}
                    }
                    RowLayout {
                        Button {text:"填入当前位置";font.pixelSize:15;enabled:root.firmware;onClicked:root.useCurrent()}
                        Button {text:"保持当前位置";font.pixelSize:15;enabled:root.ready && !root.busy && (mode.currentIndex===0 || root.calibrated);onClicked:root.holdCurrent()}
                    }
                    RowLayout {
                        Text {text:"速度";font.pixelSize:19}
                        ComboBox {id:rate;objectName:"motionRate";model:["慢速 10 RPM","正常 60 RPM","快速 120 RPM"];currentIndex:2;enabled:!root.active && !root.busy;Layout.fillWidth:true;font.pixelSize:20}
                    }
                    RowLayout {
                        Text {text:"启动电流上限";font.pixelSize:18}
                        SpinBox {id:current;objectName:"currentLimit";from:2;to:100;value:100;stepSize:5;enabled:!root.active && root.idle && !root.busy;Layout.fillWidth:true;textFromValue:function(value,locale){return (value/10).toFixed(1)+" A"}}
                    }
                    Button {objectName:"runButton";text:root.active?"② 更新目标":"② 移动到目标";font.pixelSize:24;Layout.fillWidth:true;
                        enabled:root.ready && !root.busy && (mode.currentIndex===0 || root.calibrated);
                        onClicked:mode.currentIndex===0?root.applyJoint():root.applyFoot()}
                    Text {objectName:"startHelp";text:root.busy?"正在确认上一项操作…":!root.ready?root.status:mode.currentIndex===1 && !root.calibrated?"足端控制需先标定主动杆方向":"位置由面板每50ms自动刷新。\n不要同时启动其他串口发送循环。";font.pixelSize:15;color:"#53657a";Layout.fillWidth:true;wrapMode:Text.WordWrap}
                    Item {Layout.fillHeight:true}
                }
            }
            Rectangle {
                Layout.fillWidth:true;Layout.fillHeight:true;color:"white";radius:10;border.color:"#dce4ed"
                ColumnLayout {
                    anchors.fill:parent;anchors.margins:14
                    RowLayout {
                        Text {text:root.calibrated?"腿部姿态 / 足端估算":"名义几何预览（零偏未标定）";font.pixelSize:22;color:"#155786";Layout.fillWidth:true}
                        Text {text:root.modelValid?"足端 "+root.v(4).toFixed(1)+", "+root.v(5).toFixed(1)+" mm":root.poseKnown?"角度已识别 · 姿态示意":"尚无有效关节位置";font.pixelSize:19;color:"#53657a"}
                    }
                    Canvas {
                        id:plot;Layout.fillWidth:true;Layout.fillHeight:true
                        function scale(){return Math.min(width/400,height/390);}
                        function sx(x){return width*.5+x*scale();}
                        function sy(y){return height*.26+y*scale();}
                        onPaint: {
                            var c=getContext("2d");c.clearRect(0,0,width,height);
                            c.strokeStyle="#e6edf3";c.lineWidth=1;c.font="14px sans-serif";c.fillStyle="#7b8794";
                            for(var x=-150;x<=150;x+=50){c.beginPath();c.moveTo(sx(x),0);c.lineTo(sx(x),height);c.stroke();c.fillText(x,sx(x)+3,sy(0)-8);}
                            for(var y=-50;y<=250;y+=50){c.beginPath();c.moveTo(0,sy(y));c.lineTo(width,sy(y));c.stroke();c.fillText(y,3,sy(y)-3);}
                            c.fillStyle="#d9ece2";for(var i=0;i<root.workspacePoints.length;i++){var p=root.workspacePoints[i];c.fillRect(sx(p[0])-2,sy(p[1])-2,4,4);}
                            if(root.poseKnown) {
                                var p=root.displayPose;
                                for(var i=0;i<2;i++) {
                                    var theta=(root.v(21+i)+(i===0?-root.v(0):root.v(1)))*Math.PI/180;
                                    var base=i===0?35:-35,ex=base+80*Math.cos(theta),ey=-80*Math.sin(theta);
                                    c.strokeStyle=i===0?"#247fc0":"#ee9436";c.lineWidth=5;c.beginPath();
                                    c.moveTo(sx(base),sy(0));c.lineTo(sx(ex),sy(ey));c.stroke();
                                    if(p) {
                                        c.save();if(!root.modelValid)c.setLineDash([8,6]);
                                        c.beginPath();c.moveTo(sx(ex),sy(ey));c.lineTo(sx(p[0]),sy(p[1]));c.stroke();c.restore();
                                    }
                                }
                                if(p) {c.fillStyle=root.modelValid?"#155786":"#b57520";c.beginPath();c.arc(sx(p[0]),sy(p[1]),8,0,Math.PI*2);c.fill();}
                            } else {
                                c.fillStyle="#b57520";c.font="19px sans-serif";
                                c.fillText(root.startupFailed?"磁编初始化失败，暂时无法显示姿态":"尚未恢复腿部位置，请先点击①",20,height-35);
                            }
                            c.font="18px sans-serif";c.fillStyle="#155786";c.fillText("M1",sx(-35)-20,sy(0)-20);c.fillText("M0",sx(35)-10,sy(0)-20);
                            if(mode.currentIndex===1 && root.number(footX.text) && root.number(footY.text)) {
                                var x=Number(footX.text),y=Number(footY.text);
                                c.strokeStyle="#c75348";c.lineWidth=2;c.beginPath();c.arc(sx(x),sy(y),10,0,Math.PI*2);c.stroke();
                                c.fillStyle="#c75348";c.fillText("目标",sx(x)+14,sy(y)-4);
                            }
                        }
                        MouseArea {anchors.fill:parent;enabled:mode.currentIndex===1 && root.calibrated;
                            onClicked:{footX.text=((mouse.x-parent.width*.5)/parent.scale()).toFixed(2);footY.text=((mouse.y-parent.height*.26)/parent.scale()).toFixed(2);root.message="已点选足端目标，点击移动/更新后执行。";}}
                    }
                    Text {text:"蓝色 M0 · 橙色 M1 · 红圈目标。足端模式可点击图中位置选目标。";font.pixelSize:15;color:"#53657a";Layout.fillWidth:true;wrapMode:Text.WordWrap}
                    RowLayout {
                        Text {text:"Iq M0 "+(root.firmware && isFinite(root.v(8))?root.v(8).toFixed(2):"—")+" A";font.pixelSize:18;color:"#155786"}
                        Text {text:"Iq M1 "+(root.firmware && isFinite(root.v(9))?root.v(9).toFixed(2):"—")+" A";font.pixelSize:18;color:"#155786"}
                        Item {Layout.fillWidth:true}
                        Text {text:"电压 "+(root.firmware && isFinite(root.v(10))?root.v(10).toFixed(2):"—")+" V";font.pixelSize:18;color:"#155786"}
                    }
                }
            }
        }
        Text {text:root.message;font.pixelSize:19;color:"#53657a";wrapMode:Text.WordWrap;Layout.fillWidth:true;Layout.preferredHeight:48}
    }
    Dialog {
        id:homeDialog;title:"确认零位附近区域";modal:true;anchors.centerIn:parent;width:590
        footer:DialogButtonBox {
            Button {text:"确定，识别当前位置";DialogButtonBox.buttonRole:DialogButtonBox.AcceptRole}
            Button {text:"取消";DialogButtonBox.buttonRole:DialogButtonBox.RejectRole}
            onAccepted:homeDialog.accept();onRejected:homeDialog.reject()
        }
        onAccepted:root.setHome()
        contentItem:Text {text:"请确认两路主动杆都在原上止点向下约0°～40°的区域内，并支撑静止。不必贴紧止点。\n\n程序按保存的零位恢复圈数，保留当前角度；不会启动电机。\n\n若目前在其他区域，先手动放回上述范围。同一磁编读数每51.43°关节角会重复，不能在任意姿态识别。";font.pixelSize:20;wrapMode:Text.WordWrap;color:"#53657a"}
    }
    Dialog {
        id:calDialog;title:"首次标定主动杆方向";modal:true;anchors.centerIn:parent;width:660
        onOpened:calConfirmed.checked=false
        footer:DialogButtonBox {
            Button {text:"标定并保存";enabled:calConfirmed.checked;DialogButtonBox.buttonRole:DialogButtonBox.AcceptRole}
            Button {text:"取消";DialogButtonBox.buttonRole:DialogButtonBox.RejectRole}
            onAccepted:calDialog.accept();onRejected:calDialog.reject()
        }
        onAccepted:if(calConfirmed.checked) root.calibrateSave()
        contentItem:ColumnLayout {
            spacing:16
            Text {text:"保持上电、输出关闭，把两根主动杆摆成竖直向下并固定；两个固定轴的连线应水平。\n\n确认后自动保存。完成后移到上止点向下0°～40°内，再点击恢复位置。以后无需重复标定。";font.pixelSize:20;wrapMode:Text.WordWrap;color:"#53657a";Layout.preferredWidth:610}
            CheckBox {id:calConfirmed;text:"两主动杆已竖直向下，机构已固定";font.pixelSize:19}
        }
    }
    Component.onDestruction: root.stop()
    function get_widget_ctx() {return {"path":path,"ctx":{".":{"ctx":{".":{x:x,y:y,width:width,height:height}}}}};}
    function set_widget_ctx(value) {active=false;commandText="";pendingText="";stageCommands=[];stageTarget="";actionPending="";__set_ctx__(root,value.ctx);}
}

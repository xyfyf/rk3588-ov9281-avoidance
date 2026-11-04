"""Create a portable, offline playback of recorded real ArduCopter SITL tests."""
from pathlib import Path
import argparse
import json
import statistics


def build(root,output):
    runs=[]
    for prefix,label in [('avoid_off','关闭避障 · 理想距离'),('avoid_on','开启避障 · 理想距离'),
                         ('avoid_dropout','开启避障 · 第7秒起全未知'),('visual_on','开启避障 · 四目图像算法')]:
        candidates=sorted(root.glob(prefix+'_*/result.json'),key=lambda p:p.stat().st_mtime,reverse=True)
        selected=None
        for path in candidates:
            run=json.loads(path.read_text())
            if run.get('completed') and run.get('positions') and run.get('pilot_pitch_pwm')==1100:
                selected=(path,run)
                break
        if selected is None:continue
        path,run=selected
        folder=path.parent.name
        run['label']=label
        run['folder']=folder
        frames=run.get('visual_frames',[])
        if frames:
            intervals=[b['capture_wall_s']-a['capture_wall_s'] for a,b in zip(frames,frames[1:])]
            run['fresh_frame_mean_hz']=1/statistics.mean(intervals) if intervals else None
            run['processing_median_ms']=statistics.median(f['processing_ms'] for f in frames)
            run['render_median_ms']=statistics.median(f['render_ms'] for f in frames)
        runs.append(run)
    reception_path=root/'reception_02/result.json'
    if not reception_path.exists():reception_path=root/'reception_01/result.json'
    reception=json.loads(reception_path.read_text())
    payload={'runs':runs,'reception':reception,'date':'2026-11-03'}
    (root/'summary.json').write_text(json.dumps(payload,ensure_ascii=False),encoding='utf-8')
    data=json.dumps(payload,ensure_ascii=False).replace('</','<\\/')
    html='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>四目避障 · ArduPilot SITL 实测回放</title><style>
*{box-sizing:border-box}body{font:16px/1.65 system-ui,"Microsoft Yahei",sans-serif;background:#f6f7f8;color:#202a31;margin:0}main{max-width:1160px;margin:auto;padding:30px 24px}h1{font-size:27px;line-height:1.3;margin:6px 0}h2{font-size:19px;margin:26px 0 10px}.muted,small{color:#596771}p{margin:9px 0}a{color:#086b68}.intro{max-width:950px}.notice{padding:12px 16px;background:#fff0dc;border-left:4px solid #a86109;margin:18px 0}.layout{display:grid;grid-template-columns:1.15fr 1fr;gap:22px}.panel{background:white;padding:16px;border:1px solid #dce1e5}.controls{display:flex;gap:12px;align-items:center;flex-wrap:wrap;margin:16px 0}select,button{font:inherit;padding:7px 11px;background:white;border:1px solid #b9c5cc;border-radius:4px}input{width:100%}canvas{display:block;width:100%;height:auto}table{width:100%;border-collapse:collapse;font-size:14px}td,th{border-bottom:1px solid #dce1e5;padding:9px 6px;text-align:left}th{font-weight:600}.table-wrap{overflow:auto}#cameras{width:100%;display:block}#cameras[hidden]{display:none}#metrics{font-variant-numeric:tabular-nums}.key{color:#086b68;font-weight:600}code{font-size:13px;overflow-wrap:anywhere}details{margin:15px 0}li{margin:5px 0}.caption{font-size:13px;color:#596771;margin-top:6px}#frame-label{font-variant-numeric:tabular-nums}footer{border-top:1px solid #dce1e5;margin-top:28px;padding-top:14px;font-size:13px}@media(max-width:780px){main{padding:20px 14px}.layout{grid-template-columns:1fr}h1{font-size:24px}}
</style><main><p class="muted">本机实际运行记录 · 2026-11-03 · Ubuntu 22.04 / WSL / x86_64</p>
<h1>四目避障接入 ArduPilot：真实 SITL 回放</h1>
<p class="intro">启动用户提供的 <code>arducopter</code> ELF，由 ArduCopter 的姿态、位置和避障控制器执行起飞、Loiter 向前飞行和降落。脚本持续保持向前遥控输入，比较开启、关闭避障及数据失效后的轨迹。这里的“真实”指实际飞控程序运行，环境和图像均为合成。</p>
<div class="notice">障碍物通过距离消息注入，没有实体碰撞网格。“越过障碍平面”是根据轨迹判定，不能表述成物理碰撞。当前仅测试该定制 ArduPilot 固件；PX4、RK3588、实物相机与实飞均未验证。</div>
<h2>飞行对照结果</h2><p>同为 Loiter，俯仰输入 1100 PWM 持续16秒，LOIT_SPEED 配置为1 m/s（非硬限速），避障余量1 m。表内速度均来自飞控遥测。</p><div class="table-wrap"><table><thead><tr><th>场景</th><th>最小平面间距</th><th>末2秒平均前进速度</th><th>最大前进速度</th><th>是否越过</th></tr></thead><tbody id="results"></tbody></table></div>
<div class="controls"><label for="scenario">回放场景</label><select id="scenario"></select><button id="play">播放</button><span id="frame-label"></span></div>
<input id="timeline" type="range" min="0" value="0" step="1" aria-label="飞控遥测时间轴">
<div class="layout"><section><h2>飞行轨迹 · 局部平面</h2><div class="panel"><canvas id="map" width="680" height="390"></canvas><p class="caption">横轴：相对起点的北向位置（m）；纵轴：东向位置（m）。蓝绿线为已飞轨迹，圆点为当前位置。图像场景中的方盒仅横跨东西方向 ±0.35 m，理想距离场景使用无限墙面。</p><p id="metrics"></p></div>
<h2>速度与距离随时间变化</h2><div class="panel"><canvas id="chart" width="680" height="320"></canvas><p class="caption">来源：各次运行的 LOCAL_POSITION_NED 或明确标记的 GLOBAL_POSITION_INT 派生值。横轴为向前输入开始后的本机经过时间；实测为 speedup=1。蓝绿：北向速度（m/s）；棕色：至障碍平面的有符号间距（m）。</p></div></section>
<section><h2>当前传感器与控制输入</h2><div class="panel"><p id="sensor"></p><img id="cameras" alt="该时刻附近的四目合成输入图像"><p id="optical" class="caption"></p><p>控制输入在前16秒保持不变，随后回中。脚本没有根据障碍距离发送停车指令；减速来自飞控避障控制器。</p></div>
<h2>地面接收与异常值</h2><div class="panel" id="reception"></div>
<h2>这次覆盖了哪些环节</h2><ul><li>四个方向的距离数据编码、接收和飞控回显。</li><li>实际 ArduCopter 中的起飞、Loiter、距离避障与降落。</li><li>图像场景使用四鱼眼光线投射、当前双目匹配代码和正式 MAVLink 编码器；位姿由飞控实时反馈。</li><li>全未知值的控制行为单独测试，不能假定它会要求停车。</li></ul></section></div>
<h2>测试边界与已知问题</h2><p>软件图像渲染较慢；20 Hz 是距离消息的发布目标，新图像频率单独列出。图像仿真允许结果年龄650 ms，仅为该试验的参数，正式运行的200 ms门限没有放宽。相机模型只跟随水平平移和偏航，忽略俯仰、横滚及高度变化，不含曝光、运动模糊、真实标定误差或串口时延。</p>
<p>之前的离线图像试验已发现远处误报近障和近场未知；本次控制器能停车，也不能消除视觉漏检风险。仅水平72格距离，不覆盖上下方。“最小间距”按机体参考点计算，未减旋翼或机体半径。</p>
<details><summary>复现配置、二进制身份与无效运行记录</summary><p id="identity"></p><pre id="parameters" style="white-space:pre-wrap;font-size:13px"></pre><p>保留了 avoid_off_01（未取得局部位置）、avoid_off_02（EKF尚未就绪）、avoid_off_03（遥控覆盖ID不符，落地后仍记录，判定无效）、avoid_off_04（输入过小，未接近墙面）。正式对照仅使用表中所列运行。底层 ArduPilot 仓库含用户修改，本次未改动。</p></details>
<footer>本报告内嵌实测数据，可离线拖动和播放。回放不代表现场计算速度。完整参数、遥测、障碍输入与飞控日志保存在算法包 examples/ardupilot_sitl_20261103/。</footer></main>
<script>const D=__DATA__;let index=0,timer=null;const $=id=>document.getElementById(id);const fmt=(x,n=2)=>Number(x).toFixed(n);const run=()=>D.runs[index];
D.runs.forEach((r,i)=>{const o=document.createElement('option');o.value=i;o.textContent=r.label;$('scenario').append(o);const tr=document.createElement('tr');[r.label,fmt(r.minimum_gap_m)+' m',fmt(r.last_2s_mean_north_speed_mps)+' m/s',fmt(r.maximum_north_speed_mps)+' m/s',r.crossed_virtual_wall?'是':'否'].forEach(v=>{const td=document.createElement('td');td.textContent=v;tr.append(td)});$('results').append(tr)});
for(const c of D.reception.cases){const p=document.createElement('p');const names={cardinal_metres:'前/右/后/左：1/2/3/4 m',zero_contact:'原始0 cm：固件忽略',production_zero_contact:'修复后0→1 cm：验证回显',one_centimetre:'原始1 cm：接受',all_unknown:'全未知：无距离回显',no_messages:'停止发布：无距离回显'};const echoes=c.distance_sensor_messages.map(m=>m.orientation+':'+m.current_distance);p.textContent=(names[c.label]||c.label)+'；回显 '+(echoes.length?echoes.slice(-8).join(', '):'无');$('reception').append(p)}
function chart(c,series){const x=c.getContext('2d');x.clearRect(0,0,c.width,c.height);const L=58,R=18,T=22,B=44;const W=c.width-L-R,H=c.height-T-B;const values=series.flatMap(s=>s.rows.map(r=>r[1]));let lo=Math.min(0,...values),hi=Math.max(1,...values);hi+=.1*(hi-lo);lo-=.05*(hi-lo);const end=Math.max(...series.flatMap(s=>s.rows.map(r=>r[0])));const X=t=>L+t/end*W,Y=v=>T+(hi-v)/(hi-lo)*H;x.font='13px system-ui';x.textAlign='right';for(let i=0;i<=4;i++){let v=lo+(hi-lo)*i/4;x.strokeStyle='#e0e5e8';x.beginPath();x.moveTo(L,Y(v));x.lineTo(c.width-R,Y(v));x.stroke();x.fillStyle='#596771';x.fillText(fmt(v,1),L-7,Y(v)+4)}x.textAlign='center';for(let t=0;t<=end;t+=4){x.fillText(t+'s',X(t),c.height-22)}x.fillText('向前输入开始后的时间 (s)',L+W/2,c.height-3);x.textAlign='left';x.fillText('速度(m/s) / 间距(m)',L,14);for(const s of series){x.strokeStyle=s.color;x.lineWidth=2;x.beginPath();s.rows.forEach((r,i)=>i?x.lineTo(X(r[0]),Y(r[1])):x.moveTo(X(r[0]),Y(r[1])));x.stroke()} }
function draw(){const r=run(),i=+$('timeline').value,p=r.positions[i],t=p.wall_s-r.forward_start_wall_s,gap=r.wall_x_m-p.x;$('frame-label').textContent=fmt(t)+' s · '+(i+1)+'/'+r.positions.length;$('metrics').textContent='平面间距 '+fmt(gap)+' m ｜ 北向速度 '+fmt(p.vx)+' m/s ｜ 高度 '+fmt(-p.z)+' m';
const c=$('map'),x=c.getContext('2d');x.clearRect(0,0,c.width,c.height);const wall=r.wall_x_m-r.start_x_m,max=Math.max(wall+2,...r.positions.map(v=>v.x-r.start_x_m))+1,X=n=>48+(n+1)/(max+1)*(c.width-75),Y=e=>c.height/2+e*70;x.font='13px system-ui';x.fillStyle='#596771';x.textAlign='center';for(let n=0;n<=max;n+=2){x.strokeStyle='#e0e5e8';x.beginPath();x.moveTo(X(n),30);x.lineTo(X(n),c.height-30);x.stroke();x.fillText(n+' m',X(n),c.height-9)}for(let e=-2;e<=2;e++){x.textAlign='left';x.fillText(e+' m E',0,Y(e));}x.strokeStyle='#a86109';x.lineWidth=4;x.beginPath();x.moveTo(X(wall),r.visual_frames?Y(-.35):28);x.lineTo(X(wall),r.visual_frames?Y(.35):c.height-30);x.stroke();x.fillStyle='#a86109';x.fillText(r.visual_frames?'前方盒面':'虚拟墙面',X(wall)+8,25);x.strokeStyle='#086b68';x.lineWidth=3;x.beginPath();r.positions.slice(0,i+1).forEach((q,k)=>k?x.lineTo(X(q.x-r.start_x_m),Y(q.y-r.positions[0].y)):x.moveTo(X(q.x-r.start_x_m),Y(q.y-r.positions[0].y)));x.stroke();x.fillStyle='#086b68';x.beginPath();x.arc(X(p.x-r.start_x_m),Y(p.y-r.positions[0].y),7,0,7);x.fill();
chart($('chart'),[{color:'#086b68',rows:r.positions.map(q=>[q.wall_s-r.forward_start_wall_s,q.vx])},{color:'#a86109',rows:r.positions.map(q=>[q.wall_s-r.forward_start_wall_s,r.wall_x_m-q.x])}]);
if(r.visual_frames){const frames=r.visual_frames.filter(f=>f.ready_wall_s<=p.wall_s),f=frames[frames.length-1];$('cameras').hidden=!f;if(f){$('cameras').src=f.thumbnail;$('sensor').textContent='新图像平均 '+fmt(r.fresh_frame_mean_hz)+' Hz；距离消息 '+fmt(r.metadata.publication_mean_hz,1)+' Hz。当前图像含 '+f.valid_sectors+'/72 个有效扇区。';$('optical').textContent='图像年龄 '+fmt((p.wall_s-f.capture_wall_s)*1000,0)+' ms；本帧渲染 '+fmt(f.render_ms,0)+' ms，算法 '+fmt(f.processing_ms,1)+' ms。中位渲染 '+fmt(r.render_median_ms,0)+' ms，算法 '+fmt(r.processing_median_ms,1)+' ms。'}}else{$('cameras').hidden=true;$('sensor').textContent=r.dropout_after_s&&t>=r.dropout_after_s?'故障阶段：持续发送72格全未知。':'理想距离注入：由飞控位置和偏航计算墙面射线距离。';$('optical').textContent='此场景没有运行图像算法，用于隔离检查飞控的距离接收与控制响应。'}
$('identity').textContent=r.metadata.firmware_statuses[0]+'；SHA256 '+r.metadata.binary_sha256+'；位置来源 '+r.metadata.position_source;$('parameters').textContent=JSON.stringify(r.metadata.parameter_readback,null,2);}
function select(){clearInterval(timer);timer=null;$('play').textContent='播放';index=+$('scenario').value;$('timeline').max=run().positions.length-1;$('timeline').value=0;draw()};$('scenario').onchange=select;$('timeline').oninput=draw;$('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;$('play').textContent='播放';return}if(+$('timeline').value>=+$('timeline').max)$('timeline').value=0;$('play').textContent='暂停';timer=setInterval(()=>{if(+$('timeline').value>=+$('timeline').max){clearInterval(timer);timer=null;$('play').textContent='播放';return}$('timeline').value=+$('timeline').value+1;draw()},50)};if(D.runs.length)select();
</script></html>'''.replace('__DATA__',data)
    output.write_text(html,encoding='utf-8')
    print(json.dumps({'report':str(output),'runs':len(runs)},ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--root',required=True)
    p.add_argument('--output',required=True)
    a=p.parse_args()
    build(Path(a.root),Path(a.output))

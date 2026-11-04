"""Small offline HTML view of a real algorithm output; no external dependencies."""
from __future__ import annotations

import html
import json
from pathlib import Path


def write_report(path, result, title=None):
    title = title or ("四目鱼眼合成场景验证" if result.get("synthetic") else "四目鱼眼离线回放结果")
    template = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>四目鱼眼算法离线结果</title><style>
body{font:16px/1.6 system-ui,sans-serif;background:#f5f5f2;color:#202623;margin:0}main{max-width:1100px;margin:auto;padding:30px}h1{font-size:26px}h2{font-size:19px}.columns{display:grid;grid-template-columns:1.1fr 1fr;gap:28px}section{margin:25px 0}svg{width:100%;background:white}label{display:block;margin:12px 0}input{width:110px}code{background:#e6e8e4;padding:3px 5px}small{color:#59615b}table{border-collapse:collapse;width:100%}td,th{text-align:left;padding:5px;border-bottom:1px solid #d8ddd7}.status{padding:14px;background:#e6ede7}a{color:#176245}@media(max-width:750px){.columns{display:block}}
</style><main><h1>__TITLE__</h1><p class="status">合成场景，仅验证算法链路。不是实机录像、RK3588 性能或飞行测试结果。</p><div class="columns"><section><h2>72 个方向的保守距离</h2><svg id="polar" viewBox="0 0 460 460"></svg><small>前在上，右在右；绿色线为有效距离，灰色短线为未知。单位：米。仅覆盖机体系配置高度带。</small></section><section><h2>本次输出</h2><div id="summary"></div><p>来源：包内 <code>manifest.json</code> 合成四目图像，经鱼眼重投影、SGBM、左右一致性和不确定度过滤得到。处理耗时仅属于当前测试电脑。</p><h2>静态障碍制动估算</h2><label>可靠探测距离 D（米） <input id="distance" type="number" value="3" min="0" step="0.1"></label><label>总延迟 τ（秒） <input id="delay" type="number" value="0.2" min="0" step="0.01"></label><label>保守减速度 a（米/秒²） <input id="acceleration" type="number" value="1" min="0.01" step="0.1"></label><label>预留距离 M（米） <input id="margin" type="number" value="1" min="0" step="0.1"></label><strong id="speed"></strong><p><small>公式 D ≥ vτ + v²/(2a) + M。输入是可修改的假设，不是测得参数；未考虑移动障碍、风、控制饱和和漏检，不构成推荐飞行速度。</small></p></section></div><section><h2>接入边界</h2><p>PX4：按官方文档配置对应 Position 模式的 Collision Prevention。ArduPilot：按对应 Copter 版本配置 MAVLink proximity、Simple Avoidance 或 BendyRuler。消息到达不代表避障已生效；全未知也不保证两种飞控都会自动停车。</p><p>算法不发送解锁、模式切换、速度或位置控制命令。示例标定、回放和合成数据禁止通过 live 命令发往飞控。</p></section><section><h2>逐方向数据</h2><table><thead><tr><th>角度（顺时针）</th><th>距离（米）</th><th>筛选前有效点数</th></tr></thead><tbody id="rows"></tbody></table></section></main><script>
const data=__DATA__;const ns='http://www.w3.org/2000/svg';const svg=document.querySelector('#polar');const scale=29;function el(tag,attrs){let x=document.createElementNS(ns,tag);for(let k in attrs)x.setAttribute(k,attrs[k]);svg.appendChild(x);return x}
for(let r=1;r<=6;r++){el('circle',{cx:230,cy:230,r:r*scale,fill:'none',stroke:'#dce2dd'});let t=el('text',{x:234,y:230-r*scale+14,'font-size':11,fill:'#59615b'});t.textContent=r+' m'}
[['前',230,25],['右',425,235],['后',230,445],['左',12,235]].forEach(([s,x,y])=>{el('text',{x,y,'text-anchor':'middle','font-size':15}).textContent=s});
data.distances_cm.forEach((d,i)=>{let a=i*5*Math.PI/180,r=d===65535?6.3:d/100;el('line',{x1:230+Math.sin(a)*(d===65535?6:0)*scale,y1:230-Math.cos(a)*(d===65535?6:0)*scale,x2:230+Math.sin(a)*r*scale,y2:230-Math.cos(a)*r*scale,stroke:d===65535?'#a9b1ab':'#237351','stroke-width':d===65535?2:3});let row=document.createElement('tr');[i*5+'°',d===65535?'未知':(d/100).toFixed(2),data.support_counts[i]].forEach(v=>{let td=document.createElement('td');td.textContent=v;row.appendChild(td)});document.querySelector('#rows').appendChild(row)});
document.querySelector('#summary').textContent=`有效方向 ${data.valid_sectors}/72；处理耗时 ${data.processing_ms.toFixed(1)} ms；候选三维点 ${data.point_count}`;
function speed(){let D=+document.querySelector('#distance').value,t=+document.querySelector('#delay').value,a=+document.querySelector('#acceleration').value,M=+document.querySelector('#margin').value;let v=a>0&&D>M&&t>=0?Math.sqrt((a*t)**2+2*a*(D-M))-a*t:0;document.querySelector('#speed').textContent='上述假设下的制动速度上界：'+v.toFixed(2)+' m/s'}document.querySelectorAll('input').forEach(i=>i.addEventListener('input',speed));speed();
</script></html>'''
    content = template.replace("__TITLE__", html.escape(title)).replace("__DATA__", json.dumps(result).replace("</", "<\\/"))
    if not result.get("synthetic"):
        content = content.replace("合成场景，仅验证算法链路。不是实机录像、RK3588 性能或飞行测试结果。",
                                  "离线图像回放结果。处理耗时仅属于本次运行电脑，不代表 RK3588 性能或飞行测试结果。")
        content = content.replace("合成四目图像", "离线四目图像")
    Path(path).write_text(content, encoding="utf-8")

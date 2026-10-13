import { FEMBracket } from './physics.js';

const $ = id => document.getElementById(id);
const scene = $('scene'), chart = $('history');
const ui = {
  peak: $('peak'), rise: $('rise'), hold: $('hold'), gap: $('gap'),
  cycles: $('cycles'), strength: $('strength'), energy: $('energy'),
  mass: $('mass'), exaggeration: $('exaggeration'),
};
const formats = {
  peak:v=>`${v} N`, rise:v=>`${v} ms`, hold:v=>`${v} ms`,
  gap:v=>`${v} ms`, cycles:v=>`${v} 次`, strength:v=>`${v} MPa`,
  energy:v=>`${v} kJ/m²`, mass:v=>`${Number(v).toFixed(1)} kg`,
  exaggeration:v=>`${v}×`
};
const presets = {
  fast1600:{peak:1600,rise:25,hold:130,gap:160,cycles:3},
  slow1600:{peak:1600,rise:1000,hold:180,gap:250,cycles:1},
  slow3000:{peak:3000,rise:1000,hold:180,gap:250,cycles:1}
};
let model, parameters, history=[];
let playing=false, complete=false, accumulator=0, lastFrame=0, lastSample=-Infinity;

function readParams() {
  return {
    peak:+ui.peak.value, rise:+ui.rise.value/1000,
    hold:+ui.hold.value/1000,gap:+ui.gap.value/1000,cycles:+ui.cycles.value,
    strengthMPa:+ui.strength.value,fractureEnergy:+ui.energy.value*1000,
    pedalMass:+ui.mass.value,exaggeration:+ui.exaggeration.value
  };
}
function updateOutputs() {
  for (const [key,element] of Object.entries(ui)) $(`${key}-out`).textContent=formats[key](element.value);
}
function choosePreset(key) {
  if (!presets[key]) return;
  for(const [field,value] of Object.entries(presets[key])) ui[field].value=value;
  updateOutputs();reset();
}
function setStatus(message,kind='') {
  const badge=$('state-badge');badge.textContent=message;badge.className=`state ${kind}`.trim();
}
function reset() {
  playing=false;complete=false;accumulator=0;lastFrame=0;lastSample=-Infinity;
  parameters=readParams();
  model=new FEMBracket({ strengthMPa:parameters.strengthMPa,
    fractureEnergy:parameters.fractureEnergy,pedalMass:parameters.pedalMass });
  history=[];sample();setStatus('准备就绪');updateMetrics();
  $('run').textContent='▶ 开始实验';
  draw();
}
function sample() {
  const t=model.time;
  if(t-lastSample < 0.009 && !model.terminated) return;
  history.push({time:t,force:model.load,disp:-model.tipDisplacement()*1000,
    stress:model.maxStress/1e6,damage:model.maxDamage,failed:model.failedCount});
  lastSample=t;
}
function updateMetrics() {
  $('metric-time').innerHTML=`${model.time.toFixed(3)} <small>s</small>`;
  $('metric-force').innerHTML=`${Math.round(model.load).toLocaleString('zh-CN')} <small>N</small>`;
  $('metric-disp').innerHTML=`${(-model.tipDisplacement()*1000).toFixed(2)} <small>mm</small>`;
  $('metric-stress').innerHTML=`${(model.maxStress/1e6).toFixed(1)} <small>MPa</small>`;
  $('metric-damage').textContent=model.maxDamage.toFixed(3);
  $('metric-failed').innerHTML=`${model.failedCount} <small>/ ${model.elements.length}</small>`;
}
function simulationStep() {
  const dt=Math.min(0.002,Math.max(0.0004,parameters.rise/22));
  model.step(dt,parameters);
  sample();
  if(model.terminated || model.time>=FEMBracket.duration(parameters)) {
    complete=true;playing=false;accumulator=0;sample();
    const reason=model.error ? model.error:
      (!model.connected ? '载荷端已失去与固定端的网格连接' :
      model.failedCount ? '实验结束 · 检测到局部单元失效' : '实验完成 · 未检测到单元完全失效');
    setStatus(reason,model.error||!model.connected?'fail':'done');
    $('run').textContent='↻ 重新实验';
  }
}
function play() {
  if(complete) reset();
  playing=true;lastFrame=0;setStatus('计算中');$('run').textContent='▶ 继续';
}
$('run').addEventListener('click',play);
$('pause').addEventListener('click',()=>{
  if(!complete){playing=false;accumulator=0;setStatus('已暂停');}
});
$('reset').addEventListener('click',reset);
$('step').addEventListener('click',()=>{
  if(complete)return;
  playing=false;simulationStep();updateMetrics();draw();
  if(!complete) setStatus('单步推进');
});
$('preset').addEventListener('change',e=>{
  if(e.target.value==='custom') {reset();return;}
  choosePreset(e.target.value);
});
for(const [key,element] of Object.entries(ui)) {
  element.addEventListener('input',()=>{
    updateOutputs();
    if (key==='exaggeration') {draw();return;}
    if (['peak','rise','hold','gap','cycles'].includes(key)) $('preset').value='custom';
    reset();
  });
}
$('mesh-toggle').addEventListener('change',draw);
$('damage-toggle').addEventListener('change',draw);
$('csv').addEventListener('click',()=>{
  const header='time_s,force_N,downward_displacement_mm,max_principal_tension_MPa,max_damage,failed_elements';
  const body=history.map(v=>[v.time.toFixed(5),v.force.toFixed(3),v.disp.toFixed(5),
    v.stress.toFixed(5),v.damage.toFixed(6),v.failed].join(','));
  const csv=[header,...body].join('\r\n');
  const url=URL.createObjectURL(new Blob([csv],{type:'text/csv;charset=utf-8'}));
  const a=document.createElement('a');a.href=url;a.download='FEM-bracket-history.csv';a.click();
  setTimeout(()=>URL.revokeObjectURL(url),500);
});

function canvasSize(canvas) {
  const rect=canvas.getBoundingClientRect(), dpr=Math.min(2,window.devicePixelRatio||1);
  const width=Math.max(1,rect.width),height=Math.max(1,rect.height);
  const pw=Math.round(width*dpr),ph=Math.round(height*dpr);
  if(canvas.width!==pw||canvas.height!==ph){canvas.width=pw;canvas.height=ph;}
  const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);
  ctx.clearRect(0,0,width,height);
  return {ctx,width,height};
}
function rgbMix(a,b,t) {
  const z=a.map((v,i)=>Math.round(v+(b[i]-v)*Math.max(0,Math.min(1,t))));
  return `rgb(${z.join(',')})`;
}
function colorFor(d) {
  const stops=[[51,144,153],[109,174,141],[229,192,96],[227,127,83],[191,64,72]];
  const p=Math.max(0,Math.min(.9999,d))*(stops.length-1),i=Math.floor(p);
  return rgbMix(stops[i],stops[i+1],p-i);
}
function drawScene() {
  const {ctx,width:w,height:h}=canvasSize(scene);
  const scale=Math.min((w-(w<520?116:205))/FEMBracket.length,(h-130)/.057);
  const cx=w<520?40:Math.max(65,w*.105),cy=h*.48;
  const mag=+ui.exaggeration.value;
  const xy=(nodeId)=>{
    const p=model.nodes[nodeId];
    return [cx+(p.x+model.u[2*nodeId]*mag)*scale,
      cy-(p.y+model.u[2*nodeId+1]*mag)*scale];
  };
  ctx.lineWidth=1;
  ctx.strokeStyle='rgba(126,163,188,.08)';
  const gap=40;
  for(let x=cx%gap;x<w;x+=gap){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,h);ctx.stroke();}
  for(let y=cy%gap;y<h;y+=gap){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke();}
  // Fixed back plate.
  ctx.fillStyle='#718398';ctx.fillRect(cx-12,cy-33,10,66);
  ctx.strokeStyle='#8da4b9';ctx.lineWidth=1;
  for(let y=cy-30;y<cy+32;y+=10){ctx.beginPath();ctx.moveTo(cx-12,y);ctx.lineTo(cx-20,y+9);ctx.stroke();}
  // Plane-stress elements, deformed positions. Failed elements are omitted (erosion).
  const showDamage=$('damage-toggle').checked,showMesh=$('mesh-toggle').checked;
  for(const e of model.elements) {
    const points=e.nodeIds.map(xy);
    ctx.beginPath();ctx.moveTo(...points[0]);ctx.lineTo(...points[1]);ctx.lineTo(...points[2]);ctx.closePath();
    if(!e.failed) {
      const strength=Math.max(1,parameters.strengthMPa);
      const severity=showDamage? e.damage : e.stress/1e6/strength;
      ctx.fillStyle=colorFor(severity);ctx.fill();
      if(showMesh){ctx.strokeStyle='rgba(7,25,36,.45)';ctx.lineWidth=.7;ctx.stroke();}
    } else {
      ctx.strokeStyle='rgba(248,105,105,.70)';ctx.lineWidth=1.2;ctx.stroke();
      ctx.beginPath();ctx.moveTo(...points[0]);ctx.lineTo(...points[2]);ctx.stroke();
    }
  }
  // Represent a rigid input arm attached to the FEM load edge. Informational only.
  const tipId=model.loadNodes[Math.floor(model.loadNodes.length/2)];
  const [tx,ty]=xy(tipId);
  const armX=tx+Math.min(70,w*.13),armY=ty+45;
  ctx.lineCap='round';ctx.strokeStyle='#58687c';ctx.lineWidth=12;
  ctx.beginPath();ctx.moveTo(tx,ty);ctx.lineTo(armX,armY);ctx.stroke();
  ctx.strokeStyle='#a5b7c8';ctx.lineWidth=5;
  ctx.beginPath();ctx.moveTo(tx,ty);ctx.lineTo(armX,armY);ctx.stroke();
  ctx.fillStyle='#c2cfda';ctx.save();ctx.translate(armX,armY);ctx.rotate(.72);ctx.fillRect(-17,-6,34,12);ctx.restore();
  ctx.fillStyle='#d9e6ee';ctx.beginPath();ctx.arc(tx,ty,4.3,0,2*Math.PI);ctx.fill();
  // Force arrow at the load DOF, direction is downward.
  const arrowX=armX+28, arrowY=armY-5;
  const length=25+Math.min(42,model.load/3500*42);
  ctx.strokeStyle='#f0a36c';ctx.lineWidth=3;ctx.beginPath();
  ctx.moveTo(arrowX,arrowY-length);ctx.lineTo(arrowX,arrowY);ctx.stroke();
  ctx.fillStyle='#f0a36c';ctx.beginPath();ctx.moveTo(arrowX-6,arrowY-8);ctx.lineTo(arrowX,arrowY+4);ctx.lineTo(arrowX+6,arrowY-8);ctx.fill();
  ctx.font='12px ui-sans-serif,system-ui';ctx.fillStyle='#f0ad79';
  ctx.textAlign='center';ctx.fillText(`F(t) = ${Math.round(model.load)} N`,arrowX,arrowY-length-10);
  ctx.font='11px ui-sans-serif,system-ui';ctx.fillStyle='#a8c0d3';
  ctx.textAlign='left';ctx.fillText('固定端',Math.max(7,cx-30),cy-44);
  ctx.fillText('支架 / CST 网格',cx+8,cy+59);
  ctx.fillText('等效作用端',Math.min(w-115,tx+15),Math.max(20,ty-37));
  if(model.maxDamage>0.01) {
    const severity=model.maxDamage;
    ctx.textAlign='right';ctx.fillStyle=severity>.9?'#f4a2a5':'#e6cb83';
    ctx.fillText(`最大损伤 D = ${severity.toFixed(3)}`,w-15,h-18);
  }
  if(model.terminated){
    ctx.fillStyle='rgba(8,17,27,.92)';ctx.fillRect(0,h-45,w,45);
    ctx.textAlign='center';ctx.font='bold 12px ui-sans-serif,system-ui';ctx.fillStyle='#f9bbb0';
    ctx.fillText(model.failedCount?'局部断裂已发生 · 计算在小应变极限处停止':'计算在小应变极限处停止',w/2,h-27);
    ctx.font='10px ui-sans-serif,system-ui';ctx.fillStyle='#c7d4df';
    ctx.fillText('请降低载荷或提高材料参数后重新实验',w/2,h-11);
  }
}
function drawChart() {
  const {ctx,width:w,height:h}=canvasSize(chart),left=51,right=w-49,top=18,bottom=h-34;
  const xMax=Math.max(.1,FEMBracket.duration(parameters)),peak=Math.max(500,parameters.peak);
  const maxDisp=Math.max(1,...history.map(v=>Math.abs(v.disp)))*1.1;
  const px=t=>left+(right-left)*Math.min(1,t/xMax);
  const axis=(v,max)=>bottom-(bottom-top)*Math.max(0,Math.min(1,v/max));
  ctx.font='10px ui-sans-serif,system-ui';ctx.textAlign='right';
  for(let i=0;i<=4;i++) {
    const y=top+(bottom-top)*i/4;
    ctx.strokeStyle='#29394a';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(right,y);ctx.stroke();
    ctx.fillStyle='#819aaf';ctx.fillText(`${Math.round(peak*(4-i)/4)}`,left-8,y+4);
  }
  ctx.textAlign='left';ctx.fillStyle='#93afc2';ctx.fillText('N',12,12);
  ctx.textAlign='right';ctx.fillText('mm / D',w-7,12);
  ctx.fillText(`${maxDisp.toFixed(1)} mm`,w-7,top+9);
  ctx.fillText('0',w-7,bottom+3);
  ctx.textAlign='center';
  for(let i=0;i<=4;i++){
    const x=left+(right-left)*i/4;
    ctx.fillStyle='#8097a9';ctx.fillText((xMax*i/4).toFixed(2),x,bottom+17);
  }
  ctx.fillText('模拟时间 (s)',w/2,h-3);
  function curve(prop,max,color) {
    if(history.length<2)return;
    ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();
    history.forEach((v,i)=>{
      const x=px(v.time),y=axis(Math.max(0,v[prop]),max);
      if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);
    });ctx.stroke();
  }
  curve('force',peak,'#e4a75d');curve('disp',maxDisp,'#64bfdb');curve('damage',1,'#ee7476');
  const marker=px(model.time);
  ctx.strokeStyle='rgba(220,238,249,.38)';ctx.setLineDash([4,4]);
  ctx.beginPath();ctx.moveTo(marker,top);ctx.lineTo(marker,bottom);ctx.stroke();ctx.setLineDash([]);
}
function draw() { if(!model)return;drawScene();drawChart(); }
new ResizeObserver(draw).observe(scene);
new ResizeObserver(draw).observe(chart);
function frame(timestamp) {
  if(playing) {
    if(!lastFrame)lastFrame=timestamp;
    const elapsed=Math.min(.06,(timestamp-lastFrame)/1000);
    lastFrame=timestamp;
    const stepDt=Math.min(.002,Math.max(.0004,parameters.rise/22));
    accumulator+=elapsed*Number($('play-speed').value);
    // Avoid blocking rendering on slow devices; never skip physics time.
    let count=0;
    while(accumulator>=stepDt && count<22 && playing) {
      simulationStep();accumulator-=stepDt;count++;
    }
    if(count>=22)accumulator=Math.min(accumulator,stepDt*22);
  } else lastFrame=0;
  updateMetrics();draw();
  requestAnimationFrame(frame);
}
updateOutputs();reset();requestAnimationFrame(frame);

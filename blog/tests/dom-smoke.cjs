'use strict';
// Event/DOM integration checks against the real inline scripts.
// Geometry and requestAnimationFrame are controlled fixtures.
// This is not a browser rendering, WebGL, CSS layout or physical input test.

{
const {JSDOM}=require('jsdom');
const fs=require('node:fs'), assert=require('node:assert/strict');
const html=fs.readFileSync(require('node:path').join(__dirname,'..','laplacian.html'),'utf8');
const dom=new JSDOM(html,{url:'https://example.test/blog/laplacian.html?renderer=svg',runScripts:'dangerously',pretendToBeVisual:true,beforeParse(w){Object.defineProperty(w.HTMLElement.prototype,'clientWidth',{get(){return this.id==='stage'?602:960}});Object.defineProperty(w.HTMLElement.prototype,'clientHeight',{get(){return this.id==='stage'?288:480}});}});
const w=dom.window,d=w.document,$=id=>d.getElementById(id),change=(id,value,type='change')=>{$(id).value=value;$(id).dispatchEvent(new w.Event(type,{bubbles:true}));};
assert.match($('status').textContent,/SVG/);
assert.equal($('exact').textContent,'4');
for(const [preset,answer] of [['inverted','-4'],['plane','0'],['saddle','0'],['bowl','4']]){change('preset',preset);assert.equal($('exact').textContent,answer);assert.equal($('discrete').textContent,answer);assert.equal($('fallback').querySelectorAll('circle').length,5);}
change('preset','quartic');change('center-x','0.5','input');change('center-y','-0.25','input');change('spacing','0.4','input');
assert.equal($('exact').textContent,'0.9375');assert.equal($('discrete').textContent,'1.0975');assert.equal($('error').textContent,'0.16');change('spacing','0.2','input');assert.equal($('error').textContent,'0.04');
assert.match($('stage').getAttribute('aria-label'),/0.5, -0.25/);
assert.equal(new URL($('open-page').href).searchParams.get('preset'),'quartic');
const before=$('fallback').innerHTML;$('rotate-left').click();assert.notEqual($('fallback').innerHTML,before);$('reset').click();assert.equal($('fallback').innerHTML,before);
$('zoom-in').click();assert.notEqual($('fallback').innerHTML,before);$('reset').click();assert.equal($('fallback').innerHTML,before);
$('interact').click();assert.equal($('interact').getAttribute('aria-pressed'),'true');assert.equal($('stage').style.touchAction,'none');$('interact').click();assert.equal($('stage').style.touchAction,'pan-y');
console.log('PASS: full inline UI scripts in jsdom; 5 preset selections, parameter values, h² convergence, five points, link state, camera/reset, interaction toggle. This does not test CSS layout, real input devices or WebGL.');w.close();
}

{
const {JSDOM}=require('jsdom');
const fs=require('node:fs'),assert=require('node:assert/strict');
const html=fs.readFileSync(require('node:path').join(__dirname,'..','heat.html'),'utf8');let tick;
const dom=new JSDOM(html,{url:'https://example.test/blog/heat.html',runScripts:'dangerously',pretendToBeVisual:true,beforeParse(w){w.requestAnimationFrame=fn=>{tick=fn;return 1};w.Element.prototype.getBoundingClientRect=function(){return {left:0,top:0,width:210,height:210,right:210,bottom:210}};w.Element.prototype.setPointerCapture=function(id){this.capture=id};w.Element.prototype.hasPointerCapture=function(id){return this.capture===id};w.Element.prototype.releasePointerCapture=function(){this.capture=null};}});
const w=dom.window,d=w.document,$=id=>d.getElementById(id),change=(id,value,type='change')=>{$(id).value=value;$(id).dispatchEvent(new w.Event(type,{bubbles:true}));};
const key=k=>$('heat-grid').dispatchEvent(new w.KeyboardEvent('keydown',{key:k,bubbles:true,cancelable:true}));const draw=checked=>{$('draw').checked=checked;$('draw').dispatchEvent(new w.Event('change',{bubbles:true}))};
assert.equal(d.querySelectorAll('[role=gridcell]').length,441);assert.equal($('temp-c').textContent,'80℃');assert.equal($('draw').checked,false);
change('preset','uniform');assert.equal($('temp-c').textContent,'50℃');
draw(true);change('brush-radius','0','input');change('brush-temp','100');key(' ');
assert.equal($('temp-c').textContent,'100℃');assert.equal($('neighbor-mean').textContent,'50℃');assert.equal($('next-change').textContent,'-20℃');assert.equal($('next-temp').textContent,'80℃');assert.equal($('preset').value,'custom');
$('step').click();assert.equal($('temp-c').textContent,'80℃');assert.equal($('draw').checked,false);assert.match($('clock').textContent,/第 1 步/);$('reset').click();assert.equal($('temp-c').textContent,'100℃');assert.match($('clock').textContent,/第 0 步/);
$('play').click();assert.equal($('play').textContent,'暂停');tick(1000);tick(1200);assert.match($('clock').textContent,/第 2 步/);$('play').click();tick(1500);assert.match($('clock').textContent,/第 2 步/);
draw(true);assert.equal($('temp-c').textContent,'100℃');assert.match($('clock').textContent,/第 0 步/);
change('boundary','cold');for(let i=0;i<20;i++){key('ArrowLeft');key('ArrowUp');}assert.equal($('temp-c').textContent,'20℃');key(' ');assert.equal($('temp-c').textContent,'20℃');assert.match($('explanation').textContent,/恒温/);assert.equal($('next-change').textContent,'0℃');assert.equal($('laplacian').textContent,'不推进');
change('spacing','0.1');change('dt','10');assert.equal(Number($('dt').value),0.0025);assert.match($('stability').textContent,/0.25/);
change('alpha','');assert.equal($('alpha').value,'1');
change('boundary','insulated');change('preset','uniform');draw(true);change('brush-temp','80');key(' ');change('spacing','100');change('alpha','0.001');change('dt','0.000001');
assert.ok(!/中心等于/.test($('explanation').textContent),'small nonzero temperature change must not be described as equilibrium');
const e=new w.Event('pointerdown',{bubbles:true,cancelable:true});Object.assign(e,{clientX:105,clientY:105,button:0,pointerId:1,pointerType:'mouse'});$('heat-grid').dispatchEvent(e);assert.match($('selection-title').textContent,/11, 11/);assert.equal($('temp-c').textContent,'80℃');
console.log('PASS: actual heat UI script in jsdom; painting, preset, boundaries, keyboard navigation, step/play/pause/reset, parameter clamps and low-r explanation. Not a browser/layout/input-device test.');w.close();
}

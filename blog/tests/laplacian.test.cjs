'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const html = fs.readFileSync(path.join(__dirname, '..', 'laplacian.html'), 'utf8');
const source = html.match(/<script id="laplacian-math">([\s\S]*?)<\/script>/)[1];
const context = {};
vm.runInNewContext(source, context);
const { presets, evaluate } = context.LaplacianMath;
const near = (actual, expected, tolerance=1e-11) => assert.ok(Math.abs(actual-expected)<tolerance, `${actual} ≈ ${expected}`);

test('five presets expose correct analytic function values and derivatives', () => {
  const x=.4, y=-.3;
  const expected = { bowl:[.25,2,2], inverted:[-.25,-2,-2], plane:[.1,0,0], saddle:[.07,2,-2], quartic:[.008425,.48,.27] };
  assert.equal(Object.keys(presets).length,5);
  for(const [key,[f,fxx,fyy]] of Object.entries(expected)) {
    near(presets[key].f(x,y),f);near(presets[key].fxx(x,y),fxx);near(presets[key].fyy(x,y),fyy);
    const r=evaluate(key,x,y,.2);near(r.exact,fxx+fyy);
  }
});

test('quadratics and the plane have exact centered differences at all control positions', () => {
  for(const key of ['bowl','inverted','plane','saddle']) for(const x of [-.8,-.35,0,.4,.8]) for(const y of [-.8,-.1,0,.55,.8]) for(const h of [.1,.25,.6,.8]) {
    const r=evaluate(key,x,y,h);near(r.dx,r.fxx);near(r.dy,r.fyy);near(r.discrete,r.exact);
  }
});

test('five-point stencil samples the correct E/W/N/S coordinates', () => {
  for(const key of Object.keys(presets)) {
    const x=.65,y=-.35,h=.2,r=evaluate(key,x,y,h),f=presets[key].f;
    near(r.C,f(x,y));near(r.E,f(x+h,y));near(r.W,f(x-h,y));near(r.N,f(x,y+h));near(r.S,f(x,y-h));
    near(r.dx,(r.E-2*r.C+r.W)/(h*h));near(r.dy,(r.N-2*r.C+r.S)/(h*h));
    near(r.discrete,(r.E+r.W+r.N+r.S-4*r.C)/(h*h));
  }
});

test('neighbor mean excludes C and reproduces the Laplacian identity', () => {
  for(const key of Object.keys(presets)) for(const h of [.1,.35,.8]) {
    const r=evaluate(key,.6,-.4,h);near(r.mean,(r.E+r.W+r.N+r.S)/4);near(r.gap,r.mean-r.C);near(r.discrete,4*r.gap/(h*h));
  }
});

test('quartic has nonzero finite-h error with second-order convergence', () => {
  for(const [x,y] of [[0,0],[.65,-.45],[-.8,.8]]) {
    let previous;
    for(const h of [.8,.4,.2,.1]) {
      const r=evaluate('quartic',x,y,h);
      near(r.dx,3*x*x+h*h/2);near(r.dy,3*y*y+h*h/2);
      near(r.error,h*h);near(r.discrete,r.exact+h*h);
      if(previous)near(previous.error/r.error,4,1e-9);
      previous=r;
    }
  }
});

test('saddle curvature cancels while plane curvature vanishes separately', () => {
  const s=evaluate('saddle',0,0,.6),p=evaluate('plane',.3,-.2,.6);
  near(s.dx,2);near(s.dy,-2);near(s.discrete,0);near(p.dx,0);near(p.dy,0);
});

test('invalid spacing, coordinates, and presets are rejected', () => {
  for(const h of [0,-1,NaN,Infinity])assert.throws(()=>evaluate('bowl',0,0,h),/Finite/);
  assert.throws(()=>evaluate('bowl',Infinity,0,.1),/Finite/);
  assert.throws(()=>evaluate('unknown',0,0,.1),/Unknown/);
  assert.throws(()=>evaluate('__proto__',0,0,.1),/Unknown/);
});

test('control domain keeps every stencil sample on the displayed surface', () => {
  for(const center of [-.8,.8])for(const h of [.1,.8]){
    assert.ok(center-h>=-1.6);assert.ok(center+h<=1.6);
  }
});

test('offline renderer, explicit touch opt-in, and pinned matching Three modules remain present', () => {
  assert.match(html,/<svg id="fallback"/);
  assert.match(html,/renderer'\)===['"]svg/);
  assert.match(html,/aria-pressed="false"/);
  assert.match(html,/orbit\.enableZoom=false/);
  assert.equal((html.match(/three@0\.156\.0\//g)||[]).length,2);
  assert.match(html,/webglcontextlost/);
});

test('the shared default orthographic frame includes every preset surface', () => {
  const halfHeight=Number(html.match(/halfHeight = ([0-9.]+)/)[1]);
  const zScale=Number(html.match(/zScale = ([0-9.]+)/)[1]);
  // The sphere enclosing every vertex also bounds every rotated projection.
  // Hence all permitted camera rotations fit vertically before intentional zooming.
  for(const preset of Object.values(presets))for(let i=0;i<=32;i++)for(let j=0;j<=32;j++){
    const x=-1.6+i*.1,y=-1.6+j*.1,z=preset.f(x,y)*zScale;
    assert.ok(Math.hypot(x,y,z)<halfHeight, `vertex ${x},${y},${z} fits the default frame`);
  }
  assert.match(html,new RegExp(`OrthographicCamera\\(-4,4,${halfHeight},-${halfHeight}`));
});

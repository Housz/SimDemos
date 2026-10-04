'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const html = fs.readFileSync(path.join(__dirname, '..', 'heat.html'), 'utf8');
const context = vm.createContext({});
const engine = html.match(/<script id="heat-engine">([\s\S]*?)<\/script>/);
assert.ok(engine, 'the numerical model is independently extractable');
vm.runInContext(engine[1], context);
const H = context.HeatSolver;
const close = (actual, expected, eps = 1e-9) => assert.ok(Math.abs(actual - expected) <= eps, `${actual} ≠ ${expected}`);
const values = grid => Array.from(grid);

test('stability is hard-clamped, including extreme and invalid inputs', () => {
  for (const h of [NaN, Infinity, -2, 0, .001, .01, 1, '']) {
    for (const alpha of [NaN, -1, 0, 1e-7, .01, Infinity]) {
      for (const dt of [NaN, Infinity, -1, 1e-6, .1, 1000, '']) {
        const p = H.parameters({ h, alpha, dt });
        for (const value of Object.values(p)) assert.ok(Number.isFinite(value));
        assert.ok(p.r >= 0 && p.r <= .25);
        assert.ok(p.dt <= p.maxDt);
        close(p.r, p.alpha * p.dt / p.h ** 2);
      }
    }
  }
});

test('a uniform insulated grid remains exactly uniform, even at r = 1/4', () => {
  const m = H.createModel({ preset: 'uniform', dt: 100 });
  H.advance(m, 300);
  for (const t of m.current) assert.equal(t, 50);
  assert.equal(m.steps, 300);
});

test('insulated evolution conserves heat and satisfies the maximum principle', () => {
  const initial = Array.from({ length: 21 * 21 }, (_, i) => (i * 73 % 101));
  const m = H.createModel({ initial, dt: .25 });
  const before = H.statistics(m);
  for (let k = 0; k < 300; k++) {
    const previous = H.statistics(m);
    H.advance(m);
    const after = H.statistics(m);
    close(after.sum, before.sum, 1e-7);
    assert.ok(after.min >= previous.min - 1e-12);
    assert.ok(after.max <= previous.max + 1e-12);
  }
});

test('insulated corner uses its own value for missing neighbors', () => {
  const initial = Array(25).fill(20); initial[0] = 80;
  const m = H.createModel({ n: 5, initial });
  const q = H.inspect(m, 0, 0);
  assert.equal(q.w, 80); assert.equal(q.n, 80); assert.equal(q.e, 20); assert.equal(q.s, 20);
  close(q.mean, 50); close(q.change, -12);
  H.advance(m); close(m.current[0], 68);
});

test('article one-step example is 80 + 0.1 × (4 × 20 - 4 × 80) = 56', () => {
  const initial = Array(25).fill(20); initial[12] = 80;
  const m = H.createModel({ n: 5, initial, boundary: 'cold' });
  close(m.params.r, .1);
  const q = H.inspect(m, 2, 2);
  close(q.laplacian, -2400000); close(q.change, -24); close(q.next, 56);
  H.advance(m); close(m.current[12], 56); close(m.time, .1);
  for (const index of [7, 11, 13, 17]) close(m.current[index], 26);
});

test('all fixed sides and corner precedence stay exact and cannot be painted', () => {
  for (const boundary of ['cold', 'hot-cold']) {
    const m = H.createModel({ n: 7, boundary, preset: 'uniform' });
    H.paintInitial(m, 0, 0, 99, 4);
    for (let k = 0; k < 20; k++) {
      H.advance(m);
      for (let y = 0; y < m.n; y++) for (let x = 0; x < m.n; x++) {
        const expected = H.fixedTemperature(x, y, m.n, boundary);
        if (expected !== null) { assert.equal(m.current[y * m.n + x], expected); assert.equal(m.initial[y * m.n + x], expected); }
      }
    }
    assert.equal(H.inspect(m, 0, 0).change, 0);
    assert.equal(H.inspect(m, 0, 0).laplacian, null);
  }
  const m = H.createModel({ n: 5, boundary: 'hot-cold' });
  assert.equal(m.current[0], 50); assert.equal(m.current[10], 100); assert.equal(m.current[14], 0); assert.equal(m.current[22], 50);
});

test('time stepping reads one old state, swaps buffers, and leaves saved initial intact', () => {
  const initial = Array(49).fill(20); initial[24] = 80;
  const m = H.createModel({ n: 7, initial });
  const old = m.current, next = m.next, saved = values(m.initial);
  H.advance(m);
  assert.equal(m.current, next); assert.equal(m.next, old);
  assert.deepEqual(values(old), initial);
  assert.deepEqual(values(m.initial), saved);
  // Two cells away must not receive freshly updated heat in the same step.
  assert.equal(m.current[26], 20); assert.equal(m.current[38], 20);
  for (const index of [17, 23, 25, 31]) close(m.current[index], 26);
  assert.throws(() => H.fillNext(m.current, m.current, 7, m.params, m.boundary), /separate buffers/);
});

test('h, alpha, dt determine r and a smaller h triggers a shorter safe dt', () => {
  close(H.parameters({ h: .01, alpha: 1e-4, dt: .1 }).r, .1);
  close(H.parameters({ h: .02, alpha: 1e-4, dt: .1 }).r, .025);
  close(H.parameters({ h: .01, alpha: 2e-4, dt: .1 }).r, .2);
  const p = H.parameters({ h: .002, alpha: 1e-4, dt: .1 });
  close(p.dt, .01); close(p.r, .25);
  const m = H.createModel(); H.advance(m, 3); H.setParameters(m, { dt: .2 }); H.advance(m);
  assert.equal(m.steps, 4); close(m.time, .5);
});

test('custom initial survives simulation and reset, including boundary changes', () => {
  const m = H.createModel({ preset: 'uniform' });
  H.paintInitial(m, 8, 7, 91, 2);
  const saved = values(m.initial);
  H.advance(m, 40); assert.notDeepEqual(values(m.current), saved);
  H.reset(m);
  assert.deepEqual(values(m.current), saved); assert.equal(m.steps, 0); assert.equal(m.time, 0);
  H.setBoundary(m, 'hot-cold');
  assert.equal(m.current[7 * m.n + 8], 91); assert.equal(m.current[0], 50);
  H.advance(m); H.reset(m); assert.deepEqual(values(m.current), values(m.initial));
});

test('paint input is finite and clamped; initial arrays are copied and sanitized', () => {
  const initial = Array(25).fill(20); initial[0] = NaN; initial[1] = 300; initial[2] = -30;
  const m = H.createModel({ n: 5, initial });
  assert.equal(m.current[0], 20); assert.equal(m.current[1], 100); assert.equal(m.current[2], 0);
  initial[12] = 999; assert.equal(m.current[12], 20);
  H.paintInitial(m, NaN, Infinity, NaN, NaN);
  H.paintInitial(m, 2, 2, Infinity, -1);
  for (const temp of m.current) assert.ok(Number.isFinite(temp) && temp >= 0 && temp <= 100);
});

test('touch scrolling is default and explicit draw mode enables capture; no CDN dependencies', () => {
  assert.match(html, /touch-action:pan-y/);
  assert.match(html, /\.field svg\.drawing\{touch-action:none/);
  assert.match(html, /if \(!\$\('draw'\)\.checked\) return;/);
  assert.doesNotMatch(html, /<script[^>]*\bsrc=/);
  assert.match(html, /id="draw" type="checkbox"/);
});

// A small DOM/event harness checks wiring without pretending to replace real
// browser layout, accessibility-tree, or touch-device verification.
function uiHarness() {
  const elements = new Map();
  const callbacks = [];
  class Element {
    constructor(id = '') {
      this.id = id; this.value = ''; this.checked = false; this.disabled = false;
      this.textContent = ''; this.style = {}; this.attributes = {}; this.events = {};
      this.children = []; this.classes = new Set(); this.captures = new Set();
      this.classList = { toggle: (name, on) => on ? this.classes.add(name) : this.classes.delete(name) };
    }
    setAttribute(name, value) { this.attributes[name] = String(value); if (name === 'id') { this.id = value; elements.set(value, this); } }
    append(...children) { this.children.push(...children); }
    addEventListener(name, fn) { (this.events[name] ||= []).push(fn); }
    focus() { this.focused = true; }
    setPointerCapture(id) { this.captures.add(id); }
    hasPointerCapture(id) { return this.captures.has(id); }
    releasePointerCapture(id) { this.captures.delete(id); }
    getBoundingClientRect() { return { left: 0, top: 0, width: 210, height: 210 }; }
    dispatch(name, props = {}) {
      const e = { button: 0, pointerType: 'mouse', pointerId: 1, clientX: 105, clientY: 105, prevented: false, preventDefault() { this.prevented = true; }, ...props };
      for (const callback of this.events[name] || []) callback(e);
      return e;
    }
  }
  for (const match of html.matchAll(/\bid="([^"]+)"/g)) elements.set(match[1], new Element(match[1]));
  const defaults = { preset: 'hotspot', boundary: 'insulated', 'brush-temp': '80', 'brush-radius': '1', spacing: '1', alpha: '1', dt: '.1' };
  for (const [id, value] of Object.entries(defaults)) elements.get(id).value = value;
  const document = { getElementById: id => elements.get(id), createElementNS: () => new Element(), addEventListener() {}, hidden: false };
  const ctx = vm.createContext({ document, performance: { now: () => 1000 }, requestAnimationFrame: fn => callbacks.push(fn) });
  vm.runInContext(engine[1], ctx);
  vm.runInContext(html.match(/<script id="heat-ui">([\s\S]*?)<\/script>/)[1], ctx);
  return { get: id => elements.get(id), tick: now => callbacks.shift()(now) };
}

test('UI initializes, steps, pauses, resets, and allows repeated play without duplicate loops', () => {
  const ui = uiHarness();
  assert.equal(ui.get('cells').children.length, 21);
  assert.match(ui.get('clock').textContent, /第 0 步/);
  ui.get('step').dispatch('click'); assert.match(ui.get('clock').textContent, /第 1 步/);
  ui.get('play').dispatch('click'); assert.equal(ui.get('play').textContent, '暂停');
  ui.tick(1100); assert.match(ui.get('clock').textContent, /第 2 步/);
  ui.get('play').dispatch('click'); ui.tick(1300); assert.match(ui.get('clock').textContent, /第 2 步/);
  ui.get('play').dispatch('click'); ui.tick(1500); assert.match(ui.get('clock').textContent, /第 3 步/);
  ui.get('reset').dispatch('click'); ui.tick(1700); assert.match(ui.get('clock').textContent, /第 0 步/);
});

test('UI captures touch only in explicit drawing mode and restores custom paint', () => {
  const ui = uiHarness(), grid = ui.get('heat-grid');
  const touch = { pointerType: 'touch', pointerId: 17, clientX: 105, clientY: 105 };
  const observe = grid.dispatch('pointerdown', touch);
  assert.equal(observe.prevented, false); assert.equal(grid.captures.size, 0);
  ui.get('draw').checked = true; ui.get('draw').dispatch('change');
  ui.get('brush-temp').value = '94'; ui.get('brush-radius').value = '0';
  const draw = grid.dispatch('pointerdown', touch);
  assert.equal(draw.prevented, true); assert.equal(grid.hasPointerCapture(17), true);
  assert.equal(ui.get('temp-c').textContent, '94℃'); assert.equal(ui.get('preset').value, 'custom');
  grid.dispatch('pointercancel', touch); assert.equal(grid.captures.size, 0);
  ui.get('step').dispatch('click'); assert.equal(ui.get('draw').checked, false);
  assert.notEqual(ui.get('temp-c').textContent, '94℃');
  ui.get('reset').dispatch('click'); assert.equal(ui.get('temp-c').textContent, '94℃');
});

test('UI keyboard painting respects fixed edges, and invalid parameters cannot show NaN', () => {
  const ui = uiHarness(), grid = ui.get('heat-grid');
  ui.get('boundary').value = 'hot-cold'; ui.get('boundary').dispatch('change');
  ui.get('draw').checked = true; ui.get('draw').dispatch('change'); ui.get('brush-radius').value = '0';
  for (let i = 0; i < 10; i++) grid.dispatch('keydown', { key: 'ArrowLeft' });
  ui.get('brush-temp').value = '10'; grid.dispatch('keydown', { key: ' ' });
  assert.equal(ui.get('temp-c').textContent, '100℃'); assert.equal(ui.get('next-change').textContent, '0℃');
  assert.match(ui.get('explanation').textContent, /恒温边界/);
  ui.get('spacing').value = '.2'; ui.get('dt').value = '999'; ui.get('spacing').dispatch('change');
  assert.equal(ui.get('stability').textContent, 'r = 0.25 ≤ 1/4');
  assert.match(ui.get('status').textContent, /自动缩短/);
  ui.get('alpha').value = ''; ui.get('spacing').value = 'NaN'; ui.get('dt').value = ''; ui.get('dt').dispatch('change');
  assert.doesNotMatch(ui.get('stability').textContent + ui.get('clock').textContent + ui.get('laplacian').textContent, /NaN/);
});

test('a very small nonzero r and temperature change never masquerade as equilibrium', () => {
  const ui = uiHarness();
  ui.get('preset').value = 'uniform'; ui.get('preset').dispatch('change');
  ui.get('draw').checked = true; ui.get('draw').dispatch('change');
  ui.get('brush-temp').value = '80'; ui.get('brush-radius').value = '0';
  ui.get('heat-grid').dispatch('keydown', { key: ' ' });
  ui.get('spacing').value = '100'; ui.get('alpha').value = '.001'; ui.get('dt').value = '.000001'; ui.get('dt').dispatch('change');
  assert.match(ui.get('stability').textContent, /1\.00e-13/);
  assert.match(ui.get('next-change').textContent, /-1\.20e-11/);
  assert.match(ui.get('explanation').textContent, /降温/);
  assert.doesNotMatch(ui.get('explanation').textContent, /等于/);
});

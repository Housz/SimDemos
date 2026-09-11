// ============================================================================
// 演示层冒烟测试：把 8 个场景在 Node 里无头跑一遍
//
// 为什么需要它：浏览器里的表现没法自动化验证，但场景代码本身可以。
// 这里用一个「假 harness」顶替 js/harness.js（后者依赖 WebGL / DOM / lil-gui，
// 在 Node 里实例化不了），而**物理世界、刚体、关节全部用真实的核心库**，于是：
//   · 场景模块的语法错误、坏 import、拼错的 API  → 立刻暴露
//   · 场景的数值配置（关节锚点、限制角、柔度、初始速度）是否稳定 → 可量化
//   · GUI 回调（重建、改限制、切换模式）是否抛异常 → 全部点一遍
//
// 运行（在 rigidPBD/ 目录下）：
//   node --import ./test/register.mjs test/scenes.test.mjs
// ============================================================================

import * as THREE from 'three';
import { World } from '../js/core/world.js';
import { RigidBody } from '../js/core/rigidBody.js';
import { ShapeType, plane as planeShape } from '../js/core/shapes.js';
import { scenes } from '../js/scenes/registry.js';

let pass = 0, fail = 0;
function check(name, cond, detail = '') {
    if (cond) { pass++; console.log(`  ✓ ${name} ${detail}`); }
    else { fail++; console.log(`  ✗ ${name} ${detail}`); }
}
/** 诊断数字，不计成败 */
function info(label, value) { console.log(`    · ${label}：${value}`); }

const DT = 1 / 60;

// ---------------------------------------------------------------------------
// 形状 → three.js 几何体（与 js/harness.js 的 makeGeometry 同构）。
// 不直接 import harness.js，因为它会连带拉进 OrbitControls / lil-gui / DOM。
// ---------------------------------------------------------------------------
function makeGeometry(shape) {
    switch (shape.type) {
        case ShapeType.SPHERE: return new THREE.SphereGeometry(shape.radius, 16, 10);
        case ShapeType.BOX:
            return new THREE.BoxGeometry(
                shape.halfExtents.x * 2, shape.halfExtents.y * 2, shape.halfExtents.z * 2);
        case ShapeType.CAPSULE: return new THREE.CapsuleGeometry(shape.radius, shape.length, 6, 12);
        case ShapeType.CYLINDER: return new THREE.CylinderGeometry(shape.radius, shape.radius, shape.height, 16);
        default: return new THREE.BoxGeometry(1, 1, 1);
    }
}

// ---------------------------------------------------------------------------
// lil-gui 的假实现：把控制器都记下来，测试里可以逐个「操作」它们
// ---------------------------------------------------------------------------
class FakeController {
    constructor(obj, prop, kind) {
        this.obj = obj; this.prop = prop; this.kind = kind;
        this.label = prop;
        this.onChangeFn = null;
        this.min = null; this.max = null;
        this.options = null;
    }
    name(n) { this.label = n; return this; }
    onChange(fn) { this.onChangeFn = fn; return this; }
    updateDisplay() { return this; }
    destroy() { }
    /** 模拟用户拖动滑杆 / 选择下拉项：写入新值并触发 onChange */
    apply(v) {
        const old = this.obj[this.prop];
        this.obj[this.prop] = v;
        if (this.onChangeFn) this.onChangeFn(v);
        return old;
    }
    /** 模拟点击按钮 */
    click() { return this.obj[this.prop](); }
}

class FakeGUI {
    constructor(name = 'root') { this.name = name; this.items = []; this.folders = []; this.parent = null; this.destroyed = false; }
    add(obj, prop, a, b, c) {
        let kind = 'number';
        if (typeof obj[prop] === 'function') kind = 'button';
        else if (typeof obj[prop] === 'boolean') kind = 'boolean';
        else if (typeof obj[prop] === 'string') kind = 'options';
        const ctrl = new FakeController(obj, prop, kind);
        if (kind === 'options' && a && typeof a === 'object') ctrl.options = a;
        if (kind === 'number' && typeof a === 'number') { ctrl.min = a; ctrl.max = b; ctrl.step = c; }
        this.items.push(ctrl);
        return ctrl;
    }
    addFolder(name) { const f = new FakeGUI(name); f.parent = this; this.folders.push(f); return f; }
    controllersRecursive() {
        return [...this.items, ...this.folders.filter((f) => !f.destroyed).flatMap((f) => f.controllersRecursive())];
    }
    /**
     * 与 lil-gui 一致：销毁时把自己从父级摘掉。
     * 这一点必须真实——harness 每次 setScene 都会 destroy 旧文件夹再建新的，
     * 如果这里是个空操作，历次重建的文件夹（连同它们闭包捕获的 world /
     * 网格 / 场景对象）会一直挂在根 GUI 上，测试跑到底就 OOM 了。
     */
    destroy() {
        this.destroyed = true;
        if (this.parent) {
            const i = this.parent.folders.indexOf(this);
            if (i >= 0) this.parent.folders.splice(i, 1);
        }
        this.items.length = 0;
        this.folders.length = 0;
    }
}

// ---------------------------------------------------------------------------
// 假 harness：只实现场景实际用到的那部分 API
// （harness.world / params / addBody / addObject / addGroundPlane / watchJoint /
//   clearWatchedJoints / removeObjects / computeEnergy / refreshGUI / resetScene）
// ---------------------------------------------------------------------------
class FakeHarness {
    constructor() {
        this.params = {
            gravity: 9.81, numSubsteps: 20, numPosIters: 1, dt: DT, timeScale: 1.0,
            paused: false, useGyroscopic: true, showGrid: true, showContacts: false,
            showForces: false, forceScale: 0.02, showEnergy: false,
        };
        this.world = this._newWorld();
        this.gui = new FakeGUI();
        this.sceneGuiFolder = null;
        this.currentScene = null;
        this._sceneParamOverrides = new Map();
        this.buildInfo = null;
        this._handle = null;
        this._root = new THREE.Group();
    }

    _newWorld() {
        return new World({
            gravity: new THREE.Vector3(0, -this.params.gravity, 0),
            numSubsteps: this.params.numSubsteps,
            numPosIters: this.params.numPosIters,
            useGyroscopic: this.params.useGyroscopic,
        });
    }

    /** 镜像 harness._applyParamsToWorld：把 params 推给当前世界 */
    _applyParamsToWorld() {
        if (!this.world) return;
        this.world.numSubsteps = this.params.numSubsteps;
        this.world.numPosIters = this.params.numPosIters;
        this.world.useGyroscopic = this.params.useGyroscopic;
        this.world.gravity.set(0, -this.params.gravity, 0);
    }

    meshFor(shape, { color = 0x6ea8fe } = {}) {
        if (shape.type === ShapeType.PLANE) return null;
        return new THREE.Mesh(makeGeometry(shape), new THREE.MeshStandardMaterial({ color }));
    }

    addBody({ shape, color, ...rest }) {
        const mesh = this.meshFor(shape, { color });
        const body = new RigidBody({ shape, mesh, ...rest });
        this.world.addBody(body);
        if (mesh) { mesh.userData.body = body; this._root.add(mesh); }
        return body;
    }

    addObject(obj) { this._root.add(obj); return obj; }
    addGroundPlane() {
        return this.world.createBody({ shape: planeShape(new THREE.Vector3(0, 1, 0), 0), isStatic: true, name: 'ground' });
    }
    watchJoint(joint) { return joint; }
    clearWatchedJoints() { }
    removeObjects(objects) { for (const o of objects) this._root.remove(o); }
    refreshGUI() { }
    resetScene() { if (this.currentScene) this.setScene(this.currentScene); }

    /** 镜像 harness._overridesFor / _applySceneSim / setParams（优先级：显式覆盖 > 场景默认 > 全局默认） */
    _overridesFor(id) {
        const key = id || '__none__';
        let s = this._sceneParamOverrides.get(key);
        if (!s) { s = new Set(); this._sceneParamOverrides.set(key, s); }
        return s;
    }
    _applySceneSim(def) {
        if (!def.sim) return;
        const over = this._overridesFor(def.id);
        for (const [k, v] of Object.entries(def.sim)) {
            if (!over.has(k)) this.params[k] = v;
        }
    }
    setParams(patch) {
        Object.assign(this.params, patch);
        const over = this._overridesFor(this.currentScene && this.currentScene.id);
        for (const k of Object.keys(patch)) over.add(k);
        this._applyParamsToWorld();
    }

    setScene(def) {
        this.currentScene = def;
        // 先销毁旧场景的 GUI 文件夹（真 harness 在 _disposeScene 里做同样的事），
        // 否则每次重建都会往根 GUI 上再挂一份，旧场景对象永远回收不掉
        if (this.sceneGuiFolder) { this.sceneGuiFolder.destroy(); this.sceneGuiFolder = null; }
        // 与真 harness 一致：场景用 def.sim 声明的求解参数在**建 world 之前**合并进来，
        // 且不覆盖测试/读者显式指定的值（见 harness.setParams）。
        this._applySceneSim(def);
        this.world = this._newWorld();
        this._root = new THREE.Group();
        this._handle = def.create(this) || {};
        this.sceneGuiFolder = this.gui.addFolder(`场景：${def.name}`);
        if (this._handle.onGUI) this._handle.onGUI(this.sceneGuiFolder);
        this.buildInfo = def.buildInfo || null;
    }

    step(dt = DT) {
        if (this._handle.preStep) this._handle.preStep(dt);
        this.world.step(dt);
        if (this._handle.postStep) this._handle.postStep(dt);
    }

    /** 与真 harness 完全相同的能量定义（论文图 9 用总能量 E 判断守恒性） */
    computeEnergy() {
        let kinetic = 0.0, potential = 0.0;
        const qInv = new THREE.Quaternion();
        const wLocal = new THREE.Vector3();
        for (const b of this.world.bodies) {
            if (!b.isDynamic) continue;
            kinetic += 0.5 * b.mass * b.vel.lengthSq();
            qInv.set(-b.pose.q.x, -b.pose.q.y, -b.pose.q.z, b.pose.q.w);
            wLocal.copy(b.omega).applyQuaternion(qInv);
            const invI = b.invInertia0;
            if (invI.x > 0) kinetic += 0.5 * (wLocal.x * wLocal.x) / invI.x;
            if (invI.y > 0) kinetic += 0.5 * (wLocal.y * wLocal.y) / invI.y;
            if (invI.z > 0) kinetic += 0.5 * (wLocal.z * wLocal.z) / invI.z;
            potential += -b.mass * this.world.gravity.dot(b.pose.p);
        }
        return { kinetic, potential };
    }
}

// ---------------------------------------------------------------------------
// 数值健康检查与工具
// ---------------------------------------------------------------------------
const fin = (v) => Number.isFinite(v.x) && Number.isFinite(v.y) && Number.isFinite(v.z);
const deg = (r) => r * 180 / Math.PI;

function health(h) {
    for (const b of h.world.bodies) {
        const tag = b.name || b.shape.type;
        if (!fin(b.pose.p)) return { ok: false, why: `${tag} 位置非有限` };
        if (!fin(b.vel)) return { ok: false, why: `${tag} 速度非有限` };
        if (!fin(b.omega)) return { ok: false, why: `${tag} 角速度非有限` };
        const n = b.pose.q.length();
        if (!Number.isFinite(n) || Math.abs(n - 1) > 1e-3) return { ok: false, why: `${tag} 四元数未归一化 |q|=${n}` };
        if (b.pose.p.length() > 500) return { ok: false, why: `${tag} 飞出世界 |x|=${b.pose.p.length().toFixed(1)}` };
        if (b.isDynamic && b.vel.length() > 200) return { ok: false, why: `${tag} 速度爆炸 |v|=${b.vel.length().toFixed(1)}` };
    }
    if (!Number.isFinite(h.world.maxPenetration)) return { ok: false, why: 'maxPenetration 非有限' };
    for (const j of h.world.joints) {
        if (!fin(j.debugForce) || !fin(j.debugTorque)) return { ok: false, why: '关节力/力矩非有限' };
    }
    return { ok: true, why: '' };
}

/** 推进 seconds 秒；一旦出现非有限值就提前返回并报告是哪一帧 */
function run(h, seconds, probe = null) {
    const n = Math.round(seconds / DT);
    for (let i = 0; i < n; i++) {
        h.step(DT);
        if (probe) probe(h, i * DT);
        const s = health(h);
        if (!s.ok) return { ok: false, why: `第 ${i} 帧（t=${(i * DT).toFixed(2)}s）：${s.why}` };
    }
    return { ok: true, why: '' };
}

/** 动态体在水平面内的最大位移（堆叠失稳时表现为横向“剪切”漂移） */
function maxLateral(h) {
    let m = 0;
    for (const b of h.world.bodies) {
        if (!b.isDynamic) continue;
        m = Math.max(m, Math.abs(b.pose.p.x), Math.abs(b.pose.p.z));
    }
    return m;
}

function stats(h) {
    let maxV = 0, minY = Infinity, maxY = -Infinity, n = 0;
    for (const b of h.world.bodies) {
        if (!b.isDynamic) continue;
        n++;
        maxV = Math.max(maxV, b.vel.length());
        minY = Math.min(minY, b.pose.p.y);
        maxY = Math.max(maxY, b.pose.p.y);
    }
    return { maxV, minY, maxY, n };
}

function infoText(h) {
    if (!h.buildInfo) return { ok: true, text: '(无 buildInfo)' };
    try {
        const t = h.buildInfo(h);
        if (typeof t !== 'string') return { ok: false, text: `buildInfo 返回了 ${typeof t}` };
        if (/NaN|undefined|Infinity/.test(t)) return { ok: false, text: t.replace(/\n/g, ' | ') };
        return { ok: true, text: t.replace(/\n/g, ' | ') };
    } catch (e) {
        return { ok: false, text: `${e.constructor.name}: ${e.message}` };
    }
}

/** 把该场景注册的每个 GUI 控件都「操作」一遍，检查回调不抛异常 */
function exerciseGUI(h) {
    const ctrls = h.gui.controllersRecursive();
    let acted = 0;
    const errors = [];
    for (const c of ctrls) {
        try {
            if (c.kind === 'button') { c.click(); acted++; continue; }
            if (c.kind === 'boolean') {
                c.apply(!c.obj[c.prop]); c.apply(!c.obj[c.prop]); acted++;
                continue;
            }
            if (c.kind === 'options') {
                const old = c.obj[c.prop];
                for (const v of Object.values(c.options)) c.apply(v);
                c.apply(old); acted++;
                continue;
            }
            if (c.min === null) continue;      // 无 range 的裸数字，没法安全取值
            const old = c.apply(c.min);
            c.apply(c.max);
            c.apply(old);
            acted++;
        } catch (e) {
            errors.push(`${c.label}: ${e.constructor.name}: ${e.message}`);
        }
    }
    return { acted, total: ctrls.length, errors };
}

// ===========================================================================
// 逐个场景
// ===========================================================================
const byId = Object.fromEntries(scenes.map((s) => [s.id, s]));
let h = new FakeHarness();

console.log(`\n场景注册表：${scenes.length} 个 —— ${scenes.map((s) => s.name).join(' / ')}`);

console.log('\n[0] 注册表完整性');
check('场景数 = 8', scenes.length === 8, `实际 ${scenes.length}`);
for (const s of scenes) {
    check(`${s.id} 字段齐全`,
        typeof s.id === 'string' && typeof s.name === 'string' && typeof s.desc === 'string'
        && typeof s.create === 'function' && typeof s.buildInfo === 'function'
        && Array.isArray(s.camera?.position) && Array.isArray(s.camera?.target),
        s.name);
}
{
    const ids = scenes.map((s) => s.id);
    check('id 无重复', new Set(ids).size === ids.length, ids.join(','));
}

// ---------------------------------------------------------------------------
console.log('\n[1] sceneSpringBoxes —— 图 4 / 图 6：距离关节 Δ = α·F');
{
    const def = byId.springBoxes;
    h.setScene(def);
    check('图 4 建立 4 个距离关节', h.world.joints.length === 4, `实际 ${h.world.joints.length}`);
    check('默认打开力可视化（图 4 的卖点）', h.params.showForces === true);

    // 12 秒而不是 3 秒：弹簧的阻尼是速度层的（每子步只削掉 μ·h = 0.0005 的
    // 相对速度），α 小的弹簧 3 秒时还在振荡，取样点落在不同相位上会让
    // 「Δ = α·F」的关系看起来随缘。12 秒后全部收敛到平衡。
    const r = run(h, 12.0);
    check('图 4 跑 12 秒稳定', r.ok, r.why);

    const rows = h.world.joints.filter((j) => j.meta)
        .map((j) => ({ x: j.globalPoseA.p.x, m: j.meta.mass, alpha: j.meta.alpha, elong: j.getDistance() - j.meta.rest, F: j.lastForce }))
        .sort((a, b) => a.x - b.x);
    info('各关节 (x, α, 伸长Δ, 力F)', rows.map((q) => `(${q.x.toFixed(1)}, ${q.alpha}, ${q.elong.toFixed(4)}m, ${q.F.toFixed(3)}N)`).join('  '));

    let worst = 0;
    for (const q of rows) worst = Math.max(worst, Math.abs(q.alpha * q.F - q.elong) / Math.max(Math.abs(q.alpha * q.F), 1e-6));
    check('论文核心关系 Δ = α·F（相对误差 < 5%）', worst < 0.05, `最大偏差 ${(worst * 100).toFixed(2)}%`);

    // 平衡态里 F = m·g（盒子静止），于是 Δ = α·m·g —— 图 4 想说的
    // 「同一个 α 下质量越大伸长越长，但 Δ/F 恒等于 α」就是这个式子的两半。
    let worstEq = 0;
    for (const q of rows) {
        worstEq = Math.max(worstEq, Math.abs(q.elong - q.m * 9.81 * q.alpha) / (q.m * 9.81 * q.alpha));
    }
    check('平衡态 Δ = α·m·g（相对误差 < 5%）', worstEq < 0.05, `最大偏差 ${(worstEq * 100).toFixed(2)}%`);
    check('α 相同、质量翻 4 倍 → 伸长变长',
        rows[2].elong > rows[0].elong * 1.5,
        `1kg:${rows[0].elong.toFixed(4)}m  vs  4kg:${rows[2].elong.toFixed(4)}m`);
    check('同质量下 α 减半 → 伸长减半（α=0.010 vs 0.004）',
        Math.abs(rows[1].elong / rows[0].elong - 0.4) < 0.05,
        `比值 ${(rows[1].elong / rows[0].elong).toFixed(3)}（期望 0.4）`);

    // --- 图 6：1 g 吊 1 kg，α = 0.01 / 0.001 / 0 ---
    const S = (await import('../js/scenes/sceneSpringBoxes.js')).settings;
    S.mode = 'fig6';
    h.setScene(def);
    check('图 6 建立 6 个距离关节', h.world.joints.length === 6, `实际 ${h.world.joints.length}`);
    const r6 = run(h, 3.0);
    check('图 6 跑 3 秒稳定（1000:1 质量比 + α=0 硬约束）', r6.ok, r6.why);

    /** 按关节标签收集 |Δ|（label 形如「α=0.01 上」） */
    const collect = () => {
        const byLabel = {};
        for (const j of h.world.joints) {
            if (!j.meta) continue;
            byLabel[j.meta.label] = Math.abs(j.getDistance() - j.meta.rest);
        }
        return byLabel;
    };
    const cols = {};
    for (const [label, e] of Object.entries(collect())) {
        const a = label.match(/α=([\d.]+)/)[1];
        cols[a] = Math.max(cols[a] ?? 0, e);
    }
    info('最大伸长 |Δ| 按 α 分组', Object.entries(cols).map(([a, e]) => `α=${a}: ${e.toFixed(5)}m`).join('  '));
    check('α 越大伸长越大：0.001 < 0.01', cols['0.001'] < cols['0.01'],
        `α=0.001: ${cols['0.001'].toFixed(4)}m  <  α=0.01: ${cols['0.01'].toFixed(4)}m`);
    check('α=0 的硬约束远小于 α=0.01 的软约束（差一个量级以上）',
        cols['0'] * 10 < cols['0.01'],
        `α=0: ${cols['0'].toFixed(5)}m  ≪  α=0.01: ${cols['0.01'].toFixed(5)}m`);

    // α=0 的残留不是「柔度」，而是**光体那一侧的 Gauss-Seidel 残差**：
    // 1 g 挂着 1 kg，位置层每子步只解 1 次迭代时，天花板→1g 这条约束先被求解、
    // 随后又被下面那条把 1g 拽下去，于是留下约 m·g·h²·(质量比) 的残余；
    // 迭代数一上去就线性收敛。真正的 1000:1 那一跨（下关节）几乎精确为零。
    const zeroAt = (iters) => {
        h.setScene(def);
        h.setParams({ numPosIters: iters });
        run(h, 3.0);
        const by = collect();
        return { top: by['α=0 上'], bottom: by['α=0 下'] };
    };
    const z1 = zeroAt(1);
    const z10 = zeroAt(10);
    info('α=0 列的残留（上关节=天花板↔1g，下关节=1g↔1kg）',
        `1 迭代: 上 ${z1.top.toFixed(5)}m / 下 ${z1.bottom.toExponential(1)}m   `
        + `10 迭代: 上 ${z10.top.toFixed(5)}m / 下 ${z10.bottom.toExponential(1)}m`);
    check('1000:1 质量比那一跨几乎零伸长（下关节 < 0.1 mm）', z1.bottom < 1e-4,
        `${z1.bottom.toExponential(2)} m`);
    check('α=0 的残留随迭代数线性下降（是 GS 残差，不是柔度）',
        z10.top < z1.top * 0.3, `1 迭代 ${z1.top.toFixed(5)}m → 10 迭代 ${z10.top.toFixed(5)}m`);
    h.setParams({ numPosIters: 1 });
    S.mode = 'fig4';
}

// ---------------------------------------------------------------------------
console.log('\n[2] scenePendula —— 图 8 / 图 9：铰链链与子步化');
{
    const S = (await import('../js/scenes/scenePendula.js')).settings;
    const def = byId.pendula;
    const E = (hh) => { const { kinetic, potential } = hh.computeEnergy(); return kinetic + potential; };

    for (const [mode, nJoint, label] of [['single', 1, '单摆'], ['double', 2, '双摆'], ['triple', 3, '三摆'], ['closed', 4, '闭环四连杆']]) {
        S.mode = mode;
        h.setScene(def);
        check(`${label}：建立 ${nJoint} 个铰链约束`, h.world.joints.length === nJoint, `实际 ${h.world.joints.length}`);
        const r = run(h, 3.0);
        check(`${label} 跑 3 秒稳定`, r.ok, r.why);
    }

    // 闭环：四连杆的连杆长度应被约束保持住（过约束系统的求解质量）
    S.mode = 'closed';
    h.setScene(def);
    const barJoints = h.world.joints.filter((j) => j.bodyA && j.bodyB
        && j.bodyA.shape.type === ShapeType.BOX && j.bodyB.shape.type === ShapeType.BOX);
    const barLen0 = 1.6;
    let worstBar = 0;
    run(h, 3.0, () => {
        for (const j of barJoints) {
            j.updateGlobalPoses();
            const d = j.globalPoseA.p.distanceTo(j.globalPoseB.p);
            worstBar = Math.max(worstBar, Math.abs(d));
        }
    });
    check('闭环：两个杆-连杆铰链始终重合（误差 < 5 mm）', worstBar < 0.005, `最大偏差 ${(worstBar * 1000).toFixed(2)} mm`);

    // --- 图 9：同等总工作量下，子步 vs 迭代 ---
    S.mode = 'triple';
    const drift = {};
    for (const [n, it] of [[20, 1], [10, 2], [5, 4], [2, 10], [1, 20]]) {
        // 先按场景默认（40 子步）建好，再用 setParams 显式覆盖 —— 这是 harness 的
        // 参数优先级约定：显式覆盖 > 场景默认（def.sim）> 全局默认。
        h.setScene(def);
        h.setParams({ numSubsteps: n, numPosIters: it });
        const e0 = E(h);
        run(h, 3.0);
        const e1 = E(h);
        drift[`${n}x${it}`] = Math.abs(e1 - e0) / Math.abs(e0);
    }
    info('3 秒后总能量相对漂移', Object.entries(drift).map(([k, v]) => `${k}: ${(v * 100).toFixed(3)}%`).join('  '));
    check('20×1 比 1×20 守恒更好（论文图 9 的结论）',
        drift['20x1'] < drift['1x20'],
        `20×1: ${(drift['20x1'] * 100).toFixed(3)}%  <  1×20: ${(drift['1x20'] * 100).toFixed(3)}%`);
    // 比单点阈值更有意义的是**单调性**：总工作量固定时，子步越多越守恒。
    // 这正是论文图 9 的内容（也是「Small Steps」那篇的结论）。
    const order = [[20, 1], [10, 2], [5, 4], [2, 10], [1, 20]].map(([n, it]) => drift[`${n}x${it}`]);
    const mono = order.every((v, i) => i === 0 || v > order[i - 1]);
    check('子步越多越守恒（漂移随子步数单调下降）', mono,
        order.map((v) => `${(v * 100).toFixed(2)}%`).join(' → '));
    check('20×1 的能量漂移 < 3%', drift['20x1'] < 0.03, `${(drift['20x1'] * 100).toFixed(4)}%`);
    h.setParams({ numSubsteps: 40, numPosIters: 1 });   // 还原本场景的默认配置
}

// ---------------------------------------------------------------------------
console.log('\n[3] sceneJointTypes —— 图 7：铰链 / 棱柱 / 球窝');
{
    const def = byId.jointTypes;
    const S = (await import('../js/scenes/sceneJointTypes.js')).settings;
    h.setScene(def);
    check('建立 3 个关节', h.world.joints.length === 3, `实际 ${h.world.joints.length}`);

    const door = h.world.joints.find((j) => j.maxAngle !== undefined && j.targetAngle !== undefined);
    const slide = h.world.joints.find((j) => Array.isArray(j.limits));
    const ball = h.world.joints.find((j) => j.swingLimit && j.twistLimit);
    check('铰链 / 棱柱 / 球窝三类关节都在', !!door && !!slide && !!ball,
        `铰链=${!!door} 棱柱=${!!slide} 球窝=${!!ball}`);

    let maxDoor = 0, minSlide = 0, maxSlide = 0, maxSwing = 0;
    const r = run(h, 4.0, () => {
        if (door) maxDoor = Math.max(maxDoor, Math.abs(door.getCurrentAngle()));
        if (slide) { minSlide = Math.min(minSlide, slide.targetSlide); maxSlide = Math.max(maxSlide, slide.targetSlide); }
        if (ball) {
            ball.updateGlobalPoses();
            const a1 = new THREE.Vector3(1, 0, 0).applyQuaternion(ball.globalPoseA.q);
            const a2 = new THREE.Vector3(1, 0, 0).applyQuaternion(ball.globalPoseB.q);
            maxSwing = Math.max(maxSwing, Math.acos(THREE.MathUtils.clamp(a1.dot(a2), -1, 1)));
        }
    });
    check('跑 4 秒稳定（滑台自动往复 + 球窝激发 swing/twist）', r.ok, r.why);

    info('门最大张角 / 滑台行程 / 球窝最大 swing',
        `${deg(maxDoor).toFixed(1)}°  [${minSlide.toFixed(2)}, ${maxSlide.toFixed(2)}]m  ${deg(maxSwing).toFixed(1)}°`);
    check('门限制生效（≤ 85° + 余量）', maxDoor <= THREE.MathUtils.degToRad(S.doorLimitDeg + 5),
        `${deg(maxDoor).toFixed(1)}° vs 限制 ${S.doorLimitDeg}°`);
    check('滑台限制生效（±1.5 m + 余量）', minSlide >= -1.6 && maxSlide <= 1.6,
        `[${minSlide.toFixed(2)}, ${maxSlide.toFixed(2)}]`);
    check('球窝 swing 限制生效（≤ 50° + 余量）', maxSwing <= THREE.MathUtils.degToRad(S.swingLimitDeg + 6),
        `${deg(maxSwing).toFixed(1)}° vs 限制 ${S.swingLimitDeg}°`);

    // 把硬限制调软，仍应稳定（软限制的极端参数）
    if (door) door.limitCompliance = 0.02;
    if (ball) { ball.swingCompliance = 0.02; ball.twistCompliance = 0.02; }
    const r2 = run(h, 2.0);
    check('柔度推到 0.02（很软的限制）仍稳定', r2.ok, r2.why);
}

// ---------------------------------------------------------------------------
console.log('\n[4] sceneRobot —— 图 17：机械臂 IK');
{
    const def = byId.robot;
    const S = (await import('../js/scenes/sceneRobot.js')).settings;
    h.setScene(def);

    const ik = h.world.joints.find((j) => j.meta?.role === 'ik');
    check('IK 约束存在（距离关节 restLength = 0）', !!ik && ik.restLength < 1e-9,
        ik ? `restLength=${ik.restLength}` : '未找到');
    check('至少 3 个驱动铰链', h.world.joints.filter((j) => j.maxAngle !== undefined).length >= 3);

    let worstErr = 0;
    const r = run(h, 4.0, () => {
        if (ik) { ik.updateGlobalPoses(); worstErr = Math.max(worstErr, ik.getDistance()); }
    });
    check('目标盒走 Lissajous 轨迹时跑 4 秒稳定', r.ok, r.why);
    info('目标运动时的最大跟随误差', `${worstErr.toFixed(4)} m`);
    check('动态跟踪误差 < 0.25 m', worstErr < 0.25, `${worstErr.toFixed(4)} m`);

    // 静态目标：应能收敛得很紧
    S.auto = false; S.tx = 1.5; S.ty = 1.6; S.tz = 0.0;
    h.setScene(def);
    const ik2 = h.world.joints.find((j) => j.meta?.role === 'ik');
    run(h, 3.0);
    const errStatic = ik2.getDistance();
    info('静态目标 (1.5, 1.6, 0) 的收敛误差', `${errStatic.toFixed(5)} m`);
    check('静态目标收敛（< 5 cm）', errStatic < 0.05, `${errStatic.toFixed(5)} m`);
    S.auto = true;

    // 不可达目标：论文图 17 说应当「优雅退化」而不是炸掉
    S.auto = false; S.tx = 3.6; S.ty = 3.6; S.tz = 0.0;
    h.setScene(def);
    const r3 = run(h, 3.0);
    check('不可达目标不发散（优雅退化）', r3.ok, r3.why);
    const ik3 = h.world.joints.find((j) => j.meta?.role === 'ik');
    info('不可达目标 (3.6, 3.6, 0) 的残差', `${ik3.getDistance().toFixed(3)} m（应停在最接近位姿）`);
    S.auto = true;
}

// ---------------------------------------------------------------------------
console.log('\n[5] sceneRope —— 图 18：扭转的绳子（球窝 swing/twist）');
{
    const def = byId.rope;
    const S = (await import('../js/scenes/sceneRope.js')).settings;
    h.setScene(def);
    const nSeg = S.segments;
    check(`建立 ${nSeg + 1} 个球窝关节（${nSeg} 段 + 末端盒）`,
        h.world.joints.length === nSeg + 1, `实际 ${h.world.joints.length}`);

    // 与 sceneRope.js 的 measureTwist 同构：**带符号**的 twist 角沿链累加。
    // 只有大小的 acos 会把相邻关节的一正一负累加成两倍，读数虚高。
    const twistOf = (hh) => {
        let sum = 0;
        for (const j of hh.world.joints) {
            if (!j.twistLimit) continue;
            j.updateGlobalPoses();
            const a1 = new THREE.Vector3(1, 0, 0).applyQuaternion(j.globalPoseA.q);
            const a2 = new THREE.Vector3(1, 0, 0).applyQuaternion(j.globalPoseB.q);
            const n = new THREE.Vector3().addVectors(a1, a2);
            if (n.lengthSq() < 1e-12) continue;
            n.normalize();
            const b1 = new THREE.Vector3(0, 1, 0).applyQuaternion(j.globalPoseA.q);
            b1.addScaledVector(n, -n.dot(b1));
            const b2 = new THREE.Vector3(0, 1, 0).applyQuaternion(j.globalPoseB.q);
            b2.addScaledVector(n, -n.dot(b2));
            if (b1.lengthSq() < 1e-12 || b2.lengthSq() < 1e-12) continue;
            b1.normalize(); b2.normalize();
            sum += Math.atan2(new THREE.Vector3().crossVectors(b1, b2).dot(n), b1.dot(b2));
        }
        return sum;
    };

    const r = run(h, 3.0);
    check('自由摆动 3 秒稳定', r.ok, r.why);
    const relaxed = twistOf(h);
    info('无扭矩时的残余总扭转', `${deg(relaxed).toFixed(1)}°`);
    check('无扭矩时绳子基本不扭转（< 15°）', relaxed < THREE.MathUtils.degToRad(15), `${deg(relaxed).toFixed(1)}°`);

    // 加扭矩 → 扭转沿绳传播
    S.torque = 2.5;
    let peak = 0;
    const r2 = run(h, 2.5, () => { peak = Math.max(peak, twistOf(h)); });
    check('施加末端扭矩后仍稳定', r2.ok, r2.why);
    info('加扭矩 2.5 N·m 后的峰值总扭转', `${deg(peak).toFixed(1)}°`);
    check('扭矩确实把扭转注入绳子（> 60°）', peak > THREE.MathUtils.degToRad(60), `${deg(peak).toFixed(1)}°`);

    // 撤销扭矩 → 扭转应当回落。
    // 注意这是个**扭转振子**：撤掉扭矩后绳子会绕零来回摆（软 twist 限制储能、
    // 阻尼只有 0.02），所以判据要看包络衰减，而不是「某一时刻的值比峰值小」——
    // 那样一个负号就能蒙混过关。
    S.torque = 0.0;
    const envelope = [];
    for (let i = 0; i < 6; i++) {
        let localMax = 0;
        run(h, 1.0, () => { localMax = Math.max(localMax, Math.abs(twistOf(h))); });
        envelope.push(localMax);
    }
    info('撤销扭矩后每秒的 |总扭转| 包络', envelope.map((v) => `${deg(v).toFixed(0)}°`).join(' → '));
    check('撤销扭矩后扭转包络衰减（振幅逐秒减小）',
        envelope[5] < envelope[0] * 0.75 && envelope[5] < peak,
        `${deg(envelope[0]).toFixed(0)}° → ${deg(envelope[5]).toFixed(0)}°（峰值 ${deg(peak).toFixed(0)}°）`);
}

// ---------------------------------------------------------------------------
console.log('\n[6] sceneMarbles —— 图 12（冲量传递）与图 13（初始穿透）');
{
    const def = byId.marbles;
    const S = (await import('../js/scenes/sceneMarbles.js')).settings;

    // --- 图 12 ---
    S.mode = 'impulse';
    h.setScene(def);
    const nMarble = h.world.bodies.filter((b) => b.isDynamic).length;
    check('图 12：4 颗弹珠 + 2 根轨道线', nMarble === 4 && h.world.bodies.length === 6,
        `${nMarble} 动态 / ${h.world.bodies.length} 总数`);
    const startX = h.world.bodies.filter((b) => b.isDynamic).map((b) => b.pose.p.x).sort((a, b) => a - b)[0];
    let leftMostX = startX, maxLeftSpeed = 0;
    const r = run(h, 2.5, (hh) => {
        const left = hh.world.bodies.filter((b) => b.isDynamic).sort((a, b) => a.pose.p.x - b.pose.p.x)[0];
        leftMostX = Math.min(leftMostX, left.pose.p.x);
        maxLeftSpeed = Math.max(maxLeftSpeed, left.vel.length());
    });
    check('图 12 跑 2.5 秒稳定', r.ok, r.why);
    info('最左弹珠位移 / 峰值速度', `${(leftMostX - startX).toFixed(3)} m / ${maxLeftSpeed.toFixed(3)} m/s`);
    check('冲量传到最左弹珠（它被推动）', leftMostX - startX < -0.2,
        `Δx=${(leftMostX - startX).toFixed(3)} m`);

    // --- 图 13：同一个场景，只切换 Eq. 35 ---
    // 判据必须取**起始窗口**里的速度：弹珠是嵌在 15° 斜轨道上的，
    // 之后会自己滚下去（1.5 秒重力就能给到 ~3 m/s），整段取最大值量到的是滚落速度，
    // 和「初始穿透有没有把人弹飞」根本不是一回事。
    // 0.1 秒内重力最多贡献 g·sin15°·t ≈ 0.25 m/s，与 27 m/s 量级的弹射完全不在一个量级。
    S.mode = 'penetration';
    const KICK_WINDOW = 0.10;
    const measure = (useEq35) => {
        S.useEq35 = useEq35;
        h.setScene(def);
        const y0 = Math.max(...h.world.bodies.filter((b) => b.isDynamic).map((b) => b.pose.p.y));
        let kickV = 0, maxY = y0;
        const rr = run(h, 1.5, (hh, elapsed) => {
            for (const b of hh.world.bodies) {
                if (!b.isDynamic) continue;
                if (elapsed <= KICK_WINDOW) kickV = Math.max(kickV, b.vel.length());
                maxY = Math.max(maxY, b.pose.p.y);
            }
        });
        return { ok: rr.ok, why: rr.why, kickV, rise: maxY - y0, y0 };
    };
    const on = measure(true);
    const off = measure(false);
    const gravityFloor = 9.81 * Math.sin(THREE.MathUtils.degToRad(15)) * KICK_WINDOW;
    info(`Eq.35 开：前 ${KICK_WINDOW}s 峰值速度 / 全程抬升`, `${on.kickV.toFixed(2)} m/s / ${on.rise.toFixed(3)} m`);
    info(`Eq.35 关：前 ${KICK_WINDOW}s 峰值速度 / 全程抬升`, `${off.kickV.toFixed(2)} m/s / ${off.rise.toFixed(3)} m`);
    info('参照：重力在同样窗口内最多贡献', `${gravityFloor.toFixed(2)} m/s`);
    check('两种配置都稳定（不发散）', on.ok && off.ok, `${on.why} ${off.why}`);
    check('关闭 Eq.35 → 初始穿透产生“巨大分离速度”（论文图 13 上图）',
        off.kickV > 5.0 * on.kickV && off.kickV > 5.0,
        `关:${off.kickV.toFixed(2)} m/s  ≫  开:${on.kickV.toFixed(2)} m/s`);
    check('开启 Eq.35 → 被温和地推上轨道（论文图 13 下图）',
        on.kickV < 5.0 * gravityFloor && on.rise < 0.3,
        `${on.kickV.toFixed(2)} m/s（重力基线 ${gravityFloor.toFixed(2)} m/s），抬升 ${on.rise.toFixed(3)} m`);
    S.useEq35 = true;
}

// ---------------------------------------------------------------------------
console.log('\n[7] sceneCoin —— 图 15：硬币的高频运动');
{
    const def = byId.coin;
    const S = (await import('../js/scenes/sceneCoin.js')).settings;

    S.mode = 'spin';
    h.setScene(def);
    const coins = h.world.bodies.filter((b) => b.isDynamic);
    check(`建立 ${S.count} 枚硬币`, coins.length === S.count, `实际 ${coins.length}`);

    const tiltOf = (b) => {
        // 硬币轴 = 局部 Y 轴；与竖直方向的夹角即倾角
        const axis = new THREE.Vector3(0, 1, 0).applyQuaternion(b.pose.q);
        return Math.acos(THREE.MathUtils.clamp(Math.abs(axis.y), -1, 1));
    };
    const tilt0 = coins.map(tiltOf);
    let minY = Infinity;
    const r = run(h, 4.0, (hh) => {
        for (const b of hh.world.bodies) if (b.isDynamic) minY = Math.min(minY, b.pose.p.y);
    });
    check('高速自转 4 秒稳定（接触法线逐子步重算）', r.ok, r.why);
    const tilt1 = coins.map(tiltOf);
    const fmt = (rad) => `${deg(rad).toFixed(1)}°`;
    info('倾角 (初始 → 4 秒后)', tilt0.map((t, i) => `${fmt(t)}→${fmt(tilt1[i])}`).join('  '));
    check('硬币不穿透地面', minY > -0.01, `最低点 y=${minY.toFixed(4)} m`);
    check('自转使倾角增长（进动 → 倒下，图 15 的高频运动）',
        tilt1.some((t, i) => t > tilt0[i] + THREE.MathUtils.degToRad(3)),
        `最大变化 ${deg(Math.max(...tilt1.map((t, i) => t - tilt0[i]))).toFixed(1)}°`);

    // 关掉陀螺力矩 → 应当立刻失去进动（论文的对比实验）
    // 走 setParams：直接写 params 不会推给 world，对比实验会变成空操作
    h.setParams({ useGyroscopic: false });
    const rNo = run(h, 2.0);
    check('关闭陀螺力矩后仍稳定（对比实验）', rNo.ok, rNo.why);
    info('关闭陀螺力矩 2 秒后的倾角', coins.map((b) => fmt(tiltOf(b))).join('  '));
    h.setParams({ useGyroscopic: true });

    // drop 模式
    S.mode = 'drop';
    h.setScene(def);
    const rDrop = run(h, 4.0);
    check('drop 模式跑 4 秒稳定', rDrop.ok, rDrop.why);
    const rest = h.world.bodies.filter((b) => b.isDynamic).map((b) => b.pose.p.y);
    info('drop 模式落地后的质心高度', rest.map((y) => y.toFixed(4)).join('  '));
    check('平放的硬币停在半高 y ≈ 0.011 m', rest.every((y) => Math.abs(y - 0.011) < 0.02),
        rest.map((y) => y.toFixed(4)).join(','));
    S.mode = 'spin';
}

// ---------------------------------------------------------------------------
console.log('\n[8] sceneBoxStack —— 表 1：堆叠基准');
{
    const def = byId.boxStack;
    const S = (await import('../js/scenes/sceneBoxStack.js')).settings;

    S.count = 7; S.jitter = 0.012;
    h.setScene(def);
    check('7 个盒子 + 地面', h.world.bodies.length === 8, `实际 ${h.world.bodies.length}`);

    // 实测：7 盒在 20 子步 × 3 迭代下 |x| 漂移 0.03 m、后半程最大速度 0.25 m/s。
    // 论文的 20 子步 × 1 迭代在本实现下会横向“剪切”倒塌（|x| 漂到 0.49 m），
    // 偏差原因与实测数据见 README「盒子堆叠与论文 Table 1 的偏差」。
    const r = run(h, 5.0);
    check('7 盒堆叠跑 5 秒稳定', r.ok, r.why);
    let st = stats(h);
    const lateral = maxLateral(h);
    info('末态最大速度 / 横向漂移 / 最低点 / 最高点',
        `${st.maxV.toFixed(4)} m/s  |x|max=${lateral.toFixed(4)} m  y∈[${st.minY.toFixed(3)}, ${st.maxY.toFixed(3)}]`);
    check('末态静止（最大速度 < 0.3 m/s）', st.maxV < 0.3, `${st.maxV.toFixed(4)} m/s`);
    check('盒子不穿地（最低点 > -5 mm）', st.minY > -0.005, `${st.minY.toFixed(5)} m`);
    check('堆叠没有塌（最高点仍 ≈ 3.25 m）', st.maxY > 3.1, `${st.maxY.toFixed(3)} m`);
    check('横向“剪切”漂移很小（< 0.1 m）', lateral < 0.1, `${lateral.toFixed(4)} m`);
    check('最大穿透很小（< 1 cm）', h.world.maxPenetration < 0.01, `${(h.world.maxPenetration * 1000).toFixed(3)} mm`);

    // 3 盒（Table 1 第一行）：论文原配置就能稳住
    S.count = 3;
    h.setScene(def);
    const r3 = run(h, 5.0);
    st = stats(h);
    check('3 盒堆叠跑 5 秒稳定且静止', r3.ok && st.maxV < 0.05, `${r3.why} maxV=${st.maxV.toFixed(4)} m/s`);

    // 8 盒 = GUI 滑杆上限，应当与 7 盒同样稳定
    S.count = 8;
    h.setScene(def);
    const r8 = run(h, 5.0);
    const st8 = stats(h);
    info('8 盒（滑杆上限）末态', `maxV=${st8.maxV.toFixed(4)} m/s  maxY=${st8.maxY.toFixed(3)} m`);
    check('8 盒（GUI 上限）也稳定', r8.ok && st8.maxV < 0.3 && st8.maxY > 3.6,
        `${r8.why} maxV=${st8.maxV.toFixed(4)} maxY=${st8.maxY.toFixed(3)}`);

    // 摩擦是这里的主要失稳源：把 μ 降下来，本实现的稳定高度明显提高。
    // （论文的 20 子步 × 1 迭代可以稳住 7 盒，本实现在 1 迭代下用低摩擦也只是勉强稳住，
    //   这正是 README 里记录偏差的依据之一。）
    S.count = 8; S.friction = 0.3;
    h.setScene(def);
    const rLow = run(h, 5.0);
    const stLow = stats(h);
    info('8 盒 + 低摩擦 μ=0.3 末态', `maxV=${stLow.maxV.toFixed(4)} m/s  |x|max=${maxLateral(h).toFixed(4)} m`);
    check('低摩擦下 8 盒依旧稳定', rLow.ok && stLow.maxV < 0.3, `${rLow.why} maxV=${stLow.maxV.toFixed(4)}`);

    S.count = 7; S.friction = 0.6; S.jitter = 0.012;
}

// ---------------------------------------------------------------------------
console.log('\n[9] 全部场景：GUI 回调 + 实时读数');
for (const def of scenes) {
    // 不预设求解参数：让每个场景按自己 def.sim 里声明的配置跑，这也顺带验证了
    // 「场景默认值」这条优先级确实生效（曾经它被静默丢弃，pendula 一直按 20 子步跑）
    h.setParams({ useGyroscopic: true });
    h.setScene(def);

    const { acted, total, errors } = exerciseGUI(h);
    check(`${def.id}：${acted}/${total} 个 GUI 控件操作无异常`,
        errors.length === 0, errors.slice(0, 3).join(' | '));

    const r = run(h, 1.0);
    check(`${def.id}：GUI 交互后仍稳定`, r.ok, r.why);

    const bi = infoText(h);
    check(`${def.id}：buildInfo 输出正常`, bi.ok, bi.ok ? '' : bi.text);
}
console.log('\n  实时读数示例（各场景 buildInfo 的首行）：');
for (const def of scenes) {
    h.setScene(def);
    run(h, 1.0);
    const t = h.buildInfo ? h.buildInfo(h).split('\n')[0].replace(/<[^>]+>/g, '') : '(无)';
    console.log(`    ${def.id.padEnd(12)} ${t}`);
}

console.log(`\n===== 结果：${pass} 通过 / ${fail} 失败 =====\n`);
process.exit(fail > 0 ? 1 : 0);

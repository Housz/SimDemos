// ============================================================================
// harness.js —— 演示框架（渲染、相机、GUI、调试可视化、鼠标抓取）
//
// 本文件**不含任何物理算法**：物理全部在 js/core/ 下。这里只负责
//   1. three.js 渲染环境（renderer / 相机 / 灯光 / 网格地面 / OrbitControls）
//   2. 全局参数 GUI（重力、子步数、位置迭代数、Δt、时间缩放、暂停、陀螺力矩）
//   3. 调试可视化：接触点、关节力/力矩箭头（论文 Eqs. 11、18 的 f、τ）
//   4. 能量监视曲线（复现论文图 9 的能量守恒对比）
//   5. 鼠标抓取：用一个**柔度可调的约束**（论文 §3.3 的位置投影）拖拽刚体，
//      而不是直接改速度——这样抓取在每个子步都被求解，20 子步下依然稳定。
//
// 场景模块（js/scenes/*.js）只需实现：
//   { id, name, desc, create(harness) → { preStep(dt), postStep(dt), onGUI(gui), reset() } }
// ============================================================================

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GUI } from 'three/addons/libs/lil-gui.module.min.js';
import { World } from './core/world.js';
import { RigidBody } from './core/rigidBody.js';
import { ShapeType, plane as planeShape } from './core/shapes.js';
import { applyBodyPairCorrection } from './core/constraints.js';

// ---------------------------------------------------------------------------
// 形状 → three.js 几何体（物理形状与视觉形状必须一致，否则接触看起来会“错位”）
// ---------------------------------------------------------------------------
function makeGeometry(shape) {
    switch (shape.type) {
        case ShapeType.SPHERE:
            return new THREE.SphereGeometry(shape.radius, 32, 20);
        case ShapeType.BOX:
            return new THREE.BoxGeometry(
                shape.halfExtents.x * 2, shape.halfExtents.y * 2, shape.halfExtents.z * 2);
        case ShapeType.CAPSULE:
            return new THREE.CapsuleGeometry(shape.radius, shape.length, 8, 20);
        case ShapeType.CYLINDER:
            return new THREE.CylinderGeometry(shape.radius, shape.radius, shape.height, 32);
        case ShapeType.PLANE:
            return new THREE.PlaneGeometry(1, 1);
        default:
            return new THREE.BoxGeometry(1, 1, 1);
    }
}

// ---------------------------------------------------------------------------
// 鼠标抓取约束（一个柔度 α 可调的“单侧”位置约束）
//
// 论文 §3.3 的位置投影操作本来就支持 bodyB = null（相当于连到静态世界），
// 于是抓取就是一个 α > 0 的弹簧：k = 1/α。α 越小越硬，但过小会抖；
// 配合速度层的阻尼 μ_lin（论文 Eq. 32）即可得到稳定的拖拽手感。
// ---------------------------------------------------------------------------
export class MouseGrab {
    constructor(body, localAnchor, target, { stiffness = 600.0, damping = 20.0, maxForce = 500.0 } = {}) {
        this.body = body;
        this.localAnchor = localAnchor.clone();
        this.target = target.clone();
        this.compliance = 1.0 / stiffness;
        this.damping = damping;
        this.maxForce = maxForce;
        this.lambda = { grab: 0.0 };
        this.p = new THREE.Vector3();
    }

    resetLambda() { this.lambda.grab = 0.0; }

    updateAnchor() {
        this.p.copy(this.localAnchor).applyQuaternion(this.body.pose.q).add(this.body.pose.p);
    }

    solvePos(h) {
        this.updateAnchor();
        const corr = new THREE.Vector3().subVectors(this.target, this.p);
        // 限力：弹簧位移不超过 maxForce·α，避免抓住后瞬移
        const len = corr.length();
        const maxDisp = this.maxForce * this.compliance;
        if (len > maxDisp && len > 1e-12) corr.multiplyScalar(maxDisp / len);
        applyBodyPairCorrection(this.body, null, corr, this.compliance, h, this.p, null, this.lambda, 'grab');
    }

    solveVel(h) {
        this.updateAnchor();
        const v = this.body.getVelocityAt(this.p);
        v.multiplyScalar(-Math.min(1.0, this.damping * h));
        applyBodyPairCorrection(this.body, null, v, 0.0, h, this.p, null, null, null, true);
    }
}

// ---------------------------------------------------------------------------
// 能量监视曲线（论文图 9：不同子步数下的能量守恒对比）
// ---------------------------------------------------------------------------
class EnergyMonitor {
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.capacity = 240;
        this.series = { total: [], kinetic: [], potential: [] };
        this._resize();
    }

    _resize() {
        const dpr = Math.min(window.devicePixelRatio || 1, 2);
        const w = this.canvas.clientWidth || 240;
        const h = this.canvas.clientHeight || 90;
        this.canvas.width = Math.round(w * dpr);
        this.canvas.height = Math.round(h * dpr);
        this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        this._w = w;
        this._h = h;
    }

    push(kinetic, potential) {
        const total = kinetic + potential;
        const s = this.series;
        s.kinetic.push(kinetic);
        s.potential.push(potential);
        s.total.push(total);
        if (s.total.length > this.capacity) {
            s.kinetic.shift(); s.potential.shift(); s.total.shift();
        }
    }

    clear() {
        this.series.kinetic.length = 0;
        this.series.potential.length = 0;
        this.series.total.length = 0;
    }

    draw() {
        const ctx = this.ctx;
        const w = this._w, h = this._h;
        ctx.clearRect(0, 0, w, h);

        const all = this.series.total.concat(this.series.kinetic, this.series.potential);
        if (all.length < 2) return;

        let lo = Infinity, hi = -Infinity;
        for (const v of all) { if (v < lo) lo = v; if (v > hi) hi = v; }
        if (hi - lo < 1e-6) { hi = lo + 1.0; }
        const pad = (hi - lo) * 0.08;
        lo -= pad; hi += pad;

        // 背景与零线
        ctx.fillStyle = 'rgba(18,20,26,0.72)';
        ctx.fillRect(0, 0, w, h);
        if (lo < 0 && hi > 0) {
            const y0 = h - ((0 - lo) / (hi - lo)) * h;
            ctx.strokeStyle = 'rgba(255,255,255,0.18)';
            ctx.beginPath(); ctx.moveTo(0, y0); ctx.lineTo(w, y0); ctx.stroke();
        }

        const n = this.series.total.length;
        const line = (arr, color) => {
            ctx.strokeStyle = color;
            ctx.lineWidth = 1.5;
            ctx.beginPath();
            for (let i = 0; i < n; i++) {
                const x = (i / (this.capacity - 1)) * w;
                const y = h - ((arr[i] - lo) / (hi - lo)) * h;
                if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
            }
            ctx.stroke();
        };
        line(this.series.potential, '#a78bfa'); // 势能
        line(this.series.kinetic, '#22d3ee');   // 动能
        line(this.series.total, '#fb923c');     // 总能量

        ctx.font = '10px ui-monospace, monospace';
        ctx.fillStyle = '#94a3b8';
        ctx.fillText(`${hi.toFixed(2)} J`, 4, 11);
        ctx.fillText(`${lo.toFixed(2)} J`, 4, h - 3);
    }
}

// ---------------------------------------------------------------------------
// 演示框架主体
// ---------------------------------------------------------------------------
export class Harness {
    /**
     * @param {HTMLElement} container 渲染区容器（canvas 会填满它）
     */
    constructor(container) {
        this.container = container;

        // --- three.js 基础设施 ---
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0f1116);
        this.scene.fog = new THREE.Fog(0x0f1116, 30, 90);

        this.camera = new THREE.PerspectiveCamera(50, 1, 0.05, 500);
        this.camera.position.set(6, 5, 9);

        this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        container.appendChild(this.renderer.domElement);

        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.08;
        this.controls.target.set(0, 1.5, 0);

        this._setupLights();
        this._setupGround();

        // 演示对象容器：切场景时整体清空
        this._sceneRoot = new THREE.Group();
        this.scene.add(this._sceneRoot);

        // 可视化辅助
        this._contactRoot = new THREE.Group();
        this._contactRoot.visible = false;
        this.scene.add(this._contactRoot);
        this._forceRoot = new THREE.Group();
        this._forceRoot.visible = false;
        this.scene.add(this._forceRoot);
        this._arrowPool = [];
        this._contactPool = [];
        this._watchedJoints = [];

        // 物理世界：每切换一次场景重建一个
        this.world = new World();

        // --- 全局参数 ---
        this.params = {
            gravity: 9.81,
            numSubsteps: 20,      // 论文 Table 1：默认 20 子步
            numPosIters: 1,       // 论文 Table 1：默认 1 次位置迭代
            dt: 1.0 / 60.0,
            timeScale: 1.0,
            paused: false,
            useGyroscopic: true,
            showGrid: true,
            showContacts: false,
            showForces: false,
            forceScale: 0.02,
            showEnergy: false,
        };
        // 每个场景各记一份「显式覆盖」的参数键（见 setParams）
        this._sceneParamOverrides = new Map();
        this._applyParamsToWorld();

        this._simAccum = 0.0;
        this._simTime = 0.0;
        this._stepsLastFrame = 0;
        this._fps = 60;
        this._energyAccum = 0.0;

        this._buildGUI();
        this._bindPointer();
        this._bindResize();

        this.currentScene = null;
        this._sceneHandle = null;

        this._lastTime = performance.now();
        this._animate = this._animate.bind(this);
        requestAnimationFrame(this._animate);
    }

    // -----------------------------------------------------------------------
    // 初始化
    // -----------------------------------------------------------------------
    _setupLights() {
        this.scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x2a2f3a, 1.1));
        const key = new THREE.DirectionalLight(0xffffff, 2.0);
        key.position.set(8, 14, 8);
        key.castShadow = true;
        key.shadow.mapSize.set(2048, 2048);
        const d = 16;
        key.shadow.camera.left = -d; key.shadow.camera.right = d;
        key.shadow.camera.top = d; key.shadow.camera.bottom = -d;
        key.shadow.camera.far = 60;
        key.shadow.bias = -0.0008;
        this.scene.add(key);
        const fill = new THREE.DirectionalLight(0x93b4ff, 0.5);
        fill.position.set(-9, 6, -7);
        this.scene.add(fill);
    }

    /** 视觉地面：与物理平面（y = 0）严格重合，仅作参考，不参与物理 */
    _setupGround() {
        this.ground = new THREE.Mesh(
            new THREE.PlaneGeometry(120, 120),
            new THREE.ShadowMaterial({ opacity: 0.32 }));
        this.ground.rotation.x = -Math.PI / 2;
        this.ground.receiveShadow = true;
        this.scene.add(this.ground);

        this.grid = new THREE.GridHelper(60, 60, 0x3b4252, 0x232833);
        this.grid.material.transparent = true;
        this.grid.material.opacity = 0.75;
        this.scene.add(this.grid);

        // 坐标轴（红 X / 绿 Y / 蓝 Z），便于对照论文的关节局部轴约定
        const axes = new THREE.AxesHelper(1.2);
        axes.position.set(0, 0.002, 0);
        axes.material.depthWrite = false;
        this.scene.add(axes);
    }

    /**
     * 把全局参数推给当前世界。
     *
     * `setScene()` 在建 World **之前**就会调用它，所以世界一开始拿到的就是正确配置。
     */
    _applyParamsToWorld() {
        if (!this.world) return;
        this.world.numSubsteps = this.params.numSubsteps;
        this.world.numPosIters = this.params.numPosIters;
        this.world.useGyroscopic = this.params.useGyroscopic;
        this.world.gravity.set(0, -this.params.gravity, 0);
    }

    /** 取某个场景的「显式覆盖」键集合（无场景时用占位 id，不会被任何场景读到） */
    _overridesFor(id) {
        const key = id || '__none__';
        let s = this._sceneParamOverrides.get(key);
        if (!s) { s = new Set(); this._sceneParamOverrides.set(key, s); }
        return s;
    }

    /**
     * 把场景声明的默认求解参数（`def.sim`）合并进 `params`。
     *
     * 参数的优先级是：**显式覆盖 > 场景默认 > 全局默认**。
     * 被 `setParams()` 点名过的键算「显式覆盖」，这里跳过——否则读者在 GUI 里把
     * 子步数调大、再按一下「↻ 重置场景」，值就会被场景默认值打回去。
     */
    _applySceneSim(def) {
        if (!def.sim) return;
        const over = this._overridesFor(def.id);
        for (const [k, v] of Object.entries(def.sim)) {
            if (!over.has(k)) this.params[k] = v;
        }
    }

    /**
     * 显式覆盖参数：GUI 滑杆、场景里的预设按钮、测试都走这里。
     *
     * 这些键会被记进**当前场景**的覆盖集合，之后 `setScene()` / `resetScene()`
     * 都不再用场景默认值覆盖它们（换到别的场景则不受影响，各场景各记一份）。
     */
    setParams(patch) {
        Object.assign(this.params, patch);
        const over = this._overridesFor(this.currentScene && this.currentScene.id);
        for (const k of Object.keys(patch)) over.add(k);
        this._applyParamsToWorld();
        this._syncSimGUI();
    }

    /** 参数改动之后，把 lil-gui 的滑杆显示刷新到新值 */
    _syncSimGUI() {
        if (!this._simCtrls) return;
        for (const c of Object.values(this._simCtrls)) {
            if (c && typeof c.updateDisplay === 'function') c.updateDisplay();
        }
    }

    _buildGUI() {
        const gui = new GUI({ title: '全局参数' });
        this.gui = gui;

        const fSim = gui.addFolder('仿真');
        // 保住这几个控件的句柄：参数被别处改写时要刷新滑杆显示（见 _syncSimGUI）。
        this._simCtrls = {};
        // 全部走 setParams()：lil-gui 已经把值写进 params 了，这里再调一次是为了
        // 把该键登记为「显式覆盖」，免得重置场景时被场景默认值打回去。
        fSim.add(this.params, 'gravity', 0.0, 20.0, 0.01).name('重力 g (m/s²)')
            .onChange((v) => this.setParams({ gravity: v }));
        this._simCtrls.numSubsteps = fSim.add(this.params, 'numSubsteps', 1, 60, 1).name('子步数 N')
            .onChange((v) => { this.setParams({ numSubsteps: v }); this.resetScene(); });
        this._simCtrls.numPosIters = fSim.add(this.params, 'numPosIters', 1, 20, 1).name('位置迭代数')
            .onChange((v) => { this.setParams({ numPosIters: v }); this.resetScene(); });
        fSim.add(this.params, 'dt', 1 / 240, 1 / 20, 1 / 240).name('Δt (s)');
        fSim.add(this.params, 'timeScale', 0.05, 1.0, 0.05).name('时间缩放');
        fSim.add(this.params, 'paused').name('暂停 (空格)');
        this._simCtrls.useGyroscopic = fSim.add(this.params, 'useGyroscopic').name('陀螺力矩')
            .onChange((v) => this.setParams({ useGyroscopic: v }));
        fSim.add({ reset: () => this.resetScene() }, 'reset').name('↻ 重置场景');

        const fView = gui.addFolder('显示');
        fView.add(this.params, 'showGrid').name('网格/地面')
            .onChange(() => this.applyViewParams());
        fView.add(this.params, 'showContacts').name('接触点')
            .onChange(() => this.applyViewParams());
        fView.add(this.params, 'showForces').name('关节力/力矩')
            .onChange(() => this.applyViewParams());
        fView.add(this.params, 'forceScale', 0.001, 0.2, 0.001).name('力箭头缩放');
        fView.add(this.params, 'showEnergy').name('能量曲线')
            .onChange(() => this.applyViewParams());

        // 场景专属参数的挂载点（切场景时清空重建）
        this.sceneGuiFolder = null;
    }

    // -----------------------------------------------------------------------
    // 场景对象辅助
    // -----------------------------------------------------------------------
    /** 为物理形状创建视觉网格 */
    meshFor(shape, { color = 0x6ea8fe, opacity = 1.0, wireframe = false, flat = false } = {}) {
        if (shape.type === ShapeType.PLANE) return null; // 平面用网格地面代表，不单独建网格
        const mat = new THREE.MeshStandardMaterial({
            color, metalness: 0.15, roughness: 0.55,
            transparent: opacity < 1.0, opacity,
            wireframe, flatShading: flat,
        });
        const mesh = new THREE.Mesh(makeGeometry(shape), mat);
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        return mesh;
    }

    /**
     * 创建刚体 + 网格并加入世界与场景。
     * @returns {RigidBody} 刚体（其 .mesh 为对应网格）
     */
    addBody({ shape, color, opacity, wireframe, flat, ...rest }) {
        const mesh = this.meshFor(shape, { color, opacity, wireframe, flat });
        const body = new RigidBody({ shape, mesh, ...rest });
        this.world.addBody(body);
        if (mesh) {
            mesh.userData.body = body; // 供鼠标拾取
            this._sceneRoot.add(mesh);
        }
        return body;
    }

    /** 加入静态地面（物理平面 normal·x = offset），并在视觉上摆好网格 */
    addGroundPlane(normal = new THREE.Vector3(0, 1, 0), offset = 0.0) {
        const body = new RigidBody({
            shape: planeShape(normal, offset),
            isStatic: true,
            name: 'ground',
        });
        this.world.addBody(body);
        // 视觉地面已由 _setupGround 提供；此处只调整它的高度以匹配物理平面
        if (Math.abs(normal.y - 1.0) < 1e-6) {
            this.ground.position.y = offset;
            this.grid.position.y = offset + 0.001;
        }
        return body;
    }

    /** 把任意 three.js 对象加入当前场景（切场景时自动清理） */
    addObject(obj) {
        this._sceneRoot.add(obj);
        return obj;
    }

    /** 注册需要做力/力矩箭头可视化的关节 */
    watchJoint(joint, { anchor = null, torque = true } = {}) {
        this._watchedJoints.push({ joint, anchor, torque });
        return joint;
    }

    /** 清空力/力矩可视化列表（场景在运行时重建物体时调用） */
    clearWatchedJoints() {
        this._watchedJoints.length = 0;
    }

    /**
     * 移除场景对象（含几何体/材质释放）。场景在运行时重建自身内容时调用，
     * 避免旧网格残留在场景里。
     * @param {THREE.Object3D[]} objects
     */
    removeObjects(objects) {
        for (const obj of objects) {
            this._sceneRoot.remove(obj);
            obj.traverse?.((o) => {
                if (o.geometry) o.geometry.dispose();
                if (o.material) {
                    if (Array.isArray(o.material)) o.material.forEach((m) => m.dispose());
                    else o.material.dispose();
                }
            });
        }
    }

    // -----------------------------------------------------------------------
    // 场景切换
    // -----------------------------------------------------------------------
    /**
     * @param {object} def 场景模块（见 js/scenes/registry.js）
     *
     * ⚠ 场景**不要**在自己 `create()` 里写 `harness.params` —— `create()` 是在
     * World 建好之后才调用的，那些赋值会被静默丢弃（早期版本的真实 bug：pendula
     * 声明了 40 子步却一直按 20 跑）。请在场景定义里用 `sim: {...}` 声明，
     * 它会在建 World 之前合并进来，并且不会盖掉读者显式指定的值。
     */
    setScene(def) {
        this._disposeScene();
        this.currentScene = def;

        // 先合并场景声明的求解参数，再建 World
        this._applySceneSim(def);

        this.world = new World({
            gravity: new THREE.Vector3(0, -this.params.gravity, 0),
            numSubsteps: this.params.numSubsteps,
            numPosIters: this.params.numPosIters,
            useGyroscopic: this.params.useGyroscopic,
        });

        this._simTime = 0.0;
        this._simAccum = 0.0;
        this._watchedJoints.length = 0;
        this._grab = null;
        if (this.energyMonitor) this.energyMonitor.clear();

        this._sceneHandle = def.create(this) || {};

        // 相机预设
        const cam = def.camera || { position: [6, 5, 9], target: [0, 1.5, 0] };
        this.camera.position.set(...cam.position);
        this.controls.target.set(...cam.target);
        this.controls.update();

        // 场景专属 GUI
        if (this.sceneGuiFolder) this.sceneGuiFolder.destroy();
        this.sceneGuiFolder = this.gui.addFolder(`场景：${def.name}`);
        if (this._sceneHandle.onGUI) this._sceneHandle.onGUI(this.sceneGuiFolder);

        // 场景可能改写了全局参数（如打开能量曲线/力箭头），同步一次 GUI 显示
        this.refreshGUI();
        this.updateInfo('');

        // 每个场景的实时读数：由 postStep 回调里的 buildInfo 决定刷新频率
        this._buildInfo = def.buildInfo || null;
    }

    /** 让 GUI 控件重新读取 params（场景在 create() 里改过参数时调用） */
    refreshGUI() {
        this.gui.controllersRecursive().forEach((c) => c.updateDisplay());
        this.applyViewParams();
    }

    /**
     * 把「显示」分类的参数真正作用到场景对象上。
     * 场景在 create() 里可能直接改写 params（例如默认打开力箭头），
     * 此时 lil-gui 的 onChange 不会触发，必须显式同步一次。
     */
    applyViewParams() {
        const p = this.params;
        this.grid.visible = p.showGrid;
        this.ground.visible = p.showGrid;
        this._contactRoot.visible = p.showContacts;
        this._forceRoot.visible = p.showForces;
        if (this.energyPanel) this.energyPanel.style.display = p.showEnergy ? 'block' : 'none';
    }

    resetScene() {
        if (this.currentScene) this.setScene(this.currentScene);
    }

    _disposeScene() {
        if (this._sceneHandle && this._sceneHandle.dispose) this._sceneHandle.dispose();
        // 清理场景对象与几何体（材质每体独立，一并释放）
        for (const obj of this._sceneRoot.children.slice()) {
            obj.traverse?.((o) => {
                if (o.geometry) o.geometry.dispose();
                if (o.material) {
                    if (Array.isArray(o.material)) o.material.forEach((m) => m.dispose());
                    else o.material.dispose();
                }
            });
        }
        this._sceneRoot.clear();
        this._sceneHandle = null;
        clearAll(this._contactRoot, this._contactPool);
        clearAll(this._forceRoot, this._arrowPool);
    }

    // -----------------------------------------------------------------------
    // 鼠标抓取
    // -----------------------------------------------------------------------
    _bindPointer() {
        this._raycaster = new THREE.Raycaster();
        this._pointer = new THREE.Vector2();
        this._grabPlane = new THREE.Plane();
        this._grabTarget = new THREE.Vector3();

        const dom = this.renderer.domElement;
        dom.addEventListener('pointerdown', (e) => {
            if (e.button !== 0) return;
            const hit = this._pick(e);
            if (!hit) return;
            const point = hit.point;
            this._grab = new MouseGrab(hit.body, hit.body.toLocal(point), point, {
                stiffness: 500.0, damping: 25.0, maxForce: 400.0,
            });
            // 拖拽平面：过抓取点、垂直于相机视线
            const nrm = new THREE.Vector3();
            this.camera.getWorldDirection(nrm);
            this._grabPlane.setFromNormalAndCoplanarPoint(nrm, point);
            this.world.joints.push(this._grab);
            this.controls.enabled = false;
            dom.setPointerCapture(e.pointerId);
        });

        const end = (e) => {
            if (!this._grab) return;
            const i = this.world.joints.indexOf(this._grab);
            if (i >= 0) this.world.joints.splice(i, 1);
            this._grab = null;
            this.controls.enabled = true;
            if (e && e.pointerId !== undefined && dom.hasPointerCapture?.(e.pointerId)) {
                dom.releasePointerCapture(e.pointerId);
            }
        };
        dom.addEventListener('pointerup', end);
        dom.addEventListener('pointercancel', end);

        dom.addEventListener('pointermove', (e) => {
            if (!this._grab) return;
            this._updatePointer(e);
            this._raycaster.setFromCamera(this._pointer, this.camera);
            if (this._raycaster.ray.intersectPlane(this._grabPlane, this._grabTarget)) {
                this._grab.target.copy(this._grabTarget);
            }
        });

        window.addEventListener('keydown', (e) => {
            if (e.code === 'Space') { this.params.paused = !this.params.paused; e.preventDefault(); }
            if (e.key === 'r' || e.key === 'R') this.resetScene();
        });
    }

    _updatePointer(e) {
        const r = this.renderer.domElement.getBoundingClientRect();
        this._pointer.x = ((e.clientX - r.left) / r.width) * 2 - 1;
        this._pointer.y = -((e.clientY - r.top) / r.height) * 2 + 1;
    }

    /** 射线拾取动态刚体；返回 {body, point} */
    _pick(e) {
        this._updatePointer(e);
        this._raycaster.setFromCamera(this._pointer, this.camera);
        const meshes = [];
        for (const b of this.world.bodies) {
            if (!b.mesh || !b.isDynamic) continue;
            b.mesh.userData.body = b;
            meshes.push(b.mesh);
        }
        const hits = this._raycaster.intersectObjects(meshes, false);
        if (hits.length === 0) return null;
        const h = hits[0];
        return { body: h.object.userData.body, point: h.point };
    }

    _bindResize() {
        const onResize = () => {
            const w = this.container.clientWidth;
            const h = this.container.clientHeight;
            if (w === 0 || h === 0) return;
            this.camera.aspect = w / h;
            this.camera.updateProjectionMatrix();
            this.renderer.setSize(w, h, false);
            this.renderer.domElement.style.width = '100%';
            this.renderer.domElement.style.height = '100%';
            if (this.energyMonitor) this.energyMonitor._resize();
        };
        window.addEventListener('resize', onResize);
        // 容器尺寸可能因侧栏折叠而变，用 ResizeObserver 兜住
        if (window.ResizeObserver) new ResizeObserver(onResize).observe(this.container);
        onResize();
    }

    // -----------------------------------------------------------------------
    // 主循环
    // -----------------------------------------------------------------------
    _animate() {
        requestAnimationFrame(this._animate);

        const now = performance.now();
        let elapsed = (now - this._lastTime) / 1000;
        this._lastTime = now;
        if (elapsed > 0.25) elapsed = 0.25;      // 切标签页回来不要补几千帧
        this._fps += ((1.0 / Math.max(elapsed, 1e-4)) - this._fps) * 0.1;

        if (!this.params.paused && this._sceneHandle) {
            this._simAccum += elapsed * this.params.timeScale;
            let steps = 0;
            while (this._simAccum >= this.params.dt && steps < 4) {
                this._simAccum -= this.params.dt;
                this._stepOnce(this.params.dt);
                steps++;
            }
            if (steps === 4) this._simAccum = 0.0; // 跟不上了就别累积
            this._stepsLastFrame = steps;
        }

        this._updateDebugViz();
        if (this._buildInfo) this.updateInfo(this._buildInfo(this));
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
        this._updateHud();
    }

    _stepOnce(dt) {
        if (this._sceneHandle.preStep) this._sceneHandle.preStep(dt);
        this.world.step(dt);
        if (this._sceneHandle.postStep) this._sceneHandle.postStep(dt);
        this._simTime += dt;

        // 能量采样（按仿真时间，与帧率无关）
        this._energyAccum += dt;
        if (this.energyMonitor && this._energyAccum >= 0.02) {
            this._energyAccum = 0.0;
            const { kinetic, potential } = this.computeEnergy();
            this.energyMonitor.push(kinetic, potential);
        }
    }

    /** 动能 / 势能分解（论文图 9 用总能量 E 判断守恒性） */
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

    // -----------------------------------------------------------------------
    // 调试可视化
    // -----------------------------------------------------------------------
    _updateDebugViz() {
        if (this.params.showContacts) this._drawContacts();
        if (this.params.showForces) this._drawForces();
        if (this.params.showEnergy && this.energyMonitor) this.energyMonitor.draw();
    }

    _drawContacts() {
        const contacts = this.world.contacts;
        let i = 0;
        for (; i < contacts.length; i++) {
            const c = contacts[i];
            let dot = this._contactPool[i];
            if (!dot) {
                dot = new THREE.Mesh(
                    new THREE.SphereGeometry(0.035, 8, 6),
                    new THREE.MeshBasicMaterial({ color: 0xff5555, depthTest: false }));
                dot.renderOrder = 10;
                this._contactPool[i] = dot;
                this._contactRoot.add(dot);
            }
            dot.visible = true;
            dot.position.copy(c.p1);
        }
        for (; i < this._contactPool.length; i++) this._contactPool[i].visible = false;
    }

    _drawForces() {
        let i = 0;
        const scale = this.params.forceScale;
        for (const w of this._watchedJoints) {
            const j = w.joint;
            // 力（论文 Eq. 11）：作用在关节锚点；力矩（Eq. 18）：作用在关节中心
            j.updateGlobalPoses?.();
            const anchors = [];
            if (j.debugForce && j.debugForce.lengthSq() > 1e-12) {
                anchors.push({ pos: j.globalPoseA.p, vec: j.debugForce, color: 0xffd166 });
            }
            if (w.torque && j.debugTorque && j.debugTorque.lengthSq() > 1e-12) {
                anchors.push({ pos: j.globalPoseA.p, vec: j.debugTorque, color: 0xef476f });
            }
            for (const a of anchors) {
                let arrow = this._arrowPool[i];
                if (!arrow) {
                    arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(), 1, 0xffffff, 0.18, 0.1);
                    arrow.line.material.depthTest = false;
                    arrow.cone.material.depthTest = false;
                    arrow.renderOrder = 11;
                    this._arrowPool[i] = arrow;
                    this._forceRoot.add(arrow);
                }
                const len = a.vec.length() * scale;
                if (len < 1e-3) { arrow.visible = false; i++; continue; }
                arrow.visible = true;
                arrow.position.copy(a.pos);
                arrow.setDirection(a.vec.clone().normalize());
                arrow.setLength(len, Math.min(0.22, len * 0.35), Math.min(0.12, len * 0.2));
                arrow.setColor(new THREE.Color(a.color));
                i++;
            }
        }
        for (; i < this._arrowPool.length; i++) this._arrowPool[i].visible = false;
    }

    // -----------------------------------------------------------------------
    // HUD
    // -----------------------------------------------------------------------
    /** 在 index.html 中查找 HUD 元素（可选） */
    attachHUD({ fps, stats, info, energy }) {
        this.hudFps = fps;
        this.hudStats = stats;
        this.hudInfo = info;
        this.energyPanel = energy;
        if (energy) {
            this.energyMonitor = new EnergyMonitor(energy);
            energy.style.display = this.params.showEnergy ? 'block' : 'none';
        }
    }

    updateInfo(html) {
        this._infoHTML = html;
    }

    _updateHud() {
        if (this.hudFps) this.hudFps.textContent = `${this._fps.toFixed(0)} FPS`;
        if (this.hudStats) {
            const w = this.world;
            this.hudStats.textContent =
                `t=${this._simTime.toFixed(2)}s  N=${w.numSubsteps}×${w.numPosIters}`
                + `  体=${w.bodies.length}  对=${w.pairsCount}  接触=${w.contacts.length}`
                + `  穿透max=${w.maxPenetration.toFixed(4)}m`;
        }
        if (this.hudInfo) this.hudInfo.innerHTML = this._infoHTML || '';
    }
}

/** 清空一个组下池化的对象 */
function clearAll(root, pool) {
    for (const o of pool) root.remove(o);
    pool.length = 0;
}

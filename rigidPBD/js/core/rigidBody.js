// ============================================================================
// rigidBody.js —— 刚体
//
// 论文 §3.2「Rigid Body Simulation Loop」与 Algorithm 2：
//
//   x_prev ← x;  v ← v + h·f_ext/m;  x ← x + h·v
//   q_prev ← q;  ω ← ω + h·I⁻¹(τ_ext − ω×(Iω));  q ← q·exp(½h[ω,0]);  q ← q/|q|
//   ...（位置求解）...
//   v ← (x − x_prev)/h
//   Δq ← q·q_prev⁻¹;  ω ← 2·axis(Δq)·atan2(|Δq.xyz|,|Δq.w|)/h
//
// 注：转动那两行论文写的是**一阶线性化**形式（q ← q + h·½[ω,0]q 与 ω ← 2[Δq.xyz]/h）。
// 这里改用精确的指数/对数映射：两者互为逆运算，和平动的 x += h·v / v = Δx/h 一样
// 不会凭空吃掉角速度；线性化那一对每子步会损失 |ω|²h²/8 的比例（高转速体上肉眼可见：
// 硬币自转 2 秒掉一半）。推导与实测见 math3d.js 中 applyRotationExact 的注释。
// **约束投影路径仍然用线性化形式**（见 247 行附近），那是 XPBD 推导的前提。
//
// 其中惯性张量 I 在体的 rest 系下为对角阵（论文 §3.3.2），
// 故用 Vector3 invInertia0 存储 I⁻¹ 的对角元素。
// ============================================================================

import * as THREE from 'three';
import { Pose, conj, applyRotationalCorrection, applyRotationExact, quatToRotationVectorExact } from './math3d.js';
import { shapeVolume, shapeInertia, shapeSupportLocal, ShapeType } from './shapes.js';

/** 刚体唯一编号的分配器（见 RigidBody.id） */
let _nextId = 1;

/** update() 里由 Δq 反推 ω 用的临时四元数（避免每子步分配） */
const TMP_DQ = new THREE.Quaternion();

export class RigidBody {
    /**
     * @param {object} opts
     * @param {object} opts.shape      碰撞形状（见 shapes.js）
     * @param {number} [opts.density]  密度（kg/m³），与 mass 二选一
     * @param {number} [opts.mass]     显式质量（kg），给出时忽略 density
     * @param {THREE.Vector3} [opts.position] 初始位置（世界系，质心）
     * @param {THREE.Quaternion} [opts.quaternion] 初始姿态
     * @param {boolean} [opts.isStatic] 静态体（无限质量）
     * @param {THREE.Object3D} [opts.mesh] 对应的可视化对象
     */
    constructor({
        shape,
        density = 1000.0,
        mass = null,
        position = null,
        quaternion = null,
        isStatic = false,
        mesh = null,
        name = '',
    }) {
        // 唯一编号：宽相用它给「被关节连在一起的刚体对」建索引，
        // 从而跳过相邻连杆之间的自碰撞（见 world.js 的 _excludedPairs）
        this.id = _nextId++;
        this.name = name;
        this.shape = shape;
        this.pose = new Pose(position, quaternion);
        this.prevPose = this.pose.clone();
        this.vel = new THREE.Vector3(0, 0, 0);
        this.omega = new THREE.Vector3(0, 0, 0);

        // 外部力/力矩累加器（每子步积分时清零）
        this.force = new THREE.Vector3(0, 0, 0);
        this.torque = new THREE.Vector3(0, 0, 0);

        // 材质（论文 §3.5：两侧系数不同时取平均）
        this.staticFriction = 0.6;
        this.dynamicFriction = 0.5;
        this.restitution = 0.0;

        // 子步速度快照：速度层恢复系数需要“速度更新前”的法向速度（论文 §3.6）
        this.prevVel = new THREE.Vector3(0, 0, 0);
        this.prevOmega = new THREE.Vector3(0, 0, 0);

        // 质量属性
        this.isStatic = isStatic;
        const volume = shapeVolume(shape);
        const m = isStatic ? 0.0 : (mass !== null ? mass : density * volume);
        this.mass = m;
        this.invMass = (isStatic || m <= 0) ? 0.0 : 1.0 / m;
        if (isStatic || m <= 0) {
            this.invInertia0 = new THREE.Vector3(0, 0, 0);
        } else {
            const I = shapeInertia(shape, m);
            this.invInertia0 = new THREE.Vector3(
                I.x > 0 ? 1.0 / I.x : 0.0,
                I.y > 0 ? 1.0 / I.y : 0.0,
                I.z > 0 ? 1.0 / I.z : 0.0
            );
        }

        // 可视化对象；物理量在 update() 中同步过去
        this.mesh = mesh;
        this.syncMesh();
    }

    get isDynamic() {
        return this.invMass > 0.0;
    }

    /** 体局部点 → 世界点 */
    toWorld(localPoint) {
        return this.pose.transform(localPoint.clone());
    }

    /** 世界点 → 体局部点 */
    toLocal(worldPoint) {
        return this.pose.invTransform(worldPoint.clone());
    }

    // -----------------------------------------------------------------------
    // 积分与速度导出（论文 Algorithm 2）
    // -----------------------------------------------------------------------

    /**
     * 显式积分一步。含陀螺力矩项 ω × (Iω) 的牛顿-欧拉方程
     * （论文 Algorithm 2 中的 ω ← ω + h·I⁻¹(τ_ext − ω×(Iω))）。
     *
     * 陀螺项在体的 rest 系下计算（I 在该系下对角，计算更简单），
     * 这与论文在世界系下按 I⁻¹ 求值在数学上等价。
     *
     * @param {number} h 子步长
     * @param {THREE.Vector3} gravity 重力加速度
     * @param {boolean} useGyroscopic 是否启用陀螺力矩项
     */
    integrate(h, gravity, useGyroscopic = true) {
        if (!this.isDynamic) {
            // 静态体仍然记录 prevPose，供接触点的“子步前位置”使用（论文 Eq. 27）
            this.prevPose.copy(this.pose);
            return;
        }

        this.prevPose.copy(this.pose);

        // --- 平动 ---
        this.vel.addScaledVector(gravity, h);
        if (this.force.lengthSq() > 0) this.vel.addScaledVector(this.force, h * this.invMass);
        this.pose.p.addScaledVector(this.vel, h);

        // --- 转动 ---
        const qInv = conj(this.pose.q);
        // 世界 → rest 系
        const wLoc = this.omega.clone().applyQuaternion(qInv);
        const invI0 = this.invInertia0;

        if (useGyroscopic) {
            // 陀螺力矩：ω ← ω + h·I⁻¹(−ω × (Iω))（论文 Algorithm 2 原文）。
            // I⁻¹ 是**整个括号**的系数，陀螺项也必须过一遍：
            // Euler 方程在主轴系下是 I·dω/dt = τ − ω×(Iω)，即 dω/dt = I⁻¹(τ − ω×(Iω))。
            // 漏掉 I⁻¹ 会把陀螺项缩掉 1/|I| 倍（本项目的圆柱约 10⁴ 倍），
            // 效果上等于关掉它——硬币场景里表现为「旋转完全不带稳定作用，0.25 秒就倒平」。
            const Iw = new THREE.Vector3(wLoc.x / invI0.x, wLoc.y / invI0.y, wLoc.z / invI0.z);
            const gyro = new THREE.Vector3().crossVectors(wLoc, Iw);
            wLoc.x -= h * gyro.x * invI0.x;
            wLoc.y -= h * gyro.y * invI0.y;
            wLoc.z -= h * gyro.z * invI0.z;
        }
        if (this.torque.lengthSq() > 0) {
            const tLoc = this.torque.clone().applyQuaternion(qInv);
            wLoc.x += h * tLoc.x * invI0.x;
            wLoc.y += h * tLoc.y * invI0.y;
            wLoc.z += h * tLoc.z * invI0.z;
        }

        this.omega.copy(wLoc.applyQuaternion(this.pose.q));

        // 论文 Algorithm 2 此处为 q ← q + h·½[ω,0]q（一阶）；这里用精确指数映射，
        // 与下面的精确对数反推配成互逆的一对，见文件头说明。
        applyRotationExact(this.pose, this.omega, h);
    }

    /**
     * 由位置/姿态的**变化量**导出速度（论文 Algorithm 2 第三段）。
     * 这正是 PBD 的核心思想：速度不是积分出来的，而是求解后由位置差分得到。
     */
    update(h, writeBack = true) {
        if (!this.isDynamic) {
            if (writeBack) this.syncMesh();
            return;
        }

        this.vel.subVectors(this.pose.p, this.prevPose.p).multiplyScalar(1.0 / h);

        // Δq = q · q_prev⁻¹，取其**精确**对数映射（轴 × 角）/h 作为 ω。
        // 论文写的是 2[Δq_xyz]/h，那是小角度近似：对高转速体它比真实转角小
        // sin(θ/2)/(θ/2) 倍，会让 ω 每个子步系统性衰减（见 math3d.js 的说明）。
        const dq = TMP_DQ.multiplyQuaternions(this.pose.q, conj(this.prevPose.q));
        this.omega.copy(quatToRotationVectorExact(dq)).multiplyScalar(1.0 / h);

        if (writeBack) this.syncMesh();
    }

    /** 把物理位姿同步到可视化对象 */
    syncMesh() {
        if (!this.mesh) return;
        this.mesh.position.copy(this.pose.p);
        this.mesh.quaternion.copy(this.pose.q);
    }

    // -----------------------------------------------------------------------
    // 约束求解用的冲量算子（论文 Eqs. 2-10、12-17）
    // -----------------------------------------------------------------------

    /**
     * 论文 Eqs. (2)(3)：广义逆质量 w = 1/m + (r×n)ᵀ I⁻¹ (r×n)
     * @param {THREE.Vector3} n 世界系单位方向
     * @param {THREE.Vector3|null} r 世界系力臂（锚点 − 质心）；null 表示纯角度约束
     */
    getGeneralizedInvMass(n, r = null) {
        if (!this.isDynamic) return 0.0;
        const tmp = new THREE.Vector3();
        if (r === null) {
            tmp.copy(n);
        } else {
            tmp.crossVectors(r, n);
        }
        tmp.applyQuaternion(conj(this.pose.q)); // 投影到 rest 系
        const invI = this.invInertia0;
        let w = tmp.x * tmp.x * invI.x + tmp.y * tmp.y * invI.y + tmp.z * tmp.z * invI.z;
        if (r !== null) w += this.invMass;
        return w;
    }

    /**
     * 在锚点处施加修正量（论文 Eqs. 6-9 / 16-17 / 34）。
     *
     * 位置层（velocityLevel = false）：
     *   x ← x + p/m
     *   q ← q + ½[I⁻¹(r×p), 0] q
     * 速度层（velocityLevel = true）：
     *   v ← v + p/m ;  ω ← ω + I⁻¹(r×p)
     *
     * @param {THREE.Vector3} corr 世界系修正向量 p
     * @param {THREE.Vector3|null} r 世界系力臂；null 表示纯角度修正（论文 Eq. 16/17）
     * @param {boolean} velocityLevel
     */
    applyCorrection(corr, r = null, velocityLevel = false) {
        if (!this.isDynamic) return;

        const dq = new THREE.Vector3();
        if (r === null) {
            dq.copy(corr);
        } else {
            if (velocityLevel) this.vel.addScaledVector(corr, this.invMass);
            else this.pose.p.addScaledVector(corr, this.invMass);
            dq.crossVectors(r, corr); // r × p
        }

        // 投影到 rest 系 → 乘 I⁻¹ → 变换回世界系
        dq.applyQuaternion(conj(this.pose.q));
        const invI = this.invInertia0;
        dq.set(dq.x * invI.x, dq.y * invI.y, dq.z * invI.z);
        dq.applyQuaternion(this.pose.q);

        if (velocityLevel) this.omega.add(dq);
        else applyRotationalCorrection(this.pose, dq);
    }

    /**
     * 世界系中某点的速度 v + ω × r（论文 Eq. 30）
     */
    getVelocityAt(worldPoint, out = new THREE.Vector3()) {
        // 注意叉乘顺序：点速度 = v + ω × r（r = 该点相对质心的力臂）。
        // 写成 r.cross(ω) 会得到 −ω × r，使接触点法向速度中**转动项的符号整体反号**，
        // 于是速度层的法向修正会把多点接触的旋转越推越大（堆叠场景直接发散）。
        out.subVectors(worldPoint, this.pose.p);
        out.crossVectors(this.omega, out);
        return out.add(this.vel);
    }

    /** 用**速度更新前**的快照求某点速度（论文 §3.6 恢复系数所需 v̄n） */
    getVelocityAtPrev(worldPoint, out = new THREE.Vector3()) {
        out.subVectors(worldPoint, this.pose.p);
        out.crossVectors(this.prevOmega, out);
        return out.add(this.prevVel);
    }

    /** 在由位置导出速度之前调用，保存速度快照 */
    snapshotVelocity() {
        this.prevVel.copy(this.vel);
        this.prevOmega.copy(this.omega);
    }

    /** 清空外力/力矩累加器（每个仿真帧开始时调用） */
    clearForces() {
        this.force.set(0, 0, 0);
        this.torque.set(0, 0, 0);
    }

    /** 把刚体复位到初始状态 */
    reset() {
        this.vel.set(0, 0, 0);
        this.omega.set(0, 0, 0);
        this.force.set(0, 0, 0);
        this.torque.set(0, 0, 0);
        this.syncMesh();
    }

    /** 世界系 AABB（用于 broad phase），结果写入 out（{min, max}） */
    computeAABB(out) {
        const qInv = conj(this.pose.q);
        const min = out.min, max = out.max;
        min.set(Infinity, Infinity, Infinity);
        max.set(-Infinity, -Infinity, -Infinity);
        if (this.shape.type === ShapeType.PLANE) {
            min.set(-Infinity, -Infinity, -Infinity);
            max.set(Infinity, Infinity, Infinity);
            return out;
        }
        // 对 6 个世界轴方向求支撑点，取投影极值（对任意凸形状精确）
        const dirs = AABB_DIRS;
        for (let i = 0; i < 6; i++) {
            const dLocal = dirs[i].clone().applyQuaternion(qInv);
            const pLocal = shapeSupportLocal(this.shape, dLocal);
            const pWorld = pLocal.applyQuaternion(this.pose.q).add(this.pose.p);
            const s = dirs[i];
            if (s.x > 0) max.x = Math.max(max.x, pWorld.x);
            else if (s.x < 0) min.x = Math.min(min.x, pWorld.x);
            if (s.y > 0) max.y = Math.max(max.y, pWorld.y);
            else if (s.y < 0) min.y = Math.min(min.y, pWorld.y);
            if (s.z > 0) max.z = Math.max(max.z, pWorld.z);
            else if (s.z < 0) min.z = Math.min(min.z, pWorld.z);
        }
        return out;
    }
}

const AABB_DIRS = [
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(-1, 0, 0),
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(0, -1, 0),
    new THREE.Vector3(0, 0, 1),
    new THREE.Vector3(0, 0, -1),
];

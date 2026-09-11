// ============================================================================
// rigidBody.js —— 刚体
//
// 论文 §3.2「Rigid Body Simulation Loop」与 Algorithm 2：
//
//   x_prev ← x;  v ← v + h·f_ext/m;  x ← x + h·v
//   q_prev ← q;  ω ← ω + h·I⁻¹(τ_ext − ω×(Iω));  q ← q + h·½[ω,0]q;  q ← q/|q|
//   ...（位置求解）...
//   v ← (x − x_prev)/h
//   Δq ← q·q_prev⁻¹;  ω ← 2[Δq.xyz]/h;  ω ← (Δq.w ≥ 0 ? ω : −ω)
//
// 其中惯性张量 I 在体的 rest 系下为对角阵（论文 §3.3.2），
// 故用 Vector3 invInertia0 存储 I⁻¹ 的对角元素。
// ============================================================================

import * as THREE from 'three';
import { Pose, conj, applyRotationalCorrection } from './math3d.js';
import { shapeVolume, shapeInertia, shapeSupportLocal, ShapeType } from './shapes.js';

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
            // 陀螺力矩：−ω × (Iω)，在 rest 系下求值
            const Iw = new THREE.Vector3(wLoc.x / invI0.x, wLoc.y / invI0.y, wLoc.z / invI0.z);
            const gyro = new THREE.Vector3().crossVectors(wLoc, Iw);
            wLoc.addScaledVector(gyro, -h);
        }
        if (this.torque.lengthSq() > 0) {
            const tLoc = this.torque.clone().applyQuaternion(qInv);
            wLoc.x += h * tLoc.x * invI0.x;
            wLoc.y += h * tLoc.y * invI0.y;
            wLoc.z += h * tLoc.z * invI0.z;
        }

        this.omega.copy(wLoc.applyQuaternion(this.pose.q));

        // 论文 Algorithm 2：q ← q + h·½[ω,0]q，再归一化
        applyRotationalCorrection(this.pose, this.omega, h);
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

        // Δq = q · q_prev⁻¹
        const dq = new THREE.Quaternion().multiplyQuaternions(this.pose.q, conj(this.prevPose.q));
        this.omega.set((2.0 * dq.x) / h, (2.0 * dq.y) / h, (2.0 * dq.z) / h);
        // 四元数双覆盖：Δq_w < 0 时旋转向量取反
        if (dq.w < 0.0) this.omega.multiplyScalar(-1.0);

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
        out.subVectors(worldPoint, this.pose.p).cross(this.omega);
        return out.add(this.vel);
    }

    /** 用**速度更新前**的快照求某点速度（论文 §3.6 恢复系数所需 v̄n） */
    getVelocityAtPrev(worldPoint, out = new THREE.Vector3()) {
        out.subVectors(worldPoint, this.pose.p).cross(this.prevOmega);
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

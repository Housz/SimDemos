// ============================================================================
// world.js —— 仿真世界，论文 Algorithm 2 的完整实现
//
//   while simulating do
//       CollectCollisionPairs();                                  ← 每帧一次（宽相）
//       h ← Δt / numSubsteps;
//       for numSubsteps do
//           for n bodies and particles do
//               x_prev ← x;  v ← v + h·f_ext/m;  x ← x + h·v;
//               q_prev ← q;  ω ← ω + h·I⁻¹(τ_ext − ω×(Iω));
//               q ← q + h·½[ω,0]q;  q ← q/|q|;                    ← 显式积分
//           end
//           for numPosIters do
//               SolvePositions(x₁…xₙ, q₁…qₙ);                     ← 非线性投影 Gauss-Seidel
//           end
//           for n bodies and particles do
//               v ← (x − x_prev)/h;
//               Δq ← q·q_prev⁻¹;  ω ← 2[Δq.xyz]/h;  ω ← (Δq_w ≥ 0 ? ω : −ω);
//           end
//           SolveVelocities(v₁…vₙ, ω₁…ωₙ);                        ← 摩擦/恢复/阻尼
//       end
//   end
//
// 论文强调：numPosIters 通常取 **1**，把时间预算全部用在**子步数**上
// （Macklin et al. [MSL*19] 的结论；论文图 9、11 用能量守恒与约束伸长验证）。
// 由于 XPBD 的无条件稳定性，时间步长不需要为稳定性调参。
// ============================================================================

import * as THREE from 'three';
import { RigidBody } from './rigidBody.js';
import { Contact, collide } from './contacts.js';
import { collectPairs } from './broadphase.js';

export class World {
    constructor({
        gravity = new THREE.Vector3(0, -9.81, 0),
        numSubsteps = 20,
        numPosIters = 1,
        useGyroscopic = true,
        broadphaseK = 2.0,
    } = {}) {
        this.bodies = [];
        this.joints = [];
        this.gravity = gravity.clone();
        this.numSubsteps = numSubsteps;
        this.numPosIters = numPosIters;
        this.useGyroscopic = useGyroscopic;
        this.broadphaseK = broadphaseK;

        this.contacts = [];          // 本子步的接触约束
        this._contactPool = [];      // 接触对象池，避免每子步分配
        this._pairs = [];            // 本帧的候选对缓存
        this.pairsCount = 0;

        // 统计
        this.energy = 0.0;
        this.maxPenetration = 0.0;
    }

    addBody(body) {
        this.bodies.push(body);
        return body;
    }

    addJoint(joint) {
        this.joints.push(joint);
        return joint;
    }

    /** 便捷构造并加入刚体 */
    createBody(opts) {
        return this.addBody(new RigidBody(opts));
    }

    removeBody(body) {
        const i = this.bodies.indexOf(body);
        if (i >= 0) this.bodies.splice(i, 1);
    }

    clear() {
        this.bodies.length = 0;
        this.joints.length = 0;
        this.contacts.length = 0;
        this._contactPool.length = 0;
        this._pairs.length = 0;
    }

    /**
     * 推进一步（一个 Δt）。
     * @param {number} dt 时间步长（论文所有例子取 1/60 s）
     */
    step(dt) {
        const h = dt / this.numSubsteps;
        const N = this.numSubsteps;
        const g = this.gravity;
        const gMag = g.length();

        // --- 宽相：每帧只收集一次候选碰撞对（论文 §3.5）---
        collectPairs(this.bodies, dt, this.broadphaseK, this._pairs);
        this.pairsCount = this._pairs.length;

        for (let s = 0; s < N; s++) {
            // --- 1. 显式积分（论文 Algorithm 2 第一段）---
            for (let i = 0; i < this.bodies.length; i++) {
                const b = this.bodies[i];
                if (b.isDynamic) b.clearForces();
                b.integrate(h, g, this.useGyroscopic);
            }

            // --- 2. 窄相：生成/更新接触约束（方程 27 的局部锚点在此确定）---
            this._generateContacts();

            // --- 3. 位置求解：非线性投影 Gauss-Seidel ---
            for (let i = 0; i < this.joints.length; i++) this.joints[i].resetLambda();
            for (let i = 0; i < this.contacts.length; i++) this.contacts[i].resetLambda();

            for (let it = 0; it < this.numPosIters; it++) {
                for (let i = 0; i < this.joints.length; i++) this.joints[i].solvePos(h);
                for (let i = 0; i < this.contacts.length; i++) this.contacts[i].solvePos(h);
            }

            // --- 4. 由位置差分导出速度 + 保存子步前速度快照 ---
            for (let i = 0; i < this.bodies.length; i++) {
                const b = this.bodies[i];
                b.snapshotVelocity();
                b.update(h, false);
            }

            // --- 5. 速度层：动摩擦、恢复系数、关节阻尼（论文 §3.6）---
            for (let i = 0; i < this.contacts.length; i++) this.contacts[i].solveVel(h, gMag);
            for (let i = 0; i < this.joints.length; i++) this.joints[i].solveVel(h);
        }

        // 同步可视化对象
        for (let i = 0; i < this.bodies.length; i++) this.bodies[i].syncMesh();

        this._updateStats();
    }

    _generateContacts() {
        this.contacts.length = 0;
        let slot = 0;
        this.maxPenetration = 0.0;

        for (let p = 0; p < this._pairs.length; p++) {
            const [a, b] = this._pairs[p];
            const descs = collide(a, b);
            for (let d = 0; d < descs.length; d++) {
                let c = this._contactPool[slot];
                if (c === undefined) {
                    c = new Contact(a, b, descs[d]);
                    this._contactPool[slot] = c;
                } else {
                    c.init(a, b, descs[d]);
                }
                if (c.depth > this.maxPenetration) this.maxPenetration = c.depth;
                this.contacts.push(c);
                slot++;
            }
        }
        // 池中多余的接触对象不再使用（保留以便复用）
    }

    /** 总机械能（动能 + 重力势能），用于论文图 9 的能量守恒验证 */
    _updateStats() {
        let e = 0.0;
        for (const b of this.bodies) {
            if (!b.isDynamic) continue;
            const v2 = b.vel.lengthSq();
            const w = b.omega;
            const qInv = b.pose.q;
            // 转动动能：½ ωᵀ I ω（在 rest 系下用对角 I 求值）
            const wLocal = w.clone().applyQuaternion(new THREE.Quaternion(-qInv.x, -qInv.y, -qInv.z, qInv.w));
            const invI = b.invInertia0;
            const rotKE = 0.5 * (
                (invI.x > 0 ? wLocal.x * wLocal.x / invI.x : 0) +
                (invI.y > 0 ? wLocal.y * wLocal.y / invI.y : 0) +
                (invI.z > 0 ? wLocal.z * wLocal.z / invI.z : 0));
            e += 0.5 * b.mass * v2 + rotKE - b.mass * this.gravity.dot(b.pose.p);
        }
        this.energy = e;
    }
}

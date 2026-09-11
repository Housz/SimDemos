// ============================================================================
// joints.js —— 关节（论文 §3.4「Joints」）
//
// 论文把关节看作对两刚体相对位置/转动自由度的限制，并把每种关节都
// 拆解为若干“基本投影操作”的**有序序列**（一次 Gauss-Seidel 扫描内逐个投影）。
//
//   关节类型        位置自由度                        转动自由度
//   ---------------------------------------------------------------------
//   DistanceJoint   距离（可弹簧/限长/驱动）           无约束
//   FixedJoint      完全固定                           完全固定
//   HingeJoint      固定（铰链点重合）                 仅绕铰链轴自由 + 限制/马达
//   SphericalJoint  固定（球心重合）                   摆动(swing)/扭转(twist) 限制
//   PrismaticJoint  仅沿滑动轴可动 + 上下限            锁定其余转动
//
// 关节坐标系：每个关节在两侧刚体上各有一个局部位姿（Pose），
// 关节的局部 X 轴为铰链轴 / 滑动轴，Y、Z 为其正交轴。
// ============================================================================

import * as THREE from 'three';
import { Pose, conj, quatToRotationVector } from './math3d.js';
import { applyBodyPairCorrection, applyAngularCorrection, limitAngle, wrapPi } from './constraints.js';

// ---------------------------------------------------------------------------
// 关节基类
// ---------------------------------------------------------------------------
export class Joint {
    /**
     * @param {RigidBody} bodyA
     * @param {RigidBody} bodyB
     * @param {Pose} localPoseA 关节坐标系在 bodyA 局部系下的位姿
     * @param {Pose} localPoseB 关节坐标系在 bodyB 局部系下的位姿
     */
    constructor(bodyA, bodyB, localPoseA = new Pose(), localPoseB = new Pose()) {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.localPoseA = localPoseA.clone();
        this.localPoseB = localPoseB.clone();
        this.globalPoseA = localPoseA.clone();
        this.globalPoseB = localPoseB.clone();

        this.compliance = 0.0;      // 柔度 α（m/N），0 = 无限硬
        this.posDamping = 0.0;      // 线性阻尼 μ_lin（论文 Eq. 32）
        this.rotDamping = 0.0;      // 角阻尼 μ_ang（论文 Eq. 33）

        this.lambda = {};           // 子约束的 λ 累加器（每子步清零）
        this.debugTorque = new THREE.Vector3();
    }

    /** 每子步开始时清零 Lagrange 乘子（论文 §3.3.1：求解前 λ 置零） */
    resetLambda() {
        for (const k in this.lambda) this.lambda[k] = 0.0;
    }

    /** 由刚体当前位姿计算关节坐标系的世界位姿 */
    updateGlobalPoses() {
        this.globalPoseA.copy(this.localPoseA);
        if (this.bodyA) this.bodyA.pose.transformPose(this.globalPoseA);
        this.globalPoseB.copy(this.localPoseB);
        if (this.bodyB) this.bodyB.pose.transformPose(this.globalPoseB);
    }

    /** 位置层求解（子类实现） */
    solvePos(/* h */) {}

    /** 速度层：关节阻尼（论文 §3.6, Eqs. 32-34） */
    solveVel(h) {
        if (this.rotDamping <= 0.0 && this.posDamping <= 0.0) return;

        if (this.rotDamping > 0.0) {
            // Δω = (ω2 − ω1)·min(μ_ang·h, 1)
            const dOmega = new THREE.Vector3();
            if (this.bodyA) dOmega.sub(this.bodyA.omega);
            if (this.bodyB) dOmega.add(this.bodyB.omega);
            dOmega.multiplyScalar(Math.min(1.0, this.rotDamping * h));
            applyBodyPairCorrection(this.bodyA, this.bodyB, dOmega, 0.0, h, null, null, null, null, true);
        }

        if (this.posDamping > 0.0) {
            // Δv = (v2 − v1)·min(μ_lin·h, 1)，作用在两锚点处
            this.updateGlobalPoses();
            const pA = this.globalPoseA.p;
            const pB = this.globalPoseB.p;
            const vA = this.bodyA ? this.bodyA.getVelocityAt(pA) : new THREE.Vector3();
            const vB = this.bodyB ? this.bodyB.getVelocityAt(pB) : new THREE.Vector3();
            const dv = vB.sub(vA).multiplyScalar(Math.min(1.0, this.posDamping * h));
            applyBodyPairCorrection(this.bodyA, this.bodyB, dv, 0.0, h, pA, pB, null, null, true);
        }
    }
}

// ---------------------------------------------------------------------------
// 距离关节（论文 §3.4.2 Positional Degrees of Freedom）
//
//   Δr = r2 − r1
//   固定连接：   Δx = Δr                                   （α > 0 时即零长度弹簧）
//   上限（绳）： |Δr| > d_max 时 Δx = Δr/|Δr|·(|Δr| − d_max) （Eq. 26）
//   目标驱动：   Δx = Δr/|Δr|·(|Δr| − d_target)，无条件施加
// ---------------------------------------------------------------------------
export class DistanceJoint extends Joint {
    constructor(bodyA, bodyB, localAnchorA, localAnchorB, {
        restLength = 0.0,
        compliance = 0.0,
        maxLength = Infinity,   // 绳约束上限（Infinity = 不限制）
        minLength = 0.0,
        isSpring = false,       // true 时使用 restLength 作为弹簧原长
        targetLength = null,    // 非 null 时驱动到该长度（马达）
        damping = 0.0,
    } = {}) {
        super(bodyA, bodyB, new Pose(localAnchorA), new Pose(localAnchorB));
        this.restLength = restLength;
        this.compliance = compliance;
        this.maxLength = maxLength;
        this.minLength = minLength;
        this.isSpring = isSpring;
        this.targetLength = targetLength;
        this.posDamping = damping;
        this.lastForce = 0.0; // 最近一次求解得到的约束力大小，用于可视化
    }

    /** 当前两锚点距离 */
    getDistance() {
        this.updateGlobalPoses();
        return this.globalPoseB.p.distanceTo(this.globalPoseA.p);
    }

    solvePos(h) {
        this.updateGlobalPoses();
        const pA = this.globalPoseA.p;
        const pB = this.globalPoseB.p;
        const delta = new THREE.Vector3().subVectors(pB, pA);
        const dist = delta.length();
        if (dist < 1e-12) return;

        let target = null;

        if (this.targetLength !== null) {
            // 马达：无条件驱动到目标长度
            target = this.targetLength;
        } else if (this.isSpring) {
            target = this.restLength;
        } else {
            // 单向限制：只在越界时施加修正（论文 Eq. 26 的绳约束 / 最小长度）
            if (dist > this.maxLength) target = this.maxLength;
            else if (dist < this.minLength) target = this.minLength;
        }
        if (target === null) return;

        const corr = delta.multiplyScalar((dist - target) / dist);
        const lambda = applyBodyPairCorrection(
            this.bodyA, this.bodyB, corr, this.compliance, h, pA, pB, this.lambda, 'dist');
        this.lastForce = Math.abs(lambda) / (h * h); // 论文 Eq. 11: f = λn/h²
    }
}

// ---------------------------------------------------------------------------
// 固定关节（论文 §3.4.1）
//
//   q = q1·q2⁻¹;  Δq_fixed = 2(q.x, q.y, q.z)      （Eqs. 19-20）
//   位置部分：Δx = Δr
// ---------------------------------------------------------------------------
export class FixedJoint extends Joint {
    constructor(bodyA, bodyB, localPoseA = new Pose(), localPoseB = new Pose(), { compliance = 0.0, rotCompliance = 0.0, damping = 0.0 } = {}) {
        super(bodyA, bodyB, localPoseA, localPoseB);
        this.compliance = compliance;
        this.rotCompliance = rotCompliance;
        this.posDamping = damping;
        this.rotDamping = damping;
    }

    solvePos(h) {
        // --- 转动：把相对朝向驱动到一致 ---
        this.updateGlobalPoses();
        {
            const q = new THREE.Quaternion().multiplyQuaternions(
                this.globalPoseA.q, conj(this.globalPoseB.q));
            const rotVec = quatToRotationVector(q);
            applyAngularCorrection(this.bodyA, this.bodyB, rotVec, this.rotCompliance, h, this.lambda, 'rot');
        }

        // --- 位置：锚点重合 ---
        this.updateGlobalPoses();
        {
            const pA = this.globalPoseA.p;
            const pB = this.globalPoseB.p;
            const delta = new THREE.Vector3().subVectors(pB, pA);
            applyBodyPairCorrection(this.bodyA, this.bodyB, delta, this.compliance, h, pA, pB, this.lambda, 'pos');
        }
    }
}

// ---------------------------------------------------------------------------
// 铰链关节（论文 §3.4.1）
//
//   轴对齐：   Δq_hinge = a1 × a2                            （Eq. 21）
//   角度限制： LimitAngle([n,n1,n2] = [a1,b1,b2], α, β)      （Algorithm 3）
//   目标角度： b_target = rot(a1, α)·b1;  Δq_target = b_target × b2 （Eq. 22）
//   马达：     α ← α + h·v（每子步推进目标角，柔度控制力矩大小）
//   位置：     铰链点重合
// ---------------------------------------------------------------------------
export class HingeJoint extends Joint {
    constructor(bodyA, bodyB, localPoseA = new Pose(), localPoseB = new Pose(), {
        compliance = 0.0,
        limitCompliance = 0.0,
        minAngle = -Math.PI,
        maxAngle = Math.PI,
        targetAngle = null,        // 非 null 时启用目标角度约束
        targetCompliance = 0.0,
        motorVelocity = 0.0,       // 马达目标角速度（rad/s）
        damping = 0.0,
    } = {}) {
        super(bodyA, bodyB, localPoseA, localPoseB);
        this.compliance = compliance;
        this.limitCompliance = limitCompliance;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.targetAngle = targetAngle;
        this.targetCompliance = targetCompliance;
        this.motorVelocity = motorVelocity;
        this.posDamping = damping;
        this.rotDamping = damping;
        this.lastTorque = 0.0;
    }

    /** 当前铰链两侧的相对夹角（用于显示） */
    getCurrentAngle() {
        this.updateGlobalPoses();
        const a = this.globalPoseA.q;
        const b = this.globalPoseB.q;
        const b1 = new THREE.Vector3(0, 1, 0).applyQuaternion(a);
        const b2 = new THREE.Vector3(0, 1, 0).applyQuaternion(b);
        const a1 = new THREE.Vector3(1, 0, 0).applyQuaternion(a);
        const c = new THREE.Vector3().crossVectors(b1, b2);
        let phi = Math.asin(THREE.MathUtils.clamp(c.dot(a1), -1.0, 1.0));
        if (b1.dot(b2) < 0.0) phi = Math.PI - phi;
        return wrapPi(phi);
    }

    solvePos(h) {
        // 马达：推进目标角（论文 §3.4.1）
        if (this.targetAngle !== null && this.motorVelocity !== 0.0) {
            this.targetAngle = wrapPi(this.targetAngle + h * this.motorVelocity);
        }

        // 1) 对齐铰链轴：Δq = a1 × a2
        this.updateGlobalPoses();
        {
            const a1 = new THREE.Vector3(1, 0, 0).applyQuaternion(this.globalPoseA.q);
            const a2 = new THREE.Vector3(1, 0, 0).applyQuaternion(this.globalPoseB.q);
            const corr = new THREE.Vector3().crossVectors(a1, a2);
            applyAngularCorrection(this.bodyA, this.bodyB, corr, 0.0, h, this.lambda, 'axis');
        }

        // 2) 角度限制（Algorithm 3），公共轴取 a1
        if (this.minAngle > -Math.PI || this.maxAngle < Math.PI) {
            this.updateGlobalPoses();
            const a1 = new THREE.Vector3(1, 0, 0).applyQuaternion(this.globalPoseA.q);
            const b1 = new THREE.Vector3(0, 1, 0).applyQuaternion(this.globalPoseA.q);
            const b2 = new THREE.Vector3(0, 1, 0).applyQuaternion(this.globalPoseB.q);
            limitAngle(this.bodyA, this.bodyB, a1, b1, b2,
                this.minAngle, this.maxAngle, this.limitCompliance, h, this.lambda, 'limit');
        }

        // 3) 目标角度 / 马达
        if (this.targetAngle !== null) {
            this.updateGlobalPoses();
            const a1 = new THREE.Vector3(1, 0, 0).applyQuaternion(this.globalPoseA.q);
            const b1 = new THREE.Vector3(0, 1, 0).applyQuaternion(this.globalPoseA.q);
            const b2 = new THREE.Vector3(0, 1, 0).applyQuaternion(this.globalPoseB.q);
            const q = new THREE.Quaternion().setFromAxisAngle(a1, this.targetAngle);
            const bTarget = b1.clone().applyQuaternion(q);
            const corr = new THREE.Vector3().crossVectors(bTarget, b2);
            const lambda = applyAngularCorrection(this.bodyA, this.bodyB, corr,
                this.targetCompliance, h, this.lambda, 'target');
            this.lastTorque = Math.abs(lambda) / (h * h); // 论文 Eq. 18
        }

        // 4) 位置：铰链点重合
        this.updateGlobalPoses();
        {
            const pA = this.globalPoseA.p;
            const pB = this.globalPoseB.p;
            const delta = new THREE.Vector3().subVectors(pB, pA);
            applyBodyPairCorrection(this.bodyA, this.bodyB, delta, this.compliance, h, pA, pB, this.lambda, 'pos');
        }
    }
}

// ---------------------------------------------------------------------------
// 球窝关节（ball-in-socket，论文 §3.4.1）
//
//   位置：两球心重合
//   摆动限制（swing）：[n, n1, n2] = [a1 × a2, a1, a2]
//   扭转限制（twist）： 把扭转与摆动解耦（论文 Eqs. 23-25）
//        n  ← (a1 + a2)/|a1 + a2|
//        n1 ← b1 − (n·b1)n, 再归一化
//        n2 ← b2 − (n·b2)n, 再归一化
// ---------------------------------------------------------------------------
export class SphericalJoint extends Joint {
    constructor(bodyA, bodyB, localPoseA = new Pose(), localPoseB = new Pose(), {
        compliance = 0.0,
        swingLimit = null,        // [min, max] 弧度；null 表示不限制
        swingCompliance = 0.0,
        twistLimit = null,
        twistCompliance = 0.0,
        damping = 0.0,
    } = {}) {
        super(bodyA, bodyB, localPoseA, localPoseB);
        this.compliance = compliance;
        this.swingLimit = swingLimit;
        this.swingCompliance = swingCompliance;
        this.twistLimit = twistLimit;
        this.twistCompliance = twistCompliance;
        this.posDamping = damping;
        this.rotDamping = damping;
    }

    solvePos(h) {
        // 1) 位置：球心重合
        this.updateGlobalPoses();
        {
            const pA = this.globalPoseA.p;
            const pB = this.globalPoseB.p;
            const delta = new THREE.Vector3().subVectors(pB, pA);
            applyBodyPairCorrection(this.bodyA, this.bodyB, delta, this.compliance, h, pA, pB, this.lambda, 'pos');
        }

        // 2) 摆动限制：[a1 × a2, a1, a2]
        if (this.swingLimit) {
            this.updateGlobalPoses();
            const a1 = new THREE.Vector3(1, 0, 0).applyQuaternion(this.globalPoseA.q);
            const a2 = new THREE.Vector3(1, 0, 0).applyQuaternion(this.globalPoseB.q);
            const n = new THREE.Vector3().crossVectors(a1, a2);
            if (n.lengthSq() > 1e-16) {
                n.normalize();
                limitAngle(this.bodyA, this.bodyB, n, a1, a2,
                    this.swingLimit[0], this.swingLimit[1], this.swingCompliance, h, this.lambda, 'swing');
            }
        }

        // 3) 扭转限制：先把扭转从摆动中解耦（论文 Eqs. 23-25）
        if (this.twistLimit) {
            this.updateGlobalPoses();
            const a1 = new THREE.Vector3(1, 0, 0).applyQuaternion(this.globalPoseA.q);
            const a2 = new THREE.Vector3(1, 0, 0).applyQuaternion(this.globalPoseB.q);
            const n = new THREE.Vector3().addVectors(a1, a2);
            if (n.lengthSq() > 1e-12) {
                n.normalize();
                const n1 = new THREE.Vector3(0, 1, 0).applyQuaternion(this.globalPoseA.q);
                n1.addScaledVector(n, -n.dot(n1));
                const n2 = new THREE.Vector3(0, 1, 0).applyQuaternion(this.globalPoseB.q);
                n2.addScaledVector(n, -n.dot(n2));

                if (n1.lengthSq() > 1e-12 && n2.lengthSq() > 1e-12) {
                    n1.normalize();
                    n2.normalize();
                    // 万向节死锁保护：两轴接近反向时限制单步修正量
                    const maxCorr = a1.dot(a2) > -0.5 ? 2.0 * Math.PI : 1.0 * h;
                    limitAngle(this.bodyA, this.bodyB, n, n1, n2,
                        this.twistLimit[0], this.twistLimit[1], this.twistCompliance, h, this.lambda, 'twist', maxCorr);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 棱柱关节（论文 §3.4.2）
//
//   位置：沿滑动轴（关节局部 X 轴）可动，受 [minSlide, maxSlide] 限制；
//         另两轴锁定为 0。所有轴的限制用**单次**约束投影完成。
//   转动：锁定绕 Y、Z 轴的相对转动（对齐 b、c 轴）
// ---------------------------------------------------------------------------
export class PrismaticJoint extends Joint {
    constructor(bodyA, bodyB, localPoseA = new Pose(), localPoseB = new Pose(), {
        compliance = 0.0,
        minSlide = -Infinity,
        maxSlide = Infinity,
        targetSlide = null,        // 非 null 时驱动到该位移（马达）
        targetCompliance = 0.0,
        motorVelocity = 0.0,
        damping = 0.0,
        limits = null,             // 可选：[[xlo,xhi],[ylo,yhi],[zlo,zhi]] 覆盖默认
    } = {}) {
        super(bodyA, bodyB, localPoseA, localPoseB);
        this.compliance = compliance;
        this.minSlide = minSlide;
        this.maxSlide = maxSlide;
        this.targetSlide = targetSlide;
        this.targetCompliance = targetCompliance;
        this.motorVelocity = motorVelocity;
        this.posDamping = damping;
        this.rotDamping = damping;
        this.limits = limits || [[minSlide, maxSlide], [0, 0], [0, 0]];
    }

    solvePos(h) {
        if (this.targetSlide !== null && this.motorVelocity !== 0.0) {
            this.targetSlide += h * this.motorVelocity;
        }

        // 1) 转动：锁定 b、c 轴（只保留绕 a 轴的转动自由度）
        this.updateGlobalPoses();
        {
            const b1 = new THREE.Vector3(0, 1, 0).applyQuaternion(this.globalPoseA.q);
            const b2 = new THREE.Vector3(0, 1, 0).applyQuaternion(this.globalPoseB.q);
            applyAngularCorrection(this.bodyA, this.bodyB,
                new THREE.Vector3().crossVectors(b1, b2), 0.0, h, this.lambda, 'rotB');
            this.updateGlobalPoses();
            const c1 = new THREE.Vector3(0, 0, 1).applyQuaternion(this.globalPoseA.q);
            const c2 = new THREE.Vector3(0, 0, 1).applyQuaternion(this.globalPoseB.q);
            applyAngularCorrection(this.bodyA, this.bodyB,
                new THREE.Vector3().crossVectors(c1, c2), 0.0, h, this.lambda, 'rotC');
        }

        // 2) 位置：逐轴限制，合成单一修正向量
        this.updateGlobalPoses();
        {
            const pA = this.globalPoseA.p;
            const pB = this.globalPoseB.p;
            const offset = new THREE.Vector3().subVectors(pB, pA);
            const axes = [
                new THREE.Vector3(1, 0, 0).applyQuaternion(this.globalPoseA.q),
                new THREE.Vector3(0, 1, 0).applyQuaternion(this.globalPoseA.q),
                new THREE.Vector3(0, 0, 1).applyQuaternion(this.globalPoseA.q),
            ];
            const limits = this.limits.map((l) => l.slice());

            if (this.targetSlide !== null) {
                // 马达：把滑动轴驱动到目标位移（无条件施加，柔度控制推力）
                const d = offset.dot(axes[0]);
                const corr = axes[0].clone().multiplyScalar(d - this.targetSlide);
                applyBodyPairCorrection(this.bodyA, this.bodyB, corr, this.targetCompliance, h, pA, pB, this.lambda, 'slide');
                limits[0] = null; // 该轴已由马达处理
            }

            const corr = new THREE.Vector3(0, 0, 0);
            for (let i = 0; i < 3; i++) {
                if (limits[i] === null) continue;
                const d = offset.dot(axes[i]);
                const [lo, hi] = limits[i];
                if (d < lo) corr.addScaledVector(axes[i], d - lo);
                else if (d > hi) corr.addScaledVector(axes[i], d - hi);
            }
            if (corr.lengthSq() > 0.0) {
                applyBodyPairCorrection(this.bodyA, this.bodyB, corr, this.compliance, h, pA, pB, this.lambda, 'pos');
            }
        }
    }
}

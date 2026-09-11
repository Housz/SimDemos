// ============================================================================
// contacts.js —— 接触与摩擦（论文 §3.5「Handling Contacts and Friction」与 §3.6）
//
// 论文的关键设计：**不存储静态接触平面**，而是存储“对物体的引用 + 局部接触点”，
// 并在**每一次单独的约束投影之前重新计算接触法线**。这样圆角、曲面几何、
// 一帧内发生大角度转动的物体都能被正确处理（论文图 1、13、14）。
//
// 位置层（每子步）：
//   p1 = x1 + q1·r1 ;  p2 = x2 + q2·r2 ;  p̄1 = x1,prev + q1,prev·r1 ;  p̄2 = ...   (Eq. 27)
//   d = (p1 − p2)·n ，d ≤ 0 则跳过；否则施加 Δx = d·n（α = 0）
//   Δp = (p1 − p̄1) − (p2 − p̄2) ;  Δp_t = Δp − (Δp·n)n                                 (Eqs. 28-29)
//   静摩擦：施加 −Δp_t 以抵消切向滑动，仅当 λt < μs·λn
//
// 速度层（每子步一次）：
//   v = (v1 + ω1×r1) − (v2 + ω2×r2) ;  vn = n·v ;  vt = v − n·vn                      (Eq. 30)
//   动摩擦：Δv = −(vt/|vt|)·min(h·μd·fn, |vt|)，fn = λn/h²，μ = (μ1+μ2)/2            (Eq. 31)
//   恢复系数：Δv = n(−vn + max(−e·v̄n, 0))，e 在 |vn| ≤ 2|g|h 时置 0                  (Eq. 35)
//
// 法线方向约定（全库统一）：
//   n 由 bodyB 指向 bodyA，即“把 A 推开以解除穿透”的方向；
//   施加修正 corr = depth·n 时 A 沿 +n 运动、B 沿 −n 运动。
// ============================================================================

import * as THREE from 'three';
import { applyBodyPairCorrection } from './constraints.js';
import { ShapeType } from './shapes.js';

const EPS = 1e-9;
const ZERO = new THREE.Vector3(0, 0, 0);
const AXIS_X = new THREE.Vector3(1, 0, 0);
const AXIS_Y = new THREE.Vector3(0, 1, 0);
const AXIS_Z = new THREE.Vector3(0, 0, 1);
const LOCAL_AXES = [AXIS_X, AXIS_Y, AXIS_Z];

// ===========================================================================
// 接触约束：位置层 + 速度层
// ===========================================================================
export class Contact {
    /**
     * @param {RigidBody} bodyA
     * @param {RigidBody} bodyB
     * @param {object} desc 窄相检测结果（见 makeDesc）
     */
    constructor(bodyA, bodyB, desc) {
        this.n = new THREE.Vector3();
        this.p1 = new THREE.Vector3();
        this.p2 = new THREE.Vector3();
        this.p1Prev = new THREE.Vector3();
        this.p2Prev = new THREE.Vector3();
        this.depth = 0.0;
        this.lambda = { n: 0.0, t: 0.0 }; // 法向/切向 Lagrange 乘子（每子步清零）
        this.lambdaN = 0.0;               // 位置求解得到的法向乘子大小 → 摩擦锥
        this.active = false;              // 本子步窄相判定时是否真正穿透（速度层的开关）
        this.init(bodyA, bodyB, desc);
    }

    /** 初始化/复用：接触对象由 World 池化，每子步重新绑定 */
    init(bodyA, bodyB, desc) {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.normalType = desc.normalType;
        this.normalData = desc.normalData;
        // 局部锚点：检测时刻确定，本子步内保持不变（论文 §3.5）
        this.localAnchorA = desc.localAnchorA;
        this.localAnchorB = desc.localAnchorB;

        // 材质参数：两侧系数取平均（论文 §3.5）
        this.staticFriction = 0.5 * (bodyA.staticFriction + bodyB.staticFriction);
        this.dynamicFriction = 0.5 * (bodyA.dynamicFriction + bodyB.dynamicFriction);
        this.restitution = Math.max(bodyA.restitution, bodyB.restitution);

        this.resetLambda();
        this.update();
        // 本子步的“活跃”状态：窄相判定时刻确实穿透。
        // 速度层只对本子步活跃的接触生效——位置投影之后物体可能已被推离表面，
        // 若此时仍施加 Eq. 35 的“抹掉法向速度”，就会把上一子步刚获得的弹跳速度
        // （e>0 时的反射速度）一并抹掉。
        this.active = this.depth > 0.0;
        return this;
    }

    resetLambda() {
        this.lambda.n = 0.0;
        this.lambda.t = 0.0;
        this.lambdaN = 0.0;
    }

    /**
     * 依据当前位姿重算法线，并由固定的局部锚点得到 p1、p2、p̄1、p̄2
     * （论文 Eq. 27；“每次投影前重算法线”是本文方法的核心）。
     */
    update() {
        const A = this.bodyA;
        const B = this.bodyB;
        const n = this.n;
        const d = this.normalData;

        switch (this.normalType) {
            case 'fixed':
                // 静态平面法线等世界系常向量
                n.copy(d);
                break;

            case 'centers':
                // 球-球：由当前球心连线确定
                n.subVectors(A.pose.p, B.pose.p);
                if (n.lengthSq() < 1e-16) n.set(0, 1, 0);
                else n.normalize();
                break;

            case 'sphereSegment': {
                // 球-胶囊（或作为轨道的静态线段）：由球心指向轴段最近点
                const axis = d.axisLocal.clone().applyQuaternion(d.capsule.pose.q);
                const toSphere = new THREE.Vector3().subVectors(d.sphere.pose.p, d.capsule.pose.p);
                const t = THREE.MathUtils.clamp(toSphere.dot(axis), -d.halfLength, d.halfLength);
                const closest = d.capsule.pose.p.clone().addScaledVector(axis, t);
                n.subVectors(d.sphere.pose.p, closest);
                if (n.lengthSq() < 1e-16) n.set(0, 1, 0);
                else n.normalize();
                // 若球是 B，法线需反向（保持“由 B 指向 A”）
                if (d.sphere === B) n.negate();
                break;
            }

            case 'face': {
                // 盒-盒面接触：参考面法线随参考刚体转动
                const owner = d.onB ? B : A;
                n.copy(LOCAL_AXES[d.axisIndex]).applyQuaternion(owner.pose.q).multiplyScalar(d.sign);
                // 数值保护：法线必须大致由 B 指向 A
                if (n.dot(TMP_BA.subVectors(A.pose.p, B.pose.p)) < 0) n.negate();
                break;
            }

            case 'edges': {
                // 盒-盒棱接触：法线 = 两条棱当前世界方向的叉积（每次投影前重算），
                // 并按“由 B 指向 A”定向（两侧质心连线仅用于定号）。
                const a1 = TMP_E1.copy(d.dirA).applyQuaternion(A.pose.q);
                const a2 = TMP_E2.copy(d.dirB).applyQuaternion(B.pose.q);
                n.crossVectors(a1, a2);
                if (n.lengthSq() < 1e-12) n.copy(d.fallback);
                n.normalize();
                if (n.dot(TMP_BA.subVectors(A.pose.p, B.pose.p)) < 0) n.negate();
                break;
            }

            default:
                n.set(0, 1, 0);
        }

        // --- 由固定锚点求当前/子步前的世界接触点（论文 Eq. 27）---
        this.p1.copy(this.localAnchorA);
        A.pose.transform(this.p1);
        this.p2.copy(this.localAnchorB);
        B.pose.transform(this.p2);
        this.p1Prev.copy(this.localAnchorA);
        A.prevPose.transform(this.p1Prev);
        this.p2Prev.copy(this.localAnchorB);
        B.prevPose.transform(this.p2Prev);

        // 穿透深度 d = (p2 − p1)·n（正值表示相互穿透）
        this.depth = TMP_D.subVectors(this.p2, this.p1).dot(n);
    }

    // -----------------------------------------------------------------------
    // 位置层求解（论文 §3.5）
    // -----------------------------------------------------------------------
    solvePos(h) {
        this.update();
        if (this.depth <= 0.0) return;

        const A = this.bodyA;
        const B = this.bodyB;

        // --- 法向约束：Δx = d·n，α = 0（无限硬）---
        const corr = TMP_C.copy(this.n).multiplyScalar(this.depth);
        const lambda = applyBodyPairCorrection(A, B, corr, 0.0, h, this.p1, this.p2, this.lambda, 'n');
        this.lambdaN = Math.abs(lambda);

        // --- 静摩擦（论文 Eqs. 28-29）---
        if (this.staticFriction <= 0.0) return;

        // Δp = (p1 − p̄1) − (p2 − p̄2)
        const dp = TMP_DP.subVectors(this.p1, this.p1Prev)
            .sub(TMP_DP2.subVectors(this.p2, this.p2Prev));
        // 切向分量 Δp_t
        dp.addScaledVector(this.n, -dp.dot(this.n));
        const mag = dp.length();
        if (mag < EPS) return;

        // 施加 −Δp_t 抵消切向滑动
        const nT = dp.multiplyScalar(-1.0 / mag);
        const rA = new THREE.Vector3().subVectors(this.p1, A.pose.p);
        const rB = new THREE.Vector3().subVectors(this.p2, B.pose.p);
        const wA = A.getGeneralizedInvMass(nT, rA);
        const wB = B.getGeneralizedInvMass(nT, rB);
        const w = wA + wB;
        if (w < EPS) return;

        const lambdaT = mag / w;
        // 论文：仅当 λt < μs·λn 时施加（超出则转为滑动，由速度层动摩擦处理）
        if (lambdaT < this.staticFriction * this.lambdaN) {
            const corrT = nT.multiplyScalar(mag);
            applyBodyPairCorrection(A, B, corrT, 0.0, h, this.p1, this.p2, this.lambda, 't');
        }
    }

    // -----------------------------------------------------------------------
    // 速度层求解（论文 §3.6）
    // -----------------------------------------------------------------------
    solveVel(h, gravityMagnitude) {
        if (!this.active) return; // 本子步未真正穿透 → 速度层不介入

        // 位置投影已改变位姿，用当前状态刷新接触点与法线（论文 §3.5 的核心思想）
        this.update();

        const A = this.bodyA;
        const B = this.bodyB;

        const vA = A ? A.getVelocityAt(this.p1) : ZERO;
        const vB = B ? B.getVelocityAt(this.p2) : ZERO;
        const vRel = TMP_V.subVectors(vA, vB);
        const vn = vRel.dot(this.n);
        const vt = TMP_VT.copy(vRel).addScaledVector(this.n, -vn);

        // v̄n：用**速度更新前**的速度快照计算（论文 §3.6 恢复系数一节）
        const vAp = A ? A.getVelocityAtPrev(this.p1) : ZERO;
        const vBp = B ? B.getVelocityAtPrev(this.p2) : ZERO;
        const vnPrev = TMP_VP.subVectors(vAp, vBp).dot(this.n);

        // --- 动摩擦（论文 Eq. 31）---
        const vtMag = vt.length();
        if (vtMag > EPS && this.dynamicFriction > 0.0) {
            const fn = this.lambdaN / (h * h);           // 论文 Eq. 11：法向力
            const dv = Math.min(h * this.dynamicFriction * fn, vtMag);
            const dvVec = vt.multiplyScalar(-dv / vtMag);
            applyBodyPairCorrection(A, B, dvVec, 0.0, h, this.p1, this.p2, null, null, true);
        }

        // --- 法向速度重设 + 恢复系数（论文 Eq. 35）---
        // Δv ← n(−vn + max(−e·v̄n, 0))
        //
        // 这一步**无条件**执行（不做 vn < 0 的判断），这正是论文图 13 的关键：
        // PBD 的位置投影会把“穿透深度”当成速度导出（v = d/h），初始穿透或落地瞬间
        // 会得到 d/h 量级的巨大分离速度，把物体弹飞。Eq. 35 先用 −vn 把当前法向速度
        // 整个抹掉，再补上反射速度 −e·v̄n，于是重叠物体（v̄n = 0）的法向速度归零，
        // 被“温和地”推出去。
        //
        // 抖振抑制：|v̄n| ≤ 2|g|h 时置 e = 0——该阈值等于两倍重力在预测步里
        // 增加的速度，静止接触因此不会产生虚假弹跳（论文 §3.6）。
        let e = this.restitution;
        if (Math.abs(vnPrev) <= 2.0 * gravityMagnitude * h) e = 0.0;
        const dvn = -vn + Math.max(-e * vnPrev, 0.0);
        if (Math.abs(dvn) > EPS) {
            const dvVec = this.n.clone().multiplyScalar(dvn);
            applyBodyPairCorrection(A, B, dvVec, 0.0, h, this.p1, this.p2, null, null, true);
        }
    }
}

const TMP_BA = new THREE.Vector3();
const TMP_AB = new THREE.Vector3();
const TMP_E1 = new THREE.Vector3();
const TMP_E2 = new THREE.Vector3();
const TMP_W = new THREE.Vector3();
const TMP_D = new THREE.Vector3();
const TMP_C = new THREE.Vector3();
const TMP_DP = new THREE.Vector3();
const TMP_DP2 = new THREE.Vector3();
const TMP_V = new THREE.Vector3();
const TMP_VT = new THREE.Vector3();
const TMP_VP = new THREE.Vector3();

// ===========================================================================
// 窄相碰撞检测
// ===========================================================================

/**
 * 对一对刚体做窄相检测，返回接触描述数组（最多 4 个接触点）。
 * 支持：平面 vs（球/盒/圆柱/胶囊）、球-球、球-胶囊、盒-盒（SAT+裁剪）。
 */
export function collide(bodyA, bodyB) {
    const ta = bodyA.shape.type;
    const tb = bodyB.shape.type;

    if (ta === ShapeType.PLANE && tb === ShapeType.PLANE) return [];
    // 平面统一作为 bodyB（仅支持静态平面）
    if (ta === ShapeType.PLANE) {
        const list = collideWithPlane(bodyB, bodyA); // 形状体 vs 平面
        // 交换角色：新的 A = 平面、B = 形状，法线须由 B 指向 A，故取反
        for (const c of list) {
            c.normalData = c.normalData.clone().negate();
            const t = c.localAnchorA; c.localAnchorA = c.localAnchorB; c.localAnchorB = t;
        }
        return list;
    }
    if (tb === ShapeType.PLANE) return collideWithPlane(bodyA, bodyB);

    if (ta === ShapeType.SPHERE && tb === ShapeType.SPHERE) return collideSphereSphere(bodyA, bodyB);
    if (ta === ShapeType.SPHERE && tb === ShapeType.CAPSULE) return collideSphereCapsule(bodyA, bodyB, true);
    if (ta === ShapeType.CAPSULE && tb === ShapeType.SPHERE) return collideSphereCapsule(bodyB, bodyA, false);
    if (ta === ShapeType.BOX && tb === ShapeType.BOX) return collideBoxBox(bodyA, bodyB);
    return [];
}

// ---------------------------------------------------------------------------
// 平面接触（球 / 盒 / 圆柱 / 胶囊）
// 约定：bodyA 为形状体，planeBody 为平面（静态）。
// 返回的描述中 A = 形状体、B = 平面，法线 = 平面法线（由平面指向形状体）。
// ---------------------------------------------------------------------------
function collideWithPlane(shapeBody, planeBody) {
    const s = shapeBody.shape;
    const plane = planeBody.shape;
    const normal = plane.normal; // 世界系（平面必须静态）
    const p0 = TMP_P0.copy(normal).multiplyScalar(plane.offset);
    const out = [];

    // worldP1 为形状体表面上的接触点，penetration 为其陷入平面的深度；
    // 平面上的对应点即由 worldP1 沿法线抬升 penetration（论文 Eq. 27 的 p1、p2）。
    const pushContact = (worldP1, penetration) => {
        if (penetration <= 0) return;
        const p2 = TMP_P2.copy(worldP1).addScaledVector(normal, penetration);
        out.push(makeDesc('fixed', normal.clone(),
            shapeBody.toLocal(worldP1), planeBody.toLocal(p2)));
    };

    switch (s.type) {
        case ShapeType.SPHERE: {
            const c = shapeBody.pose.p;
            const dist = TMP_T1.subVectors(c, p0).dot(normal);
            const pen = s.radius - dist;             // 球心到平面距离 < r 即穿透
            if (pen <= 0) return out;
            pushContact(TMP_T3.copy(c).addScaledVector(normal, -s.radius), pen);
            break;
        }
        case ShapeType.BOX: {
            const half = s.halfExtents;
            const scored = [];
            for (let i = 0; i < 8; i++) {
                const local = new THREE.Vector3(
                    (i & 1) ? half.x : -half.x,
                    (i & 2) ? half.y : -half.y,
                    (i & 4) ? half.z : -half.z);
                const w = local.clone().applyQuaternion(shapeBody.pose.q).add(shapeBody.pose.p);
                const pen = -TMP_T1.subVectors(w, p0).dot(normal);
                if (pen > 0) scored.push({ world: w, pen });
            }
            scored.sort((a, b) => b.pen - a.pen); // 最深的在前
            for (let i = 0; i < Math.min(4, scored.length); i++) {
                pushContact(scored[i].world, scored[i].pen);
            }
            break;
        }
        case ShapeType.CYLINDER: {
            // 两端面圆环采样：既能表现平放（多点稳定），也能表现侧立滚动
            const N = 8;
            const r = s.radius;
            const hy = 0.5 * s.height;
            const scored = [];
            for (let k = 0; k < N; k++) {
                const th = (2.0 * Math.PI * k) / N;
                for (const y of [hy, -hy]) {
                    const local = new THREE.Vector3(r * Math.cos(th), y, r * Math.sin(th));
                    const w = local.clone().applyQuaternion(shapeBody.pose.q).add(shapeBody.pose.p);
                    const pen = -TMP_T1.subVectors(w, p0).dot(normal);
                    if (pen > 0) scored.push({ world: w, pen });
                }
            }
            scored.sort((a, b) => b.pen - a.pen);
            for (let i = 0; i < Math.min(4, scored.length); i++) {
                pushContact(scored[i].world, scored[i].pen);
            }
            break;
        }
        case ShapeType.CAPSULE: {
            // 取两半球心处穿透更深的一侧
            const axis = TMP_T2.copy(AXIS_Y).applyQuaternion(shapeBody.pose.q);
            const half = 0.5 * s.length;
            let best = null;
            for (const t of [-half, half]) {
                const c = shapeBody.pose.p.clone().addScaledVector(axis, t);
                const dist = TMP_T1.subVectors(c, p0).dot(normal);
                const pen = s.radius - dist;
                if (pen > 0 && (!best || pen > best.pen)) best = { c, pen };
            }
            if (best) pushContact(TMP_T3.copy(best.c).addScaledVector(normal, -s.radius), best.pen);
            break;
        }
        default:
            break;
    }
    return out;
}

// ---------------------------------------------------------------------------
// 球-球
// ---------------------------------------------------------------------------
function collideSphereSphere(bodyA, bodyB) {
    const n = new THREE.Vector3().subVectors(bodyA.pose.p, bodyB.pose.p);
    const dist = n.length();
    if (bodyA.shape.radius + bodyB.shape.radius - dist <= 0) return [];
    if (dist < EPS) n.set(0, 1, 0);
    else n.multiplyScalar(1.0 / dist);

    const p1 = bodyA.pose.p.clone().addScaledVector(n, -bodyA.shape.radius);
    const p2 = bodyB.pose.p.clone().addScaledVector(n, bodyB.shape.radius);
    return [makeDesc('centers', null, bodyA.toLocal(p1), bodyB.toLocal(p2))];
}

// ---------------------------------------------------------------------------
// 球-胶囊（胶囊也用作静态轨道线段）。
// sphereIsA 表示在返回的接触描述中球是 bodyA（否则球是 bodyB）。
// ---------------------------------------------------------------------------
function collideSphereCapsule(sphereBody, capsuleBody, sphereIsA) {
    const sc = sphereBody.shape;
    const cc = capsuleBody.shape;
    const axisLocal = AXIS_Y.clone();
    const axis = axisLocal.clone().applyQuaternion(capsuleBody.pose.q);
    const half = 0.5 * cc.length;
    const toSphere = new THREE.Vector3().subVectors(sphereBody.pose.p, capsuleBody.pose.p);
    const t = THREE.MathUtils.clamp(toSphere.dot(axis), -half, half);
    const closest = capsuleBody.pose.p.clone().addScaledVector(axis, t);

    const n = new THREE.Vector3().subVectors(sphereBody.pose.p, closest);
    const dist = n.length();
    if (sc.radius + cc.radius - dist <= 0) return [];
    if (dist < EPS) n.set(0, 1, 0);
    else n.multiplyScalar(1.0 / dist);

    const p1 = sphereBody.pose.p.clone().addScaledVector(n, -sc.radius);
    const p2 = closest.clone().addScaledVector(n, cc.radius);

    const data = {
        sphere: sphereBody,
        capsule: capsuleBody,
        axisLocal,
        halfLength: half,
        sphereIsA,
    };
    // 描述中的 A/B 顺序按调用方要求确定
    return sphereIsA
        ? [makeDesc('sphereSegment', data, sphereBody.toLocal(p1), capsuleBody.toLocal(p2))]
        : [makeDesc('sphereSegment', data, capsuleBody.toLocal(p2), sphereBody.toLocal(p1))];
}

// ---------------------------------------------------------------------------
// 盒-盒：分离轴定理（SAT）+ 参考面裁剪
//
// 15 条候选轴：A 的 3 个面法线、B 的 3 个面法线、9 组棱叉积。
// 取穿透最小的轴作为分离方向；面轴 → 参考面裁剪得到接触多边形（最多 4 点）；
// 棱轴 → 两棱最近点构成单点接触。
// ---------------------------------------------------------------------------
function collideBoxBox(bodyA, bodyB) {
    const ha = bodyA.shape.halfExtents;
    const hb = bodyB.shape.halfExtents;
    const AR = worldAxes(bodyA);
    const BR = worldAxes(bodyB);
    const hA = [ha.x, ha.y, ha.z];
    const hB = [hb.x, hb.y, hb.z];

    // R[i][j] = A_i · B_j 的绝对值（投影矩阵）
    const absR = [];
    for (let i = 0; i < 3; i++) {
        absR.push([]);
        for (let j = 0; j < 3; j++) absR[i].push(Math.abs(AR[i].dot(BR[j])));
    }

    const dp = new THREE.Vector3().subVectors(bodyB.pose.p, bodyA.pose.p);

    let bestOverlap = Infinity;
    let bestType = 'face';
    let bestIndex = 0;   // 面：0-2 → A 的轴，3-5 → B 的轴；棱：i*3+j
    let bestAxis = null;
    let bestSign = 1;

    // --- A 的面法线 ---
    for (let i = 0; i < 3; i++) {
        const proj = Math.abs(dp.dot(AR[i]));
        const rb = hB[0] * absR[i][0] + hB[1] * absR[i][1] + hB[2] * absR[i][2];
        const overlap = hA[i] + rb - proj;
        if (overlap <= 0) return [];
        if (overlap < bestOverlap) {
            bestOverlap = overlap; bestType = 'face'; bestIndex = i;
            bestAxis = AR[i].clone(); bestSign = dp.dot(AR[i]) < 0 ? 1 : -1;
        }
    }
    // --- B 的面法线 ---
    for (let j = 0; j < 3; j++) {
        const proj = Math.abs(dp.dot(BR[j]));
        const ra = hA[0] * absR[0][j] + hA[1] * absR[1][j] + hA[2] * absR[2][j];
        const overlap = ra + hB[j] - proj;
        if (overlap <= 0) return [];
        if (overlap < bestOverlap) {
            bestOverlap = overlap; bestType = 'face'; bestIndex = 3 + j;
            bestAxis = BR[j].clone(); bestSign = dp.dot(BR[j]) < 0 ? 1 : -1;
        }
    }
    // --- 9 组棱叉积 ---
    // 注意：两条棱接近平行时叉积轴会退化（例如轴对齐盒子下 X_A × X_B ≈ Y），
    // 此时不能用“未归一化轴”的半径公式（两棱的 (i+2)/(i+1) 索引公式隐含未归一化轴），
    // 否则会算出错误的负 overlap 而误判为分离。这里统一用**归一化轴 + 真实投影半径**，
    // 退化的棱轴会与对应的面轴给出相同的 overlap，从而不会被选中（严格小于比较）。
    for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
            const axis = new THREE.Vector3().crossVectors(AR[i], BR[j]);
            const len = axis.length();
            if (len < 1e-8) continue; // 两棱平行，叉积为零向量
            axis.multiplyScalar(1.0 / len);
            // 退化保护：叉积轴若与某个面法线平行，该方向已由对应的面测试精确覆盖，
            // 跳过以免与面测试产生重复且数值更差的候选（ODE/Bullet 的常规做法）。
            if (isParallelToAnyAxis(axis, AR, i) || isParallelToAnyAxis(axis, BR, j)) continue;
            const proj = Math.abs(dp.dot(axis));
            const ra = projectionRadius(hA, AR, axis);
            const rb = projectionRadius(hB, BR, axis);
            const overlap = ra + rb - proj;
            if (overlap <= 0) return [];
            if (overlap < bestOverlap) {
                bestOverlap = overlap; bestType = 'edge'; bestIndex = i * 3 + j;
                bestAxis = axis; bestSign = dp.dot(axis) < 0 ? 1 : -1;
            }
        }
    }

    return bestType === 'face'
        ? boxFaceContacts(bodyA, bodyB, bestIndex)
        : boxEdgeContacts(bodyA, bodyB, Math.floor(bestIndex / 3), bestIndex % 3, bestOverlap);
}

/** 面接触：以穿透最小的面为参考面，裁剪另一盒的入射面 */
function boxFaceContacts(bodyA, bodyB, faceIndex) {
    const refIsA = faceIndex < 3;
    const refBody = refIsA ? bodyA : bodyB;
    const incBody = refIsA ? bodyB : bodyA;
    const refAxisIndex = refIsA ? faceIndex : faceIndex - 3;

    const refAxes = worldAxes(refBody);
    const incAxes = worldAxes(incBody);
    const refH = halfArray(refBody);
    const incH = halfArray(incBody);

    // 参考面法线：由参考盒指向入射盒
    const refNormal = refAxes[refAxisIndex].clone();
    const toInc = new THREE.Vector3().subVectors(incBody.pose.p, refBody.pose.p);
    if (refNormal.dot(toInc) < 0) refNormal.negate();

    // 参考面上一点
    const refSign = refNormal.dot(refAxes[refAxisIndex]) > 0 ? 1 : -1;
    const refFacePoint = refBody.pose.p.clone()
        .addScaledVector(refAxes[refAxisIndex], refH[refAxisIndex] * refSign);

    // 入射面：入射盒中“外法线与参考法线最反向”的那个面。
    // 注意必须按 |dot| 取最大（而非取 dot 最小）：当面法线与参考法线垂直时
    // （dot = 0，例如两轴对齐盒子）取最小会误选到侧面，导致接触完全丢失。
    let incAxisIndex = 0;
    let maxAbsDot = -1.0;
    let incFaceSign = 1;
    for (let i = 0; i < 3; i++) {
        const dot = incAxes[i].dot(refNormal);
        if (Math.abs(dot) > maxAbsDot) {
            maxAbsDot = Math.abs(dot);
            incAxisIndex = i;
            incFaceSign = dot < 0 ? 1 : -1; // 取与 refNormal 反向的那一面
        }
    }

    // 入射面的 4 个顶点（世界系）
    let poly = faceVertices(incBody, incAxes, incH, incAxisIndex, incFaceSign);

    // 用参考面的两个侧向平面裁剪
    for (const si of [(refAxisIndex + 1) % 3, (refAxisIndex + 2) % 3]) {
        const axis = refAxes[si];
        const half = refH[si];
        const c = refBody.pose.p.dot(axis);
        poly = clipPolygon(poly, axis, c + half, true);
        poly = clipPolygon(poly, axis, c - half, false);
        if (poly.length === 0) break;
    }

    const out = [];
    const kRef = refIsA ? -1 : 1; // 使存储的法线方向 = 由 bodyB 指向 bodyA
    const sign = kRef * (refNormal.dot(refAxes[refAxisIndex]) > 0 ? 1 : -1);
    const cand = [];
    for (const w of poly) {
        const dist = refNormal.dot(TMP_W.subVectors(w, refFacePoint));
        const pen = -dist;
        if (pen <= 0) continue; // 未穿透
        const pOnInc = w.clone();
        const pOnRef = w.clone().addScaledVector(refNormal, -dist);
        // bodyA / bodyB 侧各自的接触点
        const anchorA = refIsA ? pOnRef : pOnInc;
        const anchorB = refIsA ? pOnInc : pOnRef;
        // 裁剪会产生重合顶点，先在世界系去重（局部锚点分属两个体，不能直接比较）
        if (cand.some((c) => c.anchorA.distanceToSquared(anchorA) < 1e-10)) continue;
        cand.push({ anchorA, anchorB, pen });
    }
    // 每对刚体最多保留 4 个（最深的）接触点，避免过度约束与重复求解
    cand.sort((a, b) => b.pen - a.pen);
    for (let i = 0; i < Math.min(4, cand.length); i++) {
        out.push(makeDesc('face',
            { onB: !refIsA, axisIndex: refAxisIndex, sign: sign },
            bodyA.toLocal(cand[i].anchorA), bodyB.toLocal(cand[i].anchorB)));
    }
    return out;
}

/**
 * 棱-棱接触（论文 §3.5：这类接触在堆叠中被面接触覆盖，仅在真正棱交叉时出现）。
 *
 * 法线取 SAT 得到的棱叉积轴（最小平移轴），而非两棱最近点连线——
 * 两棱交叉时最近点重合，连线为零向量，无法定出法线。
 * 接触点取两棱最近点的中点，并在 bodyB 侧锚点沿 n 偏移 penetration，
 * 使得由锚点算出的 d = (p2 − p1)·n 恒等于 SAT 的穿透量。
 */
function boxEdgeContacts(bodyA, bodyB, i, j, penetration) {
    const axesA = worldAxes(bodyA);
    const axesB = worldAxes(bodyB);
    const ea = edgeEndpoints(bodyA, i);
    const eb = edgeEndpoints(bodyB, j);
    const c1 = new THREE.Vector3();
    const c2 = new THREE.Vector3();
    closestPointsSegments(ea[0], ea[1], eb[0], eb[1], c1, c2);

    // 棱叉积轴（由 B 指向 A）
    const n = new THREE.Vector3().crossVectors(axesA[i], axesB[j]);
    if (n.lengthSq() < 1e-12) n.subVectors(bodyA.pose.p, bodyB.pose.p);
    n.normalize();
    if (n.dot(TMP_AB.subVectors(bodyA.pose.p, bodyB.pose.p)) < 0) n.negate();

    const c = c1.clone().add(c2).multiplyScalar(0.5);
    const p2 = c.clone().addScaledVector(n, penetration);

    return [makeDesc('edges', {
        // 存储局部棱方向，供每次投影前用当前姿态重算法线（论文 §3.5 的核心）
        dirA: bodyA.toLocal(ea[1]).sub(bodyA.toLocal(ea[0])).normalize(),
        dirB: bodyB.toLocal(eb[1]).sub(bodyB.toLocal(eb[0])).normalize(),
        fallback: n.clone(),
    }, bodyA.toLocal(c), bodyB.toLocal(p2))];
}

// ---------------------------------------------------------------------------
// 几何辅助
// ---------------------------------------------------------------------------

function halfArray(body) {
    const h = body.shape.halfExtents;
    return [h.x, h.y, h.z];
}

/** u 是否（近乎）平行于 axes 中除 excludeIndex 之外的某条轴 */
function isParallelToAnyAxis(u, axes, excludeIndex, eps = 1e-6) {
    for (let k = 0; k < 3; k++) {
        if (k === excludeIndex) continue;
        if (Math.abs(u.dot(axes[k])) > 1.0 - eps) return true;
    }
    return false;
}

/** 盒在**单位**方向 u 上的投影半径 Σᵢ hᵢ·|u·Aᵢ|（对任意 u 都成立，包括退化轴） */
function projectionRadius(h, axes, u) {
    return h[0] * Math.abs(u.dot(axes[0]))
        + h[1] * Math.abs(u.dot(axes[1]))
        + h[2] * Math.abs(u.dot(axes[2]));
}

/** 盒的世界系三条轴 */
function worldAxes(body) {
    return [
        AXIS_X.clone().applyQuaternion(body.pose.q),
        AXIS_Y.clone().applyQuaternion(body.pose.q),
        AXIS_Z.clone().applyQuaternion(body.pose.q),
    ];
}

/** 盒的某条棱的两个端点（世界系）；axisIndex 0/1/2 表示沿 X/Y/Z 的棱 */
function edgeEndpoints(body, axisIndex) {
    const hs = halfArray(body);
    const axes = worldAxes(body);
    const a = axisIndex;
    const b = (axisIndex + 1) % 3;
    const c = (axisIndex + 2) % 3;
    const mk = (sb, sc) => body.pose.p.clone()
        .addScaledVector(axes[a], hs[a])
        .addScaledVector(axes[b], hs[b] * sb)
        .addScaledVector(axes[c], hs[c] * sc);
    return [mk(1, 1), mk(-1, -1)];
}

/** 盒的某个面的四个顶点（世界系） */
function faceVertices(body, axes, hs, axisIndex, faceSign) {
    const a = axisIndex;
    const b = (axisIndex + 1) % 3;
    const c = (axisIndex + 2) % 3;
    const center = body.pose.p.clone().addScaledVector(axes[a], hs[a] * faceSign);
    const pts = [];
    for (const [sb, sc] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) {
        pts.push(center.clone()
            .addScaledVector(axes[b], hs[b] * sb)
            .addScaledVector(axes[c], hs[c] * sc));
    }
    return pts;
}

/** 用平面 axis·x = c 裁剪多边形；keepLess 为 true 时保留 axis·x ≤ c 的一侧 */
function clipPolygon(poly, axis, c, keepLess) {
    const out = [];
    const inside = (p) => {
        const d = axis.dot(p) - c;
        return keepLess ? d <= 1e-9 : d >= -1e-9;
    };
    const n = poly.length;
    for (let i = 0; i < n; i++) {
        const a = poly[i];
        const b = poly[(i + 1) % n];
        const da = axis.dot(a) - c;
        const db = axis.dot(b) - c;
        const ia = inside(a);
        const ib = inside(b);
        if (ia) out.push(a.clone());
        if (ia !== ib) {
            const t = da / (da - db);
            out.push(a.clone().lerp(b, t));
        }
    }
    return out;
}

/** 两条线段的最近点（Ericson, Real-Time Collision Detection §5.1.9） */
export function closestPointsSegments(p1, q1, p2, q2, c1, c2) {
    const d1 = new THREE.Vector3().subVectors(q1, p1);
    const d2 = new THREE.Vector3().subVectors(q2, p2);
    const r = new THREE.Vector3().subVectors(p1, p2);
    const a = d1.dot(d1);
    const e = d2.dot(d2);
    const f = d2.dot(r);
    let s, t;
    if (a <= EPS && e <= EPS) { c1.copy(p1); c2.copy(p2); return; }
    if (a <= EPS) { s = 0; t = THREE.MathUtils.clamp(f / e, 0, 1); }
    else {
        const cc = d1.dot(r);
        if (e <= EPS) { t = 0; s = THREE.MathUtils.clamp(-cc / a, 0, 1); }
        else {
            const b = d1.dot(d2);
            const denom = a * e - b * b;
            s = denom !== 0 ? THREE.MathUtils.clamp((b * f - cc * e) / denom, 0, 1) : 0;
            t = (b * s + f) / e;
            if (t < 0) { t = 0; s = THREE.MathUtils.clamp(-cc / a, 0, 1); }
            else if (t > 1) { t = 1; s = THREE.MathUtils.clamp((b - cc) / a, 0, 1); }
        }
    }
    c1.copy(p1).addScaledVector(d1, s);
    c2.copy(p2).addScaledVector(d2, t);
}

/** 构造接触描述 */
function makeDesc(normalType, normalData, localAnchorA, localAnchorB) {
    return { normalType, normalData, localAnchorA, localAnchorB };
}

const TMP_P0 = new THREE.Vector3();
const TMP_T1 = new THREE.Vector3();
const TMP_T2 = new THREE.Vector3();
const TMP_T3 = new THREE.Vector3();
const TMP_P2 = new THREE.Vector3();

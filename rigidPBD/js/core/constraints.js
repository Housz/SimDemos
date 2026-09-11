// ============================================================================
// constraints.js —— 两个基本投影操作（论文 §3.3「Core Projection Operations」）
//
// 论文的核心结论：**任意**关节、接触、刚柔耦合都可以只由两个基本操作构建：
//
//   1. 位置修正（§3.3.1, Eqs. 2-11）——
//      在刚体上的两点 r1、r2 处施加位置修正量 Δx，同时改变质心位置与朝向，
//      改变量按广义逆质量分配，以守恒线动量与角动量。
//
//   2. 角度修正（§3.3.2, Eqs. 12-18）——
//      在两刚体间施加旋转向量修正 Δq（轴 n × 角 θ），只改变朝向，
//      按逆惯性张量分配，以守恒角动量。
//
// 两者共用同一套 XPBD 乘子更新（论文 Eqs. 4-5 与 14-15 完全同构）：
//
//        Δλ = (−C − α̃·λ) / (w1 + w2 + α̃),      α̃ = α / h²
//        λ ← λ + Δλ
//
// 其中 C 为约束函数值（位置约束取距离，角度约束取夹角），
// α 为柔度（compliance，单位 m/N，是刚度的倒数）。
// α = 0 表示无限硬约束（此时 XPBD 退化为原始 PBD）。
//
// 求解后的约束力/力矩可导出（论文 Eqs. 11、18）：
//        f = λ·n / h²        τ = λ·n / h²
// ============================================================================

import * as THREE from 'three';

/**
 * 论文 Eqs. (2)-(10)：对一对刚体施加位置/角度修正。
 *
 * 语义：corr 是“希望的修正向量”（方向 n̂、模长 C），
 *       bodyA 沿 +n̂ 移动，bodyB 沿 −n̂ 移动（论文 Eqs. 6-7）。
 *
 * @param {RigidBody|null} bodyA       刚体 A（可为 null 表示与静态世界连接）
 * @param {RigidBody|null} bodyB       刚体 B
 * @param {THREE.Vector3} corr         世界系修正向量（原位被修改，请传副本）
 * @param {number} compliance          柔度 α（m/N）
 * @param {number} h                   子步长
 * @param {THREE.Vector3|null} posA    作用点（世界系）；null 表示纯角度修正
 * @param {THREE.Vector3|null} posB
 * @param {object|null} lambdaStore    λ 累加器（每个约束一个，子步开始时清零）
 * @param {string|null} key            λ 在 lambdaStore 中的键
 * @param {boolean} velocityLevel      是否作用于速度层（论文 §3.6, Eq. 34）
 * @returns {number} 本约束当前的 λ（符号为负；力的大小取 |λ|/h²）
 */
export function applyBodyPairCorrection(
    bodyA, bodyB, corr, compliance, h,
    posA = null, posB = null, lambdaStore = null, key = null, velocityLevel = false
) {
    const C = corr.length();
    if (C === 0.0) return 0.0;

    const n = corr.clone().multiplyScalar(1.0 / C); // 单位方向

    // 广义逆质量（论文 Eqs. 2、3、12、13）
    const rA = (posA && bodyA) ? new THREE.Vector3().subVectors(posA, bodyA.pose.p) : null;
    const rB = (posB && bodyB) ? new THREE.Vector3().subVectors(posB, bodyB.pose.p) : null;
    const wA = bodyA ? bodyA.getGeneralizedInvMass(n, rA) : 0.0;
    const wB = bodyB ? bodyB.getGeneralizedInvMass(n, rB) : 0.0;
    const w = wA + wB;
    if (w === 0.0) return 0.0; // 两侧都是静态体

    // XPBD 乘子更新（论文 Eqs. 4、14）
    const alphaTilde = compliance / (h * h);
    const lambdaPrev = (lambdaStore && key !== null) ? (lambdaStore[key] || 0.0) : 0.0;
    const dLambda = (-C - alphaTilde * lambdaPrev) / (w + alphaTilde);
    const lambda = lambdaPrev + dLambda;
    if (lambdaStore && key !== null) lambdaStore[key] = lambda;

    // 位置冲量 p = Δλ·n 的反向施加：bodyA 得到 −Δλ·n（沿 +n̂），bodyB 得到相反量
    const p = n.multiplyScalar(-dLambda);
    if (bodyA) bodyA.applyCorrection(p, rA, velocityLevel);
    if (bodyB) {
        p.multiplyScalar(-1.0);
        bodyB.applyCorrection(p, rB, velocityLevel);
    }
    return lambda;
}

/**
 * 论文 §3.3.2 + Algorithm 2：纯角度修正（Eqs. 16-17）。
 * rotVec 为世界系旋转向量（旋转轴 × 旋转角），即位姿修正量。
 */
export function applyAngularCorrection(
    bodyA, bodyB, rotVec, compliance, h, lambdaStore = null, key = null
) {
    return applyBodyPairCorrection(bodyA, bodyB, rotVec, compliance, h, null, null, lambdaStore, key, false);
}

/**
 * 论文 Algorithm 3「Handling joint angle limits」。
 *
 * 把两刚体上轴 n1、n2 之间的夹角限制在 [minAngle, maxAngle]，公共旋转轴为 n：
 *
 *   φ ← arcsin((n1 × n2)·n)
 *   if n1·n2 < 0 then φ ← 2π − φ
 *   把 φ 规约到 (−π, π]
 *   若 φ 越界：φ ← clamp(φ, α, β);  n1 ← rot(n, φ)·n1;  施加 Δq_limit = n1 × n2
 *
 * （PBD.js 使用等价的 π − φ 形式；两者在规约到 (−π,π] 后一致。）
 *
 * @param {THREE.Vector3} n   公共旋转轴（单位向量）
 * @param {THREE.Vector3} n1  刚体 A 上的轴
 * @param {THREE.Vector3} n2  刚体 B 上的轴
 * @param {number} maxCorr    单次修正的最大角度（用于万向节死锁时避免爆炸）
 */
export function limitAngle(
    bodyA, bodyB, n, n1, n2, minAngle, maxAngle, compliance, h,
    lambdaStore = null, key = null, maxCorr = Math.PI
) {
    const c = new THREE.Vector3().crossVectors(n1, n2);
    let phi = Math.asin(THREE.MathUtils.clamp(c.dot(n), -1.0, 1.0));
    if (n1.dot(n2) < 0.0) phi = Math.PI - phi;

    // 规约到 (−π, π]
    phi = wrapPi(phi);

    if (phi < minAngle || phi > maxAngle) {
        phi = THREE.MathUtils.clamp(phi, minAngle, maxAngle);

        // n1 ← rot(n, φ) · n1
        const q = new THREE.Quaternion().setFromAxisAngle(n, phi);
        const n1Rot = n1.clone().applyQuaternion(q);

        // Δq_limit = n1 × n2
        const corr = new THREE.Vector3().crossVectors(n1Rot, n2);
        clampVectorLength(corr, maxCorr);
        applyAngularCorrection(bodyA, bodyB, corr, compliance, h, lambdaStore, key);
    }
}

// ---------------------------------------------------------------------------
// 小工具
// ---------------------------------------------------------------------------

/** 把角度规约到 (−π, π] */
export function wrapPi(angle) {
    while (angle > Math.PI) angle -= 2.0 * Math.PI;
    while (angle < -Math.PI) angle += 2.0 * Math.PI;
    return angle;
}

/** 就地限长（用于万向节死锁保护） */
export function clampVectorLength(v, maxLen) {
    const len = v.length();
    if (len > maxLen && len > 0) v.multiplyScalar(maxLen / len);
    return v;
}

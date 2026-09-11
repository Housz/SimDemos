// ============================================================================
// math3d.js —— 三维数学工具（基于 three.js）
//
// 论文：《Detailed Rigid Body Simulation with Extended Position Based Dynamics》
//       M. Müller, M. Macklin, N. Chentanez, S. Jeschke, T.-Y. Kim
//       SCA 2020, Computer Graphics Forum 39(8)
//
// 本文件只做两件事：
//   1. 提供 Pose（位置 + 姿态）容器与局部/世界坐标变换（沿用 PBDRigidBody/PBD.js
//      中经过验证的约定，便于与论文公式对照）；
//   2. 提供四元数的线性化更新与体坐标轴提取（论文 §3.2 与 Eqs. 8-9、16-17）。
//
// 约定（全文一致）：
//   - 四元数 q 表示“体局部 → 世界”的旋转：v_world = q · v_local
//   - rotate(v)   : v ← q · v      （局部 → 世界）
//   - invRotate(v): v ← q⁻¹ · v    （世界 → 局部）
//   - 角速度 ω、力矩 τ 均定义在**世界系**：q̇ = ½ [ω, 0] q
//   - 惯性张量在体的静止（rest）系下为对角阵，用 Vector3 存储其对角线元素
// ============================================================================

import * as THREE from 'three';

// 单个子步内允许的最大旋转增量（弧度）。论文并未设置该钳制，
// 但极端情况下（例如约束冲突）线性化四元数更新会产生过大的旋转，
// 这里沿用 PBD.js 的安全钳制值。
export const MAX_ROTATION_PER_SUBSTEP = 0.5;

/** applyRotationExact 用的临时四元数（避免每子步分配） */
const TMP_Q_EXP = new THREE.Quaternion();

// ---------------------------------------------------------------------------
// Pose：刚体的位姿（位置 p + 姿态 q）
// ---------------------------------------------------------------------------
export class Pose {
    constructor(p = null, q = null) {
        this.p = p ? p.clone() : new THREE.Vector3(0, 0, 0);
        this.q = q ? q.clone() : new THREE.Quaternion(0, 0, 0, 1);
    }

    copy(pose) {
        this.p.copy(pose.p);
        this.q.copy(pose.q);
        return this;
    }

    clone() {
        return new Pose(this.p, this.q);
    }

    /** 世界向量 → 世界向量：绕 q 正向旋转（局部 → 世界） */
    rotate(v) {
        return v.applyQuaternion(this.q);
    }

    /** 世界向量 → 局部向量（世界 → 局部） */
    invRotate(v) {
        const inv = conj(this.q);
        return v.applyQuaternion(inv);
    }

    /** 局部点 → 世界点 */
    transform(v) {
        v.applyQuaternion(this.q);
        v.add(this.p);
        return v;
    }

    /** 世界点 → 局部点 */
    invTransform(v) {
        v.sub(this.p);
        return this.invRotate(v);
    }

    /**
     * this ∘ pose：把子位姿 pose 从 this 的局部系变换到世界系。
     * 注意：pose 被就地修改（与 PBD.js 一致）。
     */
    transformPose(pose) {
        pose.q.multiplyQuaternions(this.q, pose.q);
        this.rotate(pose.p);
        pose.p.add(this.p);
        return pose;
    }
}

// ---------------------------------------------------------------------------
// 四元数辅助
// ---------------------------------------------------------------------------

/** 共轭（单位四元数的逆），返回新对象，避免依赖 three.js 的版本差异 */
export function conj(q) {
    return new THREE.Quaternion(-q.x, -q.y, -q.z, q.w);
}

/** 体局部坐标轴（X/Y/Z）在世界系下的方向：旋转矩阵 R 的第 i 列 */
export function getQuatAxis(q, i) {
    const v = new THREE.Vector3(i === 0 ? 1 : 0, i === 1 ? 1 : 0, i === 2 ? 1 : 0);
    return v.applyQuaternion(q);
}

export const getAxisX = (q) => getQuatAxis(q, 0);
export const getAxisY = (q) => getQuatAxis(q, 1);
export const getAxisZ = (q) => getQuatAxis(q, 2);

/**
 * 论文 Algorithm 2 的四元数积分 / 论文 Eqs. (8)(9)(16)(17) 的朝向修正：
 *
 *   q ← q + ½ [ω, 0] q ,  然后归一化
 *
 * 其中 ω 为**世界系**旋转向量（角速度 × dt 或角冲量）。
 * 这是论文所说“快速、鲁棒且适合子步化小时间步”的线性化公式。
 *
 * 当旋转量过大时按 MAX_ROTATION_PER_SUBSTEP 缩放（安全钳制）。
 */
export function applyRotationalCorrection(pose, rot, scale = 1.0) {
    let phi = rot.length();
    if (phi === 0) return;
    if (phi * scale > MAX_ROTATION_PER_SUBSTEP) {
        scale = MAX_ROTATION_PER_SUBSTEP / phi;
    }
    const dq = new THREE.Quaternion(rot.x * scale, rot.y * scale, rot.z * scale, 0.0);
    dq.multiply(pose.q); // dq = [ω,0] · q  （three.js 的 multiply 为 this = this * arg）
    pose.q.set(
        pose.q.x + 0.5 * dq.x,
        pose.q.y + 0.5 * dq.y,
        pose.q.z + 0.5 * dq.z,
        pose.q.w + 0.5 * dq.w
    );
    pose.q.normalize();
}

/**
 * 自由旋转的**精确**指数映射积分：q ← q · exp(½ [ω,0] h)。
 *
 * 与上面的 applyRotationalCorrection 分工不同，别混用：
 *   · 位置层的约束投影（applyRotationalCorrection）**必须**用一阶线性化形式，
 *     XPBD 的推导（Eqs. 15-17 的广义逆质量、Gauss-Seidel 的线性叠加）建立在
 *     「修正量是约束误差的线性函数」之上，换成指数映射会破坏这个前提；
 *   · 这里做的是**时间积分**（把 ω 沿时间推进一个 h），可以也应该精确求解——
 *     平动那侧 x ← x + h·v 与 v ← Δx/h 本来就是精确互逆的一对，转动没理由不是。
 *
 * 为什么非精确不可：线性化形式会把实际转角缩短为 2·atan(|ω|h/2) ≈ |ω|h(1 − |ω|²h²/12)，
 * 而由位置反推速度这一步又按 sin(x)/x 再压一次，合起来每个子步损失 |ω|²h²/8 的比例。
 * 实测（ω=55 rad/s、20 子步）：ω 以 0.31/s 的速率指数衰减，硬币自转 2 秒就掉一半——
 * 这是纯数值损耗，不是物理。改用精确指积分 + 精确对数反推后，角动量守恒到机器精度。
 */
export function applyRotationExact(pose, omega, h) {
    const w = omega.length();
    if (w === 0.0) return;
    const half = 0.5 * w * h;
    const s = Math.sin(half) / w;      // ω·s = 单位轴 × sin(φ/2)
    // 左乘（世界系）：q ← exp(½h[ω,0])·q，与 applyRotationalCorrection 的
    // 「q ← q + ½([ω,0]·q)」同一约定（three.js 的 premultiply 即 a ← b·a）
    pose.q.premultiply(TMP_Q_EXP.set(omega.x * s, omega.y * s, omega.z * s, Math.cos(half)));
    pose.q.normalize();
}

/**
 * 四元数的**精确**对数映射：返回旋转向量（轴 × 角，轴为单位向量）。
 *
 * 即 Δq 真正代表的那个旋转，angle = 2·atan2(|Δq_xyz|, |Δq_w|)。
 * 论文 Algorithm 2 写的 `ω ← 2[Δq_x, Δq_y, Δq_z]/h` 是它的小角度一阶近似
 * （2·sin(θ/2) 之于 θ），对高转速体每子步都要吃掉一部分角速度，故这里用精确式。
 */
export function quatToRotationVectorExact(q) {
    const s = Math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    if (s < 1e-12) return new THREE.Vector3();
    const angle = 2.0 * Math.atan2(s, Math.abs(q.w));
    return new THREE.Vector3(q.x, q.y, q.z).multiplyScalar(angle / s);
}

/**
 * 论文 §3.3.2 的角度误差向量：由相对四元数得到旋转向量（轴 × 角）。
 *
 * 论文 Eqs. (19)(20)：q = q1 q2⁻¹，Δq_fixed = 2 (q.x, q.y, q.z)
 */
export function quatToRotationVector(q) {
    const v = new THREE.Vector3(2.0 * q.x, 2.0 * q.y, 2.0 * q.z);
    // 论文：Δq_w ≥ 0 时取正号，否则取反（对应四元数的双覆盖）
    if (q.w < 0.0) v.multiplyScalar(-1.0);
    return v;
}

/** 绕世界轴 axis 旋转 angle 得到的四元数（作用于向量） */
export function rotationAbout(axis, angle) {
    return new THREE.Quaternion().setFromAxisAngle(axis, angle);
}

/** 数值安全的归一化：长度过小时返回原向量（避免 NaN） */
export function safeNormalize(v, eps = 1e-12) {
    const len = v.length();
    if (len < eps) return v.set(0, 0, 0);
    return v.multiplyScalar(1.0 / len);
}

/**
 * 论文 §3.3.1 的广义逆质量：
 *
 *   w = 1/m + (r × n)ᵀ I⁻¹ (r × n)          （Eqs. 2, 3 —— 位置约束）
 *   w = nᵀ I⁻¹ n                            （Eqs. 12, 13 —— 角度约束）
 *
 * I 随朝向变化，故把 (r × n) 投影到体的 rest 系后再乘对角化的 invInertia0
 * （论文 §3.3.2 明确要求“把 n、r、p 投影到体的静止状态再求值”）。
 *
 * @param {RigidBody} body
 * @param {THREE.Vector3} nWorld 世界系方向（已归一化）
 * @param {THREE.Vector3|null} rWorld 世界系力臂（锚点 − 质心）；null 表示纯角度约束
 */
export function generalizedInverseMass(body, nWorld, rWorld) {
    if (!body) return 0.0;
    const tmp = new THREE.Vector3();
    if (rWorld === null || rWorld === undefined) {
        tmp.copy(nWorld);
    } else {
        tmp.crossVectors(rWorld, nWorld); // r × n
    }
    // 世界 → rest 系
    tmp.applyQuaternion(conj(body.pose.q));
    const invI = body.invInertia0;
    let w = tmp.x * tmp.x * invI.x + tmp.y * tmp.y * invI.y + tmp.z * tmp.z * invI.z;
    if (rWorld !== null && rWorld !== undefined) w += body.invMass;
    return w;
}

/**
 * 把世界系的角冲量转换为体局部 rest 系的角速度增量（I⁻¹ p）后再变回世界系。
 * 论文 Eqs. (8)(9)(16)(17) 中 I⁻¹ 的求值顺序即为此。
 */
export function applyInertiaInverse(body, vWorld) {
    const local = vWorld.applyQuaternion(conj(body.pose.q));
    const invI = body.invInertia0;
    local.set(local.x * invI.x, local.y * invI.y, local.z * invI.z);
    return local.applyQuaternion(body.pose.q);
}

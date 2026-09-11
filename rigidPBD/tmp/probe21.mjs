// 临时探针：四元数积分/速度反推的四种组合，各损失多少角速度？（用完删除）
import * as THREE from 'three';

const W = 55.0;          // rad/s
const H = 1 / 1200;      // 子步长（20 子步 @ 60fps）
const N = 20;            // 每帧子步数

const axis = new THREE.Vector3(0.3, 0.2, 0.93).normalize();
const omega = axis.clone().multiplyScalar(W);

/** 线性化积分：q ← normalize(q + ½h[ω,0]q) */
function integLin(q) {
    const dq = new THREE.Quaternion(omega.x * H, omega.y * H, omega.z * H, 0).multiply(q);
    q.set(q.x + 0.5 * dq.x, q.y + 0.5 * dq.y, q.z + 0.5 * dq.z, q.w + 0.5 * dq.w);
    q.normalize();
}
/** 精确指数积分：q ← q·exp(½h[ω,0]) */
function integExp(q) {
    const phi = W * H;
    const s = Math.sin(phi / 2), c = Math.cos(phi / 2);
    const dq = new THREE.Quaternion(axis.x * s, axis.y * s, axis.z * s, c);
    q.multiply(dq).normalize();
}
/** 论文式反推：ω = 2[Δq_xyz]/h */
function recovSmall(dq, h) {
    const v = new THREE.Vector3(2 * dq.x / h, 2 * dq.y / h, 2 * dq.z / h);
    if (dq.w < 0) v.multiplyScalar(-1);
    return v.length();
}
/** 精确对数映射反推：ω = 2·atan2(|xyz|, w)/h */
function recovExact(dq, h) {
    const s = Math.hypot(dq.x, dq.y, dq.z);
    const angle = 2 * Math.atan2(s, Math.abs(dq.w));
    return angle / h;
}

for (const [name, integ, recov] of [
    ['线性化积分 + 2[Δq]/h 反推（当前实现 / 论文写法）', integLin, recovSmall],
    ['线性化积分 + atan2 精确反推', integLin, recovExact],
    ['精确指积分 + 2[Δq]/h 反推', integExp, recovSmall],
    ['精确指积分 + atan2 精确反推', integExp, recovExact],
]) {
    const q = new THREE.Quaternion();
    const prev = new THREE.Quaternion();
    let w = W;
    for (let i = 0; i < N; i++) {
        prev.copy(q);
        integ(q);
        const dq = q.clone().multiply(prev.clone().invert());
        if (i === 0) w = recov(dq, H);
    }
    const perSub = w / W;
    console.log(`${name}\n    单子步 ω 保持 = ${(perSub).toFixed(9)}（亏 ${((1 - perSub) * 100).toExponential(2)}%）`
        + `   每帧 = ${Math.pow(perSub, N).toFixed(6)}（亏 ${((1 - Math.pow(perSub, N)) * 100).toFixed(3)}%）`
        + `   每秒衰减率 ≈ ${(Math.pow(perSub, N * 60) * 0 + (1 - Math.pow(perSub, N)) * 60).toFixed(3)}/s`);
}

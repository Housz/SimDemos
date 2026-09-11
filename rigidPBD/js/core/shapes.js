// ============================================================================
// shapes.js —— 碰撞形状与质量属性
//
// 论文 §3.2：刚体的状态除位置 x、速度 v、质量 m 外，还包含
//   朝向 q ∈ R⁴（单位四元数）、角速度 ω ∈ R³、惯性张量 I ∈ R³ˣ³
//
// 论文 §3.3.2 要求：把体在其 rest 姿态下旋转到主轴与坐标轴对齐的位置，
// 使 I 成为**对角阵**，从而可以用 Vector3 存储。本文件正是为此：
// 每种形状的惯性张量都在“体局部系”下计算，其主轴即为体的局部坐标轴。
//
// 局部坐标轴约定（与 three.js 的几何体一致）：
//   - Capsule / Cylinder 的轴向 = 局部 Y 轴
//   - Box 的尺寸 = 局部 X/Y/Z 三个方向的全长
// ============================================================================

import * as THREE from 'three';

/** 碰撞形状类型 */
export const ShapeType = {
    SPHERE: 'sphere',
    BOX: 'box',
    CAPSULE: 'capsule',
    CYLINDER: 'cylinder',
    PLANE: 'plane', // 无限大平面，只能用作静态体
};

// ---------------------------------------------------------------------------
// 形状构造
// ---------------------------------------------------------------------------

/** 球：radius */
export function sphere(radius) {
    return { type: ShapeType.SPHERE, radius };
}

/** 长方体：半长向量 (hx, hy, hz) */
export function box(hx, hy, hz) {
    return { type: ShapeType.BOX, halfExtents: new THREE.Vector3(hx, hy, hz) };
}

/** 由全长构造长方体（等价于 three.js BoxGeometry(sizeX, sizeY, sizeZ)） */
export function boxFromSize(sx, sy, sz) {
    return box(sx * 0.5, sy * 0.5, sz * 0.5);
}

/**
 * 胶囊：半径 radius + 圆柱段长 length（两半球心之间的距离）
 * 总长度 = length + 2·radius，与 three.js CapsuleGeometry(radius, length) 一致。
 */
export function capsule(radius, length) {
    return { type: ShapeType.CAPSULE, radius, length };
}

/** 圆柱：半径 radius + 全长 height（与 three.js CylinderGeometry 一致） */
export function cylinder(radius, height) {
    return { type: ShapeType.CYLINDER, radius, height };
}

/** 无限平面：法线 normal（单位）+ 平面经过 offset·normal 处。只能用于静态体。 */
export function plane(normal = new THREE.Vector3(0, 1, 0), offset = 0.0) {
    return { type: ShapeType.PLANE, normal: normal.clone().normalize(), offset };
}

// ---------------------------------------------------------------------------
// 体积与惯性张量
//
// 返回 [{volume}, {inertia: Vector3}]，inertia 为体局部系下的对角惯性张量
// （顺序 Ixx, Iyy, Izz）。论文 §3.2 提到 Blow & Binstock [BB04] 的任意网格算法，
// 这里对基本形状使用解析公式（更简单也更精确）。
// ---------------------------------------------------------------------------

export function shapeVolume(s) {
    switch (s.type) {
        case ShapeType.SPHERE:
            return (4.0 / 3.0) * Math.PI * s.radius ** 3;
        case ShapeType.BOX:
            return 8.0 * s.halfExtents.x * s.halfExtents.y * s.halfExtents.z;
        case ShapeType.CAPSULE:
            return Math.PI * s.radius ** 2 * s.length + (4.0 / 3.0) * Math.PI * s.radius ** 3;
        case ShapeType.CYLINDER:
            return Math.PI * s.radius ** 2 * s.height;
        default:
            return 0.0;
    }
}

/** 解析惯性张量（质量 mass，体局部系对角元素） */
export function shapeInertia(s, mass) {
    const I = new THREE.Vector3(0, 0, 0);
    switch (s.type) {
        case ShapeType.SPHERE: {
            const i = 0.4 * mass * s.radius ** 2; // 2/5 m r²
            I.set(i, i, i);
            break;
        }
        case ShapeType.BOX: {
            const h = s.halfExtents;
            const c = mass / 12.0;
            I.set(
                c * (4 * h.y * h.y + 4 * h.z * h.z),
                c * (4 * h.z * h.z + 4 * h.x * h.x),
                c * (4 * h.x * h.x + 4 * h.y * h.y)
            );
            break;
        }
        case ShapeType.CYLINDER: {
            // 轴向 = 局部 Y
            const r2 = s.radius * s.radius;
            const perp = (mass * (3.0 * r2 + s.height * s.height)) / 12.0;
            I.set(perp, 0.5 * mass * r2, perp);
            break;
        }
        case ShapeType.CAPSULE: {
            // 分解为圆柱段 + 两个半球，用平行轴定理合成
            const r = s.radius;
            const L = s.length;
            const r2 = r * r;
            const vCyl = Math.PI * r2 * L;
            const vSph = (4.0 / 3.0) * Math.PI * r ** 3;
            const mCyl = mass * vCyl / (vCyl + vSph);
            const mSph = mass - mCyl; // 两半球总质量
            const mHemi = 0.5 * mSph;

            // 绕轴（Y）
            const iAxis = 0.5 * mCyl * r2 + 0.4 * mSph * r2;

            // 绕垂直于轴的任一方向：圆柱 + 两个半球（半球质心在距球心 3r/8 处）
            const d = 0.5 * L + 0.375 * r; // 半球质心到胶囊质心的距离
            const iHemiCom = 0.4 * mHemi * r2 - mHemi * (0.375 * r) ** 2; // 半球绕自身质心（垂直于轴）
            const iPerp = (mCyl * (3.0 * r2 + L * L)) / 12.0 + 2.0 * (iHemiCom + mHemi * d * d);
            I.set(iPerp, iAxis, iPerp);
            break;
        }
        default:
            I.set(0, 0, 0);
    }
    return I;
}

/**
 * 体局部系下的支撑点（support point）：在局部方向 dLocal 上最远的表面点。
 * 用于精确计算世界 AABB（broad phase）以及凸体接触检测。
 * 对圆柱/胶囊，轴向为局部 Y。
 */
export function shapeSupportLocal(s, dLocal) {
    switch (s.type) {
        case ShapeType.SPHERE: {
            const dir = safeDir(dLocal);
            return dir.multiplyScalar(s.radius);
        }
        case ShapeType.BOX:
            return new THREE.Vector3(
                dLocal.x >= 0 ? s.halfExtents.x : -s.halfExtents.x,
                dLocal.y >= 0 ? s.halfExtents.y : -s.halfExtents.y,
                dLocal.z >= 0 ? s.halfExtents.z : -s.halfExtents.z
            );
        case ShapeType.CAPSULE: {
            const dir = safeDir(dLocal);
            const hy = 0.5 * s.length;
            const p = new THREE.Vector3(0, dir.y >= 0 ? hy : -hy, 0);
            return p.addScaledVector(dir, s.radius);
        }
        case ShapeType.CYLINDER: {
            const dir = safeDir(dLocal);
            const hy = 0.5 * s.height;
            const y = dir.y >= 0 ? hy : -hy;
            // 端面圆环上沿径向分量最远的点
            const t = new THREE.Vector3(dir.x, 0, dir.z);
            if (t.lengthSq() < 1e-16) t.set(s.radius, 0, 0);
            else t.normalize().multiplyScalar(s.radius);
            t.y = y;
            return t;
        }
        default:
            return new THREE.Vector3(0, 0, 0);
    }
}

/** 归一化；零向量时退化为 +X（避免 NaN） */
function safeDir(v) {
    const len = v.length();
    if (len < 1e-12) return new THREE.Vector3(1, 0, 0);
    return v.clone().multiplyScalar(1.0 / len);
}

// ============================================================================
// broadphase.js —— 宽相碰撞检测（论文 §3.5 第一段）
//
// 论文原文：
//   “To save computational cost we collect potential collision pairs once per
//    time step instead of once per sub-step using a tree of axis aligned
//    bounding boxes. We expand the boxes by a distance k·Δt·v_body, where k ≥ 1
//    is a safety multiplier accounting for potential accelerations during the
//    time step. We use k = 2 in our examples.”
//
// 即：**每帧只收集一次**候选碰撞对（而非每子步），并把 AABB 按
// k·Δt·|v| 外扩，以覆盖一个时间步内的运动。对缓存的候选对，
// 再在每个子步做窄相检测。
//
// 这里的实现是简单版本（精确 AABB + O(n²) 带早退）。论文提到的 AABB 树
// （即 BVH）作为后续优化项，可复用 LearnTenMinutePhysics/bvh.html 的 Morton 码实现。
// ============================================================================

import * as THREE from 'three';
import { ShapeType } from './shapes.js';

const _aabbA = { min: new THREE.Vector3(), max: new THREE.Vector3() };
const _aabbB = { min: new THREE.Vector3(), max: new THREE.Vector3() };
const _ownA = { min: new THREE.Vector3(), max: new THREE.Vector3() };
const _ownB = { min: new THREE.Vector3(), max: new THREE.Vector3() };

/** 被关节直接连在一起的两刚体对，编码成集合键（用于跳过自碰撞） */
export function pairKey(a, b) {
    return a.id < b.id ? `${a.id}:${b.id}` : `${b.id}:${a.id}`;
}

/**
 * 收集本帧的候选碰撞对。
 *
 * @param {RigidBody[]} bodies
 * @param {number} dt 时间步长 Δt
 * @param {number} k  安全系数（论文取 2）
 * @param {Array} out 复用的输出数组
 * @param {Set<string>} [excluded] 需要跳过的对（被关节连接的刚体）
 * @returns {Array<[RigidBody, RigidBody]>} 候选对
 */
export function collectPairs(bodies, dt, k = 2.0, out = [], excluded = null) {
    out.length = 0;

    // 1. 计算每个刚体的精确 AABB，并按 k·Δt·|v| 外扩
    const boxes = [];
    for (const body of bodies) {
        if (body.shape.type === ShapeType.PLANE) {
            boxes.push(null); // 无限平面：与所有动态体都能碰撞
            continue;
        }
        const bb = { min: new THREE.Vector3(), max: new THREE.Vector3() };
        body.computeAABB(bb);
        const margin = k * dt * body.vel.length();
        bb.min.addScalar(-margin);
        bb.max.addScalar(margin);
        boxes.push(bb);
    }

    // 2. 两两测试（跳过静态-静态对）
    const n = bodies.length;
    for (let i = 0; i < n; i++) {
        const a = bodies[i];
        for (let j = i + 1; j < n; j++) {
            const b = bodies[j];

            // 两个静态体之间不可能产生有意义的接触
            if (!a.isDynamic && !b.isDynamic) continue;

            const ba = boxes[i];
            const bb = boxes[j];
            if (ba && bb && !overlaps(ba, bb)) continue;

            // 被关节直接连接的刚体不做碰撞：相邻连杆在关节处本来就会
            // 接触/重叠，若参与碰撞求解，接触投影会和关节约束互相打架，
            // 把能量源源不断地注入系统（摆链会直接发散）。
            if (excluded && excluded.has(pairKey(a, b))) continue;

            out.push([a, b]);
        }
    }
    return out;
}

function overlaps(a, b) {
    return a.min.x <= b.max.x && a.max.x >= b.min.x
        && a.min.y <= b.max.y && a.max.y >= b.min.y
        && a.min.z <= b.max.z && a.max.z >= b.min.z;
}

// 临时探针：图 6 的 α=0 列（1g ↔ 1kg）稳态伸长有多少？初始 0.15m 错位是否主因？（用完删除）
import * as THREE from 'three';
import { World } from '../js/core/world.js';
import { RigidBody } from '../js/core/rigidBody.js';
import { boxFromSize } from '../js/core/shapes.js';
import { DistanceJoint } from '../js/core/joints.js';
import { Pose } from '../js/core/math3d.js';

const DT = 1 / 60;
const CEIL_Y = 5.0, LEN = 2.0;

function make({ drop = 0.15, iters = 1, sub = 20, seconds }) {
    const w = new World({ gravity: new THREE.Vector3(0, -9.81, 0), numSubsteps: sub, numPosIters: iters });
    const small = new RigidBody({ shape: boxFromSize(0.1, 0.1, 0.1), mass: 0.001,
        position: new THREE.Vector3(0, CEIL_Y - LEN, 0) });
    const big = new RigidBody({ shape: boxFromSize(0.3, 0.3, 0.3), mass: 1.0,
        position: new THREE.Vector3(0, CEIL_Y - 2 * LEN - drop, 0) });
    w.addBody(small); w.addBody(big);
    w.addJoint(new DistanceJoint(null, small,
        new Pose(new THREE.Vector3(0, CEIL_Y, 0)), new Pose(new THREE.Vector3(0, 0, 0)),
        { restLength: LEN, compliance: 0, isSpring: true, damping: 0.6 }));
    w.addJoint(new DistanceJoint(small, big,
        new Pose(new THREE.Vector3(0, 0, 0)), new Pose(new THREE.Vector3(0, 0, 0)),
        { restLength: LEN, compliance: 0, isSpring: true, damping: 0.6 }));
    const frames = Math.round(seconds / DT);
    let worstLate = 0, worstTop = 0;
    for (let i = 0; i < frames; i++) {
        w.step(DT);
        if (i > frames / 2) {
            const dTop = Math.abs(w.joints[0].getDistance() - LEN);
            const dBot = Math.abs(w.joints[1].getDistance() - LEN);
            worstLate = Math.max(worstLate, dTop, dBot);
            worstTop = Math.max(worstTop, dTop);
        }
    }
    const dTop = w.joints[0].getDistance() - LEN;
    const dBot = w.joints[1].getDistance() - LEN;
    return { dTop, dBot, worstLate, worstTop, small, big };
}

console.log('初始错位 / 迭代数 → 3s 与 12s 后的 |Δ|（后半程最大值）');
for (const seconds of [3, 12]) {
    console.log(`\n=== 跑 ${seconds}s ===`);
    for (const drop of [0.15, 0.0]) {
        for (const iters of [1, 3, 10]) {
            const r = make({ drop, iters, seconds });
            console.log(`  初始错位=${drop} 迭代=${iters}: 末态 上Δ=${r.dTop.toFixed(5)} 下Δ=${r.dBot.toFixed(5)}`
                + `  后半程上|Δ|max=${r.worstTop.toFixed(5)}  后半程总|Δ|max=${r.worstLate.toFixed(5)}`);
        }
    }
}

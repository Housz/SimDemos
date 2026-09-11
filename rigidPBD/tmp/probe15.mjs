// 临时探针：图 4 里 α=0.004 / m=1kg 那个关节的力为什么只有 1.18 N（用完删除）
import * as THREE from 'three';
import { World } from '../js/core/world.js';
import { RigidBody } from '../js/core/rigidBody.js';
import { boxFromSize } from '../js/core/shapes.js';
import { DistanceJoint } from '../js/core/joints.js';
import { Pose } from '../js/core/math3d.js';

const DT = 1 / 60;
const CEILING_Y = 5.0;
const BASE_LENGTH = 2.0;

function make({ m, alpha, sub = 20, damp = 0.6, seconds }) {
    const w = new World({ gravity: new THREE.Vector3(0, -9.81, 0), numSubsteps: sub, numPosIters: 1 });
    const half = 0.16 * Math.cbrt(m);
    const body = new RigidBody({
        shape: boxFromSize(half * 2, half * 2, half * 2), mass: m,
        position: new THREE.Vector3(0, CEILING_Y - BASE_LENGTH - half, 0),
    });
    w.addBody(body);
    const j = new DistanceJoint(
        null, body,
        new Pose(new THREE.Vector3(0, CEILING_Y, 0)),
        new Pose(new THREE.Vector3(0, 0, 0)),
        { restLength: BASE_LENGTH, compliance: alpha, isSpring: true, damping: damp });
    w.addJoint(j);
    const frames = Math.round(seconds / DT);
    for (let i = 0; i < frames; i++) w.step(DT);
    return { w, body, j };
}

for (const seconds of [1, 3, 6, 12]) {
    const line = [];
    for (const [m, alpha] of [[1.0, 0.010], [1.0, 0.004], [4.0, 0.010], [4.0, 0.004]]) {
        const { body, j } = make({ m, alpha, seconds });
        const d = j.getDistance() - BASE_LENGTH;
        line.push(`m=${m} α=${alpha}: Δ=${d.toFixed(4)} F=${j.lastForce.toFixed(3)}N `
            + `y=${body.pose.p.y.toFixed(4)} v=${body.vel.length().toFixed(4)} 期望Δ=${(m * 9.81 * alpha).toFixed(4)}`);
    }
    console.log(`\n=== 跑 ${seconds}s ===`);
    for (const l of line) console.log('  ' + l);
}

// 阻尼关掉看看是不是阻尼在捣鬼
console.log('\n=== 阻尼 damp=0（跑 12s）===');
for (const [m, alpha] of [[1.0, 0.004], [4.0, 0.004]]) {
    const { j } = make({ m, alpha, damp: 0.0, seconds: 12 });
    console.log(`  m=${m} α=${alpha}: Δ=${(j.getDistance() - BASE_LENGTH).toFixed(4)} F=${j.lastForce.toFixed(3)}N`);
}

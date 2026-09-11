// 临时探针：静摩擦单元测试 —— 盒子放在斜面上会不会滑？（不属于交付物，用完删除）
import * as THREE from 'three';
import { World } from '../js/core/world.js';
import { RigidBody } from '../js/core/rigidBody.js';
import { boxFromSize, plane as planeShape } from '../js/core/shapes.js';

const DT = 1 / 60;

function slopeTest(angleDeg, mu, sub = 20, iters = 1, seconds = 3.0, label = '') {
    const a = angleDeg * Math.PI / 180;
    const n = new THREE.Vector3(Math.sin(a), Math.cos(a), 0);   // 斜面法线
    const w = new World({
        gravity: new THREE.Vector3(0, -9.81, 0),
        numSubsteps: sub, numPosIters: iters,
    });
    w.addBody(new RigidBody({ shape: planeShape(n, 0), isStatic: true }));

    // 盒子摆成与斜面平行：绕 Z 转 −a
    const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -a);
    const b = new RigidBody({
        shape: boxFromSize(0.5, 0.5, 0.5), mass: 1.0,
        position: n.clone().multiplyScalar(0.25),
        quaternion: q,
    });
    b.staticFriction = mu; b.dynamicFriction = mu; b.restitution = 0;
    w.addBody(b);

    const p0 = b.pose.p.clone();
    for (let i = 0; i < Math.round(seconds / DT); i++) w.step(DT);
    const slide = b.pose.p.clone().sub(p0);
    slide.addScaledVector(n, -slide.dot(n));                     // 沿斜面的位移
    console.log(`${label.padEnd(34)} 斜面 ${String(angleDeg).padStart(2)}°  μ=${mu.toFixed(2)}  `
        + `沿坡滑移=${slide.length().toFixed(4)} m  |v|=${b.vel.length().toFixed(4)}`);
}

console.log('== 静摩擦单元测试：tan(角度) < μ 就应该站住 ==');
console.log('   （μ=0.6 → 临界角 arctan(0.6)=31°，20° 应该纹丝不动）');
slopeTest(0, 0.6);
slopeTest(20, 0.6);
slopeTest(25, 0.6);
slopeTest(20, 1.2);
slopeTest(40, 0.6);
console.log('\n== 斜面 + 多迭代对照 ==');
slopeTest(20, 0.6, 20, 4, 3.0, '20x4');
slopeTest(20, 0.6, 60, 1, 3.0, '60x1');

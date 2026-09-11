// 临时探针：平放 2 盒堆叠的逐子步接触诊断，看横向漂移从哪来（不属于交付物，用完删除）
import * as THREE from 'three';
import { World } from '../js/core/world.js';
import { RigidBody } from '../js/core/rigidBody.js';
import { boxFromSize, plane as planeShape } from '../js/core/shapes.js';
import { _debug } from '../js/core/contacts.js';

const DT = 1 / 60, BOX = 0.5;
const N = 20;

const w = new World({ gravity: new THREE.Vector3(0, -9.81, 0), numSubsteps: N, numPosIters: 1 });
w.addBody(new RigidBody({ shape: planeShape(new THREE.Vector3(0, 1, 0), 0), isStatic: true }));
const b1 = new RigidBody({ shape: boxFromSize(BOX, BOX, BOX), mass: 1.0, position: new THREE.Vector3(0, BOX / 2 - 0.001, 0) });
const b2 = new RigidBody({ shape: boxFromSize(BOX, BOX, BOX), mass: 1.0, position: new THREE.Vector3(0, BOX * 1.5 - 0.001, 0) });
for (const b of [b1, b2]) { b.staticFriction = 0.6; b.dynamicFriction = 0.6; b.restitution = 0; w.addBody(b); }

const f = (v, n = 6) => (v >= 0 ? ' ' : '') + v.toFixed(n);

// 复刻 world.step 的子步循环，但每一步都打印
w._jointedPairs.clear();
import { collectPairs } from '../js/core/broadphase.js';
collectPairs(w.bodies, DT, 2.0, w._pairs, w._jointedPairs);

const h = DT / N;
for (let s = 0; s < 8; s++) {
    if (s > 6 && s % 7 !== 0) { // 只打印有代表性的子步
        for (const b of w.bodies) b.integrate(h, w.gravity, true);
        w._generateContacts();
    w.contacts.reverse();   // 诊断：反转接触顺序
        for (const c of w.contacts) c.resetLambda();
        for (const c of w.contacts) c.solvePos(h);
        for (const b of w.bodies) { b.snapshotVelocity(); b.update(h, false); }
        for (const c of w.contacts) c.solveVel(h, 9.81);
        continue;
    }
    console.log(`\n--- 子步 ${s} ---`);
    for (const b of w.bodies) b.integrate(h, w.gravity, true);

    w._generateContacts();
    w.contacts.reverse();   // 诊断：反转接触顺序
    console.log(`  窄相 ${w.contacts.length} 个接触:`);
    for (const c of w.contacts) {
        console.log(`    d=${f(c.depth)}  p1=(${f(c.p1.x, 7)},${f(c.p1.y, 6)},${f(c.p1.z, 7)})  `
            + `n=(${c.n.x.toFixed(3)},${c.n.y.toFixed(3)},${c.n.z.toFixed(3)})  `
            + `A=${w.bodies.indexOf(c.bodyA)} B=${w.bodies.indexOf(c.bodyB)}`);
    }

    for (const c of w.contacts) c.resetLambda();
    for (const c of w.contacts) c.solvePos(h);

    console.log('  位置求解后:');
    for (const c of w.contacts) {
        console.log(`    λn=${f(c.lambdaN, 6)}  d_now=${f(c.depth)}  `
            + `Δpt=(${f(c.p1.x - c.p1Prev.x, 7)},${f(c.p1.z - c.p1Prev.z, 7)})`);
    }
    for (const [i, b] of [b1, b2].entries()) {
        console.log(`  盒${i + 1}: x=${f(b.pose.p.x, 8)} y=${f(b.pose.p.y, 5)}  ω=(${f(b.omega.x, 4)},${f(b.omega.y, 4)},${f(b.omega.z, 4)})`);
    }
    for (const b of w.bodies) { b.snapshotVelocity(); b.update(h, false); }
    for (const c of w.contacts) c.solveVel(h, 9.81);
    console.log(`  速度层后 盒1 v=(${f(b1.vel.x, 5)},${f(b1.vel.y, 5)},${f(b1.vel.z, 5)})  盒2 v=(${f(b2.vel.x, 5)},${f(b2.vel.y, 5)},${f(b2.vel.z, 5)})`);
}
_debug.maxPointsPerPair = 4;

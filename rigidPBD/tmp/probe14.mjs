// 临时探针：为 [8] boxStack 测试改写收集实测数字（不属于交付物，用完删除）
import * as THREE from 'three';
import { World } from '../js/core/world.js';
import { RigidBody } from '../js/core/rigidBody.js';
import { boxFromSize, plane as planeShape } from '../js/core/shapes.js';

const DT = 1 / 60, BOX = 0.5;

function build({ n, pen = 0, jit = 0.012, mu = 0.6, sub = 20, iters = 3 }) {
    const w = new World({ gravity: new THREE.Vector3(0, -9.81, 0), numSubsteps: sub, numPosIters: iters });
    w.addBody(new RigidBody({ shape: planeShape(new THREE.Vector3(0, 1, 0), 0), isStatic: true }));
    let seed = 12345;
    const rand = () => { seed = (seed * 1664525 + 1013904223) % 4294967296; return seed / 4294967296 - 0.5; };
    const bodies = [];
    for (let i = 0; i < n; i++) {
        const b = new RigidBody({ shape: boxFromSize(BOX, BOX, BOX), mass: 1.0,
            position: new THREE.Vector3(rand() * 2 * jit, BOX / 2 + i * BOX + 0.004 * i - pen * i, rand() * 2 * jit) });
        b.staticFriction = mu; b.dynamicFriction = mu; b.restitution = 0;
        w.addBody(b); bodies.push(b);
    }
    return { w, bodies };
}

function stats(w) {
    let maxV = 0, minY = Infinity, maxY = -Infinity, maxX = 0;
    for (const b of w.bodies) {
        if (!b.isDynamic) continue;
        maxV = Math.max(maxV, b.vel.length());
        minY = Math.min(minY, b.pose.p.y - BOX / 2);
        maxY = Math.max(maxY, b.pose.p.y);
        maxX = Math.max(maxX, Math.abs(b.pose.p.x), Math.abs(b.pose.p.z));
    }
    return { maxV, minY, maxY, maxX };
}

function trial(label, opts, seconds = 5.0) {
    const { w, bodies } = build(opts);
    const frames = Math.round(seconds / DT);
    let earlyMaxV = 0, lateMaxV = 0;
    for (let i = 0; i < frames; i++) {
        w.step(DT);
        const mv = Math.max(...bodies.map((b) => b.vel.length()));
        if (i < 6) earlyMaxV = Math.max(earlyMaxV, mv);
        if (i > frames / 2) lateMaxV = Math.max(lateMaxV, mv);
    }
    const s = stats(w);
    console.log(`  ${label.padEnd(30)} 末态maxV=${s.maxV.toFixed(4)}  末态maxX=${s.maxX.toFixed(4)}  `
        + `y∈[${s.minY.toFixed(4)}, ${s.maxY.toFixed(3)}]  后半程maxV=${lateMaxV.toFixed(3)}  `
        + `前6帧maxV=${earlyMaxV.toFixed(3)}  最大穿透=${w.maxPenetration.toFixed(5)}`);
}

for (const pen of [0.005, 0.01, 0.02]) {
    for (const on of [true, false]) {
        const { w, bodies } = build({ n: 5, pen });
        w.enableRestitutionReset = on;
        let early = 0;
        for (let i = 0; i < 30; i++) { w.step(DT); early = Math.max(early, ...bodies.map((b) => b.vel.length())); }
        for (let i = 0; i < 270; i++) w.step(DT);
        const s2 = stats(w);
        console.log(`  pen=${pen} Eq.35=${on ? '开' : '关'}  前0.5s最大速度=${early.toFixed(2)} m/s  `
            + `5s后 maxV=${s2.maxV.toFixed(4)} maxX=${s2.maxX.toFixed(3)} maxY=${s2.maxY.toFixed(3)}`);
    }
}

// 临时探针：自转硬币能转多久？参数扫描（用完删除）
import * as THREE from 'three';
import { World } from '../js/core/world.js';
import { RigidBody } from '../js/core/rigidBody.js';
import { cylinder, plane as planeShape } from '../js/core/shapes.js';

const DT = 1 / 60;
const COIN_R = 0.13, COIN_H = 0.022;

function sim({ spinRate, tiltDeg, mu, seconds = 6.0 }) {
    const w = new World({ gravity: new THREE.Vector3(0, -9.81, 0), numSubsteps: 20, numPosIters: 1, useGyroscopic: true });
    w.addBody(new RigidBody({ shape: planeShape(new THREE.Vector3(0, 1, 0), 0), isStatic: true }));
    const tilt = THREE.MathUtils.degToRad(tiltDeg);
    const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), tilt);
    const y = COIN_R * Math.cos(tilt) + (COIN_H / 2) * Math.sin(tilt);
    const coin = new RigidBody({ shape: cylinder(COIN_R, COIN_H), mass: 0.02,
        position: new THREE.Vector3(0, y, 0), quaternion: q });
    coin.omega.copy(new THREE.Vector3(0, 1, 0).applyQuaternion(q).multiplyScalar(spinRate));
    coin.restitution = 0.25; coin.dynamicFriction = mu; coin.staticFriction = mu * 1.4;
    w.addBody(coin);

    const tiltOf = () => {
        const a = new THREE.Vector3(0, 1, 0).applyQuaternion(coin.pose.q);
        return Math.acos(THREE.MathUtils.clamp(Math.abs(a.y), -1, 1));
    };
    const frames = Math.round(seconds / DT);
    let flatAt = null, maxWobble = 0, prevTilt = tiltOf(), flipped = 0;
    for (let i = 0; i < frames; i++) {
        w.step(DT);
        const t = tiltOf();
        if (i > 5) {
            // 高频抖动：倾角逐帧变化量
            maxWobble = Math.max(maxWobble, Math.abs(t - prevTilt) / DT);
            if ((t - prevTilt) * 1 > 0) flipped++;
        }
        prevTilt = t;
        if (flatAt === null && t < THREE.MathUtils.degToRad(1.5)) flatAt = i * DT;
    }
    const degf = (r) => (r * 180 / Math.PI).toFixed(1);
    return { flatAt, maxWobble, tiltEnd: prevTilt, wEnd: coin.omega.length(),
        yEnd: coin.pose.p.y, degf };
}

console.log('参数扫描：μ / 自转 / 初始倾角 → 何时倒平（<1.5°）、最大倾角变化率、末态 |ω|');
for (const mu of [0.35, 0.15, 0.05, 0.02]) {
    const parts = [];
    for (const [spin, tilt] of [[55, 8], [55, 4], [100, 4], [150, 3]]) {
        const r = sim({ spinRate: spin, tiltDeg: tilt, mu });
        parts.push(`ω${spin}/tilt${tilt}: 倒平@${r.flatAt === null ? '—' : r.flatAt.toFixed(2) + 's'}`
            + ` 抖动${r.degf(r.maxWobble)}°/s 末|ω|${r.wEnd.toFixed(1)}`);
    }
    console.log(`  μ=${mu}`);
    for (const p of parts) console.log('      ' + p);
}

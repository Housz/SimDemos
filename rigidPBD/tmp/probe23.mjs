// 临时探针：硬币（正确落地高度）的行为扫描（用完删除）
import * as THREE from 'three';
import { World } from '../js/core/world.js';
import { RigidBody } from '../js/core/rigidBody.js';
import { cylinder, plane as planeShape } from '../js/core/shapes.js';

const DT = 1 / 60, R = 0.13, H = 0.022;

function sim({ spin, tiltDeg, mu = 0.35, seconds = 8 }) {
    const w = new World({ gravity: new THREE.Vector3(0, -9.81, 0), numSubsteps: 20, numPosIters: 1, useGyroscopic: true });
    w.addBody(new RigidBody({ shape: planeShape(new THREE.Vector3(0, 1, 0), 0), isStatic: true }));
    const tilt = THREE.MathUtils.degToRad(tiltDeg);
    const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), tilt);
    const y = R * Math.sin(tilt) + (H / 2) * Math.cos(tilt);   // 轮缘最低点恰在 y=0
    const coin = new RigidBody({ shape: cylinder(R, H), mass: 0.02, position: new THREE.Vector3(0, y, 0), quaternion: q });
    coin.omega.copy(new THREE.Vector3(0, 1, 0).applyQuaternion(q).multiplyScalar(spin));
    coin.restitution = 0.25; coin.dynamicFriction = mu; coin.staticFriction = mu * 1.4;
    w.addBody(coin);

    const axisY = () => { const a = new THREE.Vector3(0, 1, 0).applyQuaternion(coin.pose.q); return a.y; };
    const phiOf = () => Math.acos(THREE.MathUtils.clamp(Math.abs(axisY()), -1, 1)); // 与竖直的夹角
    const prevs = [];
    let maxPen = 0, restAt = null;
    const frames = Math.round(seconds / DT);
    for (let i = 0; i < frames; i++) {
        w.step(DT);
        maxPen = Math.max(maxPen, w.maxPenetration);
        if (i > 4) prevs.push(phiOf());
        if (restAt === null && coin.omega.length() < 0.5) restAt = (i + 1) * DT;
    }
    // 末段倾角的振荡频率：数过零点
    const tail = prevs.slice(Math.floor(prevs.length * 0.5));
    const mean = tail.reduce((a, b) => a + b, 0) / tail.length;
    let zc = 0;
    for (let i = 1; i < tail.length; i++) if ((tail[i] - mean) * (tail[i - 1] - mean) < 0) zc++;
    const freq = zc / 2 / (tail.length * DT);
    const amp = Math.max(...tail) - Math.min(...tail);
    return { phi0: phiOf() * 57.2958, phiEnd: prevs[prevs.length - 1] * 57.2958, minPhi: Math.min(...prevs) * 57.2958,
        restAt, freq, ampDeg: amp * 57.2958, wEnd: coin.omega.length(), yEnd: coin.pose.p.y, maxPen };
}
console.log('φ0=初始与竖直夹角, φ末, 最小φ, 静止时刻(ω<0.5), 末段摆动频率Hz, 摆幅°, 末|ω|, 末y, 最大穿透');
for (const tilt of [8, 45, 70, 82, 88]) {
    for (const spin of [55, 120]) {
        const r = sim({ spin, tiltDeg: tilt });
        console.log(`  tilt=${String(tilt).padStart(2)}° ω=${String(spin).padStart(3)} → φ0=${r.phi0.toFixed(1)} φ末=${r.phiEnd.toFixed(1)}`
            + ` minφ=${r.minPhi.toFixed(1)} 静止@${r.restAt === null ? '—' : r.restAt.toFixed(2)}s`
            + ` f=${r.freq.toFixed(2)}Hz 摆幅${r.ampDeg.toFixed(1)}° 末|ω|${r.wEnd.toFixed(2)} y=${r.yEnd.toFixed(4)} pen${(r.maxPen*1000).toFixed(2)}mm`);
    }
}

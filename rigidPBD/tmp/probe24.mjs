// 临时探针：侧立自转硬币的完整减速过程（图 15 的时间线）（用完删除）
import * as THREE from 'three';
import { World } from '../js/core/world.js';
import { RigidBody } from '../js/core/rigidBody.js';
import { cylinder, plane as planeShape } from '../js/core/shapes.js';

const DT = 1 / 60, R = 0.13, H = 0.022;

function sim({ spin, leanDeg, mu = 0.35, seconds = 20 }) {
    const w = new World({ gravity: new THREE.Vector3(0, -9.81, 0), numSubsteps: 20, numPosIters: 1, useGyroscopic: true });
    w.addBody(new RigidBody({ shape: planeShape(new THREE.Vector3(0, 1, 0), 0), isStatic: true }));
    const lean = THREE.MathUtils.degToRad(leanDeg);              // 轴与竖直的夹角
    const flat = Math.PI / 2 - lean;                             // 相对「平放」的倾角
    const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), flat);
    const y = R * Math.sin(flat) + (H / 2) * Math.cos(flat);
    const coin = new RigidBody({ shape: cylinder(R, H), mass: 0.02, position: new THREE.Vector3(0, y, 0), quaternion: q });
    coin.omega.copy(new THREE.Vector3(0, 1, 0).applyQuaternion(q).multiplyScalar(spin));
    coin.restitution = 0.25; coin.dynamicFriction = mu; coin.staticFriction = mu * 1.4;
    w.addBody(coin);

    const leanOf = () => Math.acos(THREE.MathUtils.clamp(Math.abs(new THREE.Vector3(0,1,0).applyQuaternion(coin.pose.q).y), -1, 1));
    const frames = Math.round(seconds / DT);
    let last = leanOf(), prevW = spin;
    console.log(`\n== lean=${leanDeg}° ω0=${spin} μ=${mu} ==`);
    console.log('   t     lean°   |ω|     ψ(自转)  d|ω|/dt   接触  穿透mm');
    let prevLean = last, zc = [];
    for (let i = 0; i < frames; i++) {
        w.step(DT);
        const L = leanOf();
        const wm = coin.omega.length();
        if (i % 60 === 0) {
            console.log(`  ${((i+1)*DT).toFixed(1)}   ${(L*57.2958).toFixed(2).padStart(5)}  ${wm.toFixed(2).padStart(6)}`
                + `  ${coin.omega.dot(new THREE.Vector3(0,1,0).applyQuaternion(coin.pose.q)).toFixed(1).padStart(7)}`
                + `  ${((wm - prevW)/DT).toFixed(3).padStart(8)}  ${String(w.contacts.length).padStart(3)}  ${(w.maxPenetration*1000).toFixed(2)}`);
        }
        prevW = wm; prevLean = L;
    }
    console.log(`  末：lean=${(leanOf()*57.2958).toFixed(2)}°  |ω|=${coin.omega.length().toFixed(3)}  y=${coin.pose.p.y.toFixed(5)}`);
}
sim({ spin: 55, leanDeg: 8 });
sim({ spin: 55, leanDeg: 20 });

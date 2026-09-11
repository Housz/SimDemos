// 临时探针：自转硬币的角动量到底被什么吃掉了？（逐帧诊断）（用完删除）
import * as THREE from 'three';
import { World } from '../js/core/world.js';
import { RigidBody } from '../js/core/rigidBody.js';
import { cylinder, plane as planeShape } from '../js/core/shapes.js';

const DT = 1 / 60;
const COIN_R = 0.13, COIN_H = 0.022;

function sim({ spinRate = 55, tiltDeg = 8, mu = 0.35, ground = true, frames = 8, label = '' }) {
    const w = new World({ gravity: new THREE.Vector3(0, -9.81, 0), numSubsteps: 20, numPosIters: 1, useGyroscopic: true });
    if (ground) w.addBody(new RigidBody({ shape: planeShape(new THREE.Vector3(0, 1, 0), 0), isStatic: true }));
    const tilt = THREE.MathUtils.degToRad(tiltDeg);
    const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), tilt);
    const y = COIN_R * Math.cos(tilt) + (COIN_H / 2) * Math.sin(tilt);
    const coin = new RigidBody({ shape: cylinder(COIN_R, COIN_H), mass: 0.02,
        position: new THREE.Vector3(0, y, 0), quaternion: q });
    coin.omega.copy(new THREE.Vector3(0, 1, 0).applyQuaternion(q).multiplyScalar(spinRate));
    coin.restitution = 0.25; coin.dynamicFriction = mu; coin.staticFriction = mu * 1.4;
    w.addBody(coin);

    const I = coin.invInertia0;
    const Iw = (om) => {
        const loc = om.clone().applyQuaternion(coin.pose.q.clone().invert());
        return new THREE.Vector3(loc.x / I.x, loc.y / I.y, loc.z / I.z)
            .applyQuaternion(coin.pose.q);
    };
    console.log(`\n=== ${label} ===`);
    console.log('  t     倾角°   ω_轴   ω_⊥     y质心     |L|      T     接触数');
    for (let i = 0; i < frames; i++) {
        w.step(DT);
        const n = new THREE.Vector3(0, 1, 0).applyQuaternion(coin.pose.q);
        const wAxis = coin.omega.dot(n);
        const wPerp = Math.sqrt(Math.max(0, coin.omega.lengthSq() - wAxis * wAxis));
        const L = Iw(coin.omega);
        const loc = coin.omega.clone().applyQuaternion(coin.pose.q.clone().invert());
        const T = 0.5 * (loc.x * loc.x / I.x + loc.y * loc.y / I.y + loc.z * loc.z / I.z);
        const tiltNow = Math.acos(THREE.MathUtils.clamp(Math.abs(n.y), -1, 1));
        if (i < 8 || i % 10 === 0) {
            console.log(`  ${((i + 1) * DT).toFixed(3)}  ${(tiltNow * 57.2958).toFixed(1).padStart(5)}`
                + `  ${wAxis.toFixed(1).padStart(6)} ${wPerp.toFixed(2).padStart(6)}`
                + `  ${coin.pose.p.y.toFixed(4)}  ${L.length().toFixed(6)}  ${T.toFixed(6)}   ${w.contacts.length}`);
        }
    }
}

sim({ label: '基准：有地面，μ=0.35' });
sim({ label: '无地面（纯自由旋转，考察积分器本身）', ground: false });

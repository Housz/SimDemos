// 临时探针：自转硬币的倾角/进动随时间（陀螺力矩开关对比）（用完删除）
import * as THREE from 'three';
import { World } from '../js/core/world.js';
import { RigidBody } from '../js/core/rigidBody.js';
import { cylinder, plane as planeShape } from '../js/core/shapes.js';

const DT = 1 / 60;
const COIN_R = 0.13, COIN_H = 0.022;

function run({ spinRate, tiltDeg, gyro, seconds = 5.0, note = '' }) {
    const w = new World({
        gravity: new THREE.Vector3(0, -9.81, 0), numSubsteps: 20, numPosIters: 1, useGyroscopic: gyro,
    });
    w.addBody(new RigidBody({ shape: planeShape(new THREE.Vector3(0, 1, 0), 0), isStatic: true }));

    const tilt = THREE.MathUtils.degToRad(tiltDeg);
    const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), tilt);
    const y = COIN_R * Math.cos(tilt) + (COIN_H / 2) * Math.sin(tilt);
    const coin = new RigidBody({
        shape: cylinder(COIN_R, COIN_H), mass: 0.02,
        position: new THREE.Vector3(0, y, 0), quaternion: q,
    });
    const axis = new THREE.Vector3(0, 1, 0).applyQuaternion(q);
    coin.omega.copy(axis).multiplyScalar(spinRate);
    coin.restitution = 0.25; coin.dynamicFriction = 0.35; coin.staticFriction = 0.5;
    w.addBody(coin);

    const tiltOf = () => {
        const a = new THREE.Vector3(0, 1, 0).applyQuaternion(coin.pose.q);
        return Math.acos(THREE.MathUtils.clamp(Math.abs(a.y), -1, 1));
    };
    const azimOf = () => {
        const a = new THREE.Vector3(0, 1, 0).applyQuaternion(coin.pose.q);
        return Math.atan2(a.z, a.x);
    };
    const sample = [];
    const frames = Math.round(seconds / DT);
    for (let i = 0; i < frames; i++) {
        w.step(DT);
        if (i % 15 === 0) sample.push({ t: i * DT, tilt: tiltOf(), az: azimOf(), y: coin.pose.p.y, w: coin.omega.length() });
    }
    const degf = (r) => (r * 180 / Math.PI).toFixed(1);
    console.log(`\n--- ${note || `ω=${spinRate} tilt=${tiltDeg}° gyro=${gyro}`}`);
    console.log('   t:     ' + sample.map((s) => s.t.toFixed(2).padStart(6)).join(''));
    console.log('   倾角°: ' + sample.map((s) => degf(s.tilt).padStart(6)).join(''));
    console.log('   方位°: ' + sample.map((s) => degf(s.az).padStart(6)).join(''));
    console.log('   质心y: ' + sample.map((s) => s.y.toFixed(4).padStart(6)).join(''));
    console.log('   |ω|:   ' + sample.map((s) => s.w.toFixed(1).padStart(6)).join(''));
    return sample;
}

run({ spinRate: 55, tiltDeg: 8, gyro: true, note: '陀螺力矩 开（图 15 的配置）' });
run({ spinRate: 55, tiltDeg: 8, gyro: false, note: '陀螺力矩 关（对比）' });
run({ spinRate: 0, tiltDeg: 8, gyro: true, note: '不自转（应当立刻倒下）' });
run({ spinRate: 55, tiltDeg: 8, gyro: true, seconds: 12, note: '陀螺力矩 开，跑 12 秒' });

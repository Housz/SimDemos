// 临时探针：陀螺力矩开关到底有没有生效？（用完删除）
import * as THREE from 'three';
import { World } from '../js/core/world.js';
import { RigidBody } from '../js/core/rigidBody.js';
import { cylinder, plane as planeShape } from '../js/core/shapes.js';

const COIN_R = 0.13, COIN_H = 0.022;

function mk(gyro) {
    const w = new World({ gravity: new THREE.Vector3(0, -9.81, 0), numSubsteps: 20, numPosIters: 1, useGyroscopic: gyro });
    w.addBody(new RigidBody({ shape: planeShape(new THREE.Vector3(0, 1, 0), 0), isStatic: true }));
    const tilt = THREE.MathUtils.degToRad(8);
    const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), tilt);
    const y = COIN_R * Math.cos(tilt) + (COIN_H / 2) * Math.sin(tilt);
    const coin = new RigidBody({ shape: cylinder(COIN_R, COIN_H), mass: 0.02,
        position: new THREE.Vector3(0, y, 0), quaternion: q });
    coin.omega.copy(new THREE.Vector3(0, 1, 0).applyQuaternion(q).multiplyScalar(55));
    coin.restitution = 0.25; coin.dynamicFriction = 0.35; coin.staticFriction = 0.5;
    w.addBody(coin);
    return { w, coin };
}

console.log('world.useGyroscopic 字段：', mk(true).w.useGyroscopic, mk(false).w.useGyroscopic);

const a = mk(true), b = mk(false);
a.w.step(1 / 60); b.w.step(1 / 60);
console.log('1 帧后 ω 开:', a.coin.omega.toArray().map((v) => v.toFixed(9)).join(', '));
console.log('1 帧后 ω 关:', b.coin.omega.toArray().map((v) => v.toFixed(9)).join(', '));
console.log('1 帧后 q 开:', a.coin.pose.q.toArray().map((v) => v.toFixed(9)).join(', '));
console.log('1 帧后 q 关:', b.coin.pose.q.toArray().map((v) => v.toFixed(9)).join(', '));

// 单看积分器：不接触，直接让硬币自由旋转（斜着转），陀螺项应当让 ω 在体坐标系里进动
console.log('\n--- 无接触自由旋转（ω 与对称轴不平行 → 陀螺项应当立刻起作用）---');
function free(gyro) {
    const w = new World({ gravity: new THREE.Vector3(0, 0, 0), numSubsteps: 20, numPosIters: 1, useGyroscopic: gyro });
    const coin = new RigidBody({ shape: cylinder(COIN_R, COIN_H), mass: 0.02,
        position: new THREE.Vector3(0, 0, 0) });
    // 斜轴自转：ω 偏离对称轴 40°
    coin.omega.copy(new THREE.Vector3(Math.sin(0.7), Math.cos(0.7), 0).multiplyScalar(55));
    w.addBody(coin);
    const I = coin.invInertia0;
    for (let i = 0; i < 60; i++) w.step(1 / 60);
    const wLoc = coin.omega.clone().applyQuaternion(coin.pose.q.clone().invert());
    return { wLoc, I, omega: coin.omega.clone() };
}
const fOn = free(true), fOff = free(false);
console.log('invI0 =', fOn.I.toArray().map((v) => v.toExponential(3)).join(', '));
console.log('1s 后 体坐标系 ω 开:', fOn.wLoc.toArray().map((v) => v.toFixed(4)).join(', '));
console.log('1s 后 体坐标系 ω 关:', fOff.wLoc.toArray().map((v) => v.toFixed(4)).join(', '));
console.log('1s 后 世界系  |ω| 开:', fOn.omega.length().toFixed(4), ' 关:', fOff.omega.length().toFixed(4));

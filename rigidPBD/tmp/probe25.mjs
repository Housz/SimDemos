// 临时探针：为 [7] sceneCoin 的新断言测量真实数值（用完删除）
import * as THREE from 'three';
import { World } from '../js/core/world.js';
import { RigidBody } from '../js/core/rigidBody.js';
import { cylinder, plane as planeShape } from '../js/core/shapes.js';

const DT = 1 / 60, R = 0.13, H = 0.022;
const rimOffset = (flat) => R * Math.sin(flat) + (H / 2) * Math.cos(flat);

function build({ spin = 60, leanDeg = 12, count = 3, gyro = true, mu = 0.35 }) {
    const w = new World({ gravity: new THREE.Vector3(0, -9.81, 0), numSubsteps: 20, numPosIters: 1, useGyroscopic: gyro });
    w.addBody(new RigidBody({ shape: planeShape(new THREE.Vector3(0, 1, 0), 0), isStatic: true }));
    const coins = [];
    for (let i = 0; i < count; i++) {
        const x = (i - (count - 1) / 2) * 0.55;
        const lean = THREE.MathUtils.degToRad(leanDeg * (i + 1) / count);
        const flat = Math.PI / 2 - lean;
        const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), flat);
        const coin = new RigidBody({ shape: cylinder(R, H), mass: 0.02,
            position: new THREE.Vector3(x, rimOffset(flat), 0), quaternion: q });
        coin.omega.copy(new THREE.Vector3(0, 1, 0).applyQuaternion(q).multiplyScalar(spin));
        coin.restitution = 0.25; coin.dynamicFriction = mu; coin.staticFriction = mu * 1.4;
        w.addBody(coin); coins.push({ coin, flat });
    }
    return { w, coins };
}
const leanOf = (c) => Math.acos(THREE.MathUtils.clamp(Math.abs(new THREE.Vector3(0,1,0).applyQuaternion(c.pose.q).y), -1, 1));
const lowestOf = (c) => c.pose.p.y - rimOffset(Math.PI/2 - leanOf(c));
// 侧立时轮缘最低点：用解析式相对于实际姿态；这里用「离地高度」的实用近似
function lowestPointY(c) {
    const a = new THREE.Vector3(0,1,0).applyQuaternion(c.pose.q);
    const flat = Math.acos(THREE.MathUtils.clamp(Math.abs(a.y), -1, 1));
    return c.pose.p.y - (R * Math.sin(flat) + (H/2) * Math.cos(flat));
}

function run({ label, seconds = 4, spin = 60, leanDeg = 12, count = 3, gyro = true, report = false, flatAfter = false }) {
    const { w, coins } = build({ spin, leanDeg, count, gyro });
    const cs = coins.map(o => o.coin);
    const t0 = cs.map(leanOf);
    const w0 = cs.map(c => c.omega.length());
    const y0 = cs.map(lowestPointY);
    const frames = Math.round(seconds / DT);
    let minY = Infinity, maxPen = 0, maxContacts = 0, flatAt = null;
    const leanSeries = cs.map(() => []);
    for (let i = 0; i < frames; i++) {
        w.step(DT);
        maxPen = Math.max(maxPen, w.maxPenetration);
        maxContacts = Math.max(maxContacts, w.contacts.length);
        for (let k = 0; k < cs.length; k++) {
            minY = Math.min(minY, lowestPointY(cs[k]));
            leanSeries[k].push(leanOf(cs[k]));
        }
        if (flatAfter && flatAt === null && cs.every(c => leanOf(c) < THREE.MathUtils.degToRad(3))) flatAt = (i+1)*DT;
    }
    const t1 = cs.map(leanOf), w1 = cs.map(c => c.omega.length());
    const deg = (r) => r * 180 / Math.PI;
    // 末段摆动幅度与过零次数（第一枚硬币）
    const tail = leanSeries[0].slice(Math.floor(frames * 0.5));
    const amp = (Math.max(...tail) - Math.min(...tail)) * 180 / Math.PI;
    const mean = tail.reduce((a,b)=>a+b,0)/tail.length;
    let zc = 0; for (let i = 1; i < tail.length; i++) if ((tail[i]-mean)*(tail[i-1]-mean) < 0) zc++;
    const freq = zc / 2 / (tail.length * DT);
    console.log(`\n== ${label} ==`);
    console.log(`  t=0 轮缘最低点离地(m)： ${y0.map(v=>v.toFixed(6)).join('  ')}`);
    console.log(`  轴偏角： ${t0.map(v=>deg(v).toFixed(2)).join('  ')}  →  ${t1.map(v=>deg(v).toFixed(2)).join('  ')}`);
    console.log(`  |ω|：   ${w0.map(v=>v.toFixed(2)).join('  ')}  →  ${w1.map(v=>v.toFixed(2)).join('  ')}`);
    console.log(`  轮缘最低 y 全程最低 = ${minY.toFixed(5)} m   最大穿透 = ${(maxPen*1e3).toFixed(3)} mm   最大接触数 = ${maxContacts}`);
    console.log(`  后半程摆动：幅度 ${amp.toFixed(3)}°  频率 ${freq.toFixed(2)} Hz` + (flatAfter ? `  全部躺平@${flatAt===null?'—':flatAt.toFixed(2)+'s'}` : ''));
    return { minY, maxPen, maxContacts, amp, freq, flatAt, t1, w1 };
}
run({ label: '默认 4 秒（陀螺开）', seconds: 4 });
run({ label: '陀螺**关** 4 秒', seconds: 4, gyro: false });
console.log('\n--- 倒下时间对比（20 秒，看谁先全部躺平）---');
const g1 = run({ label: '陀螺开 20 秒', seconds: 20, flatAfter: true });
const g0 = run({ label: '陀螺关 20 秒', seconds: 20, gyro: false, flatAfter: true });
console.log(`\n陀螺开 躺平@${g1.flatAt}   陀螺关 躺平@${g0.flatAt}`);
console.log('\n--- drop 模式（平放落下 4 秒）---');
{
    const w = new World({ gravity: new THREE.Vector3(0,-9.81,0), numSubsteps: 20, numPosIters: 1, useGyroscopic: true });
    w.addBody(new RigidBody({ shape: planeShape(new THREE.Vector3(0,1,0), 0), isStatic: true }));
    const cs = [];
    for (let i = 0; i < 3; i++) {
        const c = new RigidBody({ shape: cylinder(R,H), mass: 0.02, position: new THREE.Vector3((i-1)*0.55, 1.2, 0) });
        c.omega.set(0, 6, 0); c.restitution = 0.25; c.dynamicFriction = 0.35; c.staticFriction = 0.5;
        w.addBody(c); cs.push(c);
    }
    for (let i = 0; i < 4*60; i++) w.step(DT);
    console.log('  末态质心高度 y =', cs.map(c=>c.pose.p.y.toFixed(5)).join('  '), ' |ω| =', cs.map(c=>c.omega.length().toFixed(3)).join('  '));
}

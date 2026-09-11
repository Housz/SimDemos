// ============================================================================
// 场景 ⑥：弹珠 —— 论文图 12 与图 13
//
// 图 12：「第 3.6 节的速度层给出正确的冲量传播：一颗弹珠从右侧撞向三颗
//         弹珠，冲量被正确地转移到最左侧那颗（中间图）。底图是使用 PBD
//         原有速度导出时的结果。」
//         —— 关键：动摩擦（Eq. 31）与恢复系数（Eq. 35）在**速度层**处理，
//            位置层只负责消除穿透，于是碰撞不再“糊”在一起。
//
// 图 13：「弹珠在初始状态就嵌在轨道里。上图：规则 PBD 的速度导出给出巨大的
//         速度，把弹珠弹飞；下图：用我们的方法，弹珠被温和地推上轨道并留在上面。」
//         —— 这就是 Eq. 35 的作用：先抹掉由位置差分导出的法向速度，
//            再补上反射速度；初始重叠时 v̄n = 0，于是分离速度为零。
//
// 场景里可以直接切换「Eq. 35 速度层」开关，亲手复现图 13 的上下对比。
// ============================================================================

import * as THREE from 'three';
import { sphere, capsule } from '../core/shapes.js';

const GRAVITY = 9.81;

export const settings = {
    mode: 'impulse',      // 'impulse'（图 12） | 'penetration'（图 13）
    useEq35: true,        // 是否启用论文 Eq. 35（关闭 = 图 13 上图的“规则 PBD”）
    penetration: 0.05,    // 图 13 的初始嵌入深度 (m)
    hitSpeed: 4.0,        // 图 12 入射弹珠速度 (m/s)
};

const MARBLE_R = 0.12;
const WIRE_R = 0.03;
const GROOVE_SEP = 0.10;  // 两根轨道线在 z 方向的半间距

export default {
    id: 'marbles',
    name: '⑥ 弹珠：冲量与初始穿透',
    tag: '图 12 / 图 13 · 速度层',
    desc: `弹珠轨道（静止胶囊线）。<br>
          <b>图 12</b>：右方弹珠以 <code>e=1</code> 撞向三颗静止弹珠，冲量沿链传递；
          速度层的恢复系数只在碰撞瞬间介入，位置层只负责消除穿透。<br>
          <b>图 13</b>：弹珠初始就<b>嵌在轨道里</b>。关闭「Eq. 35 速度层」，
          就能看到规则 PBD 把弹珠弹飞；开启后它们被温和地推上轨道——
          因为初始重叠时子步前法向速度 <code>v̄n = 0</code>，分离速度被清零。`,
    camera: { position: [1.2, 3.0, 6.4], target: [0, 1.0, 0] },
    // showContacts：接触点可视化，帮助理解
    sim: { numSubsteps: 20, numPosIters: 1, showContacts: true, showForces: false },

    create(harness) {
        const world = harness.world;
        const state = { objects: [] };


        const addBody = (opts) => {
            const b = harness.addBody(opts);
            if (b.mesh) state.objects.push(b.mesh);
            return b;
        };

        /** 一条线段状的静止轨道（胶囊） */
        function wire(from, to, radius, color, z) {
            const a = new THREE.Vector3(from.x, from.y, z);
            const b = new THREE.Vector3(to.x, to.y, z);
            const dir = new THREE.Vector3().subVectors(b, a);
            const len = dir.length();
            dir.normalize();
            const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
            const body = addBody({
                shape: capsule(radius, Math.max(0.01, len - 2 * radius)),
                isStatic: true,
                position: new THREE.Vector3().addVectors(a, b).multiplyScalar(0.5),
                quaternion: q,
                color,
            });
            return body;
        }

        /** 一对平行轨道线，构成能约束弹珠横向的“槽” */
        function groove(from, to, color) {
            wire(from, to, WIRE_R, color, -GROOVE_SEP);
            wire(from, to, WIRE_R, color, +GROOVE_SEP);
        }

        const marble = (x, y, z, color) => addBody({
            shape: sphere(MARBLE_R), mass: 0.05,
            position: new THREE.Vector3(x, y, z), color,
        });

        // -------------------------------------------------------------------
        // 图 12：冲量传递
        //   槽是水平的，弹珠在槽底静止高度 h = √((r+rw)² − sep²)
        // -------------------------------------------------------------------
        function buildImpulse() {
            const restY = Math.sqrt((MARBLE_R + WIRE_R) ** 2 - GROOVE_SEP ** 2);
            groove(new THREE.Vector3(-3.2, 0, 0), new THREE.Vector3(3.2, 0, 0), 0x4a5567);

            // 三颗静止弹珠（间距略大于直径，留出小缝隙）
            const gap = 0.03;
            const step = 2 * MARBLE_R + gap;
            const colors = [0x6ea8fe, 0x8bd3dd, 0xa78bfa];
            for (let i = 0; i < 3; i++) {
                const b = marble(-1.4 + i * step, restY, 0, colors[i]);
                b.restitution = 1.0;            // 完全弹性（论文图 12）
                b.dynamicFriction = 0.05;
                b.staticFriction = 0.05;
            }

            // 入射弹珠
            const hitter = marble(2.3, restY, 0, 0xef476f);
            hitter.restitution = 1.0;
            hitter.dynamicFriction = 0.05;
            hitter.staticFriction = 0.05;
            hitter.vel.set(-settings.hitSpeed, 0, 0);
        }

        // -------------------------------------------------------------------
        // 图 13：初始穿透
        //   轨道倾斜 15°，弹珠的球心被放在轨道表面**以下** penetration 米处
        // -------------------------------------------------------------------
        function buildPenetration() {
            const slope = THREE.MathUtils.degToRad(15);
            const d = new THREE.Vector3(Math.cos(slope), -Math.sin(slope), 0);   // 沿轨向下
            const n = new THREE.Vector3(Math.sin(slope), Math.cos(slope), 0);    // 轨道上方法线
            const start = new THREE.Vector3(-2.6, 2.0, 0);
            const end = start.clone().addScaledVector(d, 5.2);
            groove(start, end, 0x4a5567);

            const restH = Math.sqrt((MARBLE_R + WIRE_R) ** 2 - GROOVE_SEP ** 2);
            const colors = [0x6ea8fe, 0x8bd3dd, 0xa78bfa, 0xffc857];
            for (let i = 0; i < 4; i++) {
                const s = 1.1 + i * (2 * MARBLE_R + 0.06);
                const p = start.clone().addScaledVector(d, s);
                // 球心 = 轨道线上一点 + 法线方向 ×（静止高度 − 嵌入深度）
                p.addScaledVector(n, restH - settings.penetration);
                const b = marble(p.x, p.y, p.z, colors[i]);
                b.restitution = 0.0;
                b.dynamicFriction = 0.15;
                b.staticFriction = 0.3;
            }
        }

        function rebuild() {
            harness.removeObjects(state.objects);
            state.objects.length = 0;
            world.clear();
            harness.clearWatchedJoints();

            world.gravity.set(0, -GRAVITY, 0);
            world.enableRestitutionReset = settings.useEq35;

            if (settings.mode === 'impulse') buildImpulse();
            else buildPenetration();
        }
        rebuild();

        return {
            onGUI(gui) {
                gui.add(settings, 'mode', {
                    '图 12：冲量传递 (e=1)': 'impulse',
                    '图 13：初始穿透': 'penetration',
                }).name('模式').onChange(() => rebuild());

                gui.add(settings, 'useEq35').name('启用 Eq. 35 速度层')
                    .onChange((v) => {
                        // 直接切换，无需重建：这正是图 13 上下的唯一差别
                        harness.world.enableRestitutionReset = v;
                    });

                const f = gui.addFolder('参数');
                f.add(settings, 'hitSpeed', 1.0, 8.0, 0.1).name('入射速度 (m/s)')
                    .onChange(() => rebuild());
                f.add(settings, 'penetration', 0.005, 0.12, 0.005).name('初始嵌入深度 (m)')
                    .onChange(() => { if (settings.mode === 'penetration') rebuild(); });
                f.add({ go: () => rebuild() }, 'go').name('↻ 重放');
            },

            dispose() { harness.removeObjects(state.objects); },
        };
    },

    buildInfo(harness) {
        const w = harness.world;
        const balls = w.bodies.filter((b) => b.isDynamic);
        let maxV = 0, topY = -Infinity;
        for (const b of balls) {
            maxV = Math.max(maxV, b.vel.length());
            topY = Math.max(topY, b.pose.p.y);
        }
        const eq35 = w.enableRestitutionReset;
        return `<b>弹珠状态</b>\n`
            + `弹珠数 ${balls.length}   最大速度 ${maxV.toFixed(2)} m/s\n`
            + `最高球心 y = ${topY.toFixed(3)} m\n`
            + `接触点 ${w.contacts.length}   最大穿透 ${w.maxPenetration.toFixed(4)} m\n`
            + `Eq. 35 速度层：<span class="${eq35 ? 'ok' : 'warn'}">`
            + `${eq35 ? '已启用（论文方法）' : '已关闭（规则 PBD → 会被弹飞）'}</span>\n`;
    },
};

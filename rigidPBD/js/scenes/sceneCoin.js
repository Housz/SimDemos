// ============================================================================
// 场景 ⑦：硬币的高频运动 —— 论文图 15
//
// 图 15：「有了处理**曲面几何**与**子步化**的能力，我们能够复现硬币在静止前
//         的高频运动。」
//
// 这里复现的是「侧立自转的硬币」——硬币几乎立着（自转轴只偏离竖直十几度），
// 与地面的接触只发生在轮缘上的**一个点**，而这个点还在以 ωR 的速度划过地面。
//
// 为什么这个场景是论文方法的试金石：
//   1. **曲面几何**：圆柱与平面的接触点是轮缘上的单点，且随姿态连续移动。
//      contacts.js 里用「解析最深点 + 端面圆环加密采样」求它（见该处注释：
//      早期 8 点采样相当于给轮缘做了一圈 1cm 高的锯齿，硬币会被自己颠倒）。
//   2. **每子步重算法线**：接触点每子步都在动，法线若跨子步缓存就会抖。
//   3. **子步化**：ωR ≈ 7 m/s 的接触点滑移速度，在 h = 1/1200 s 内也走过 6 mm，
//      一步一算必然穿透。
//
// 观察到的（与真实硬币一致）行为时间线：
//   ① 最初 1 秒：接触点以 ωR 高速滑移，动摩擦按 μ·g/R 的速率啃掉自转
//      （μ=0.35、R=0.13 m 时约 26 rad/s²）；
//   ② 随后十几秒：硬币立着缓慢进动，倾角以肉眼可见的高频小幅摆动，
//      且**摆得越来越快**（进动频率 Ω ≈ m g R sinφ /(I_轴 ω) ∝ 1/ω）；
//   ③ 最后：失稳倒下、平躺静止。
//
// 对比实验：把全局参数 → 仿真 → 「陀螺力矩」关掉，硬币立刻失去进动、直接倒下。
// ============================================================================

import * as THREE from 'three';
import { cylinder } from '../core/shapes.js';

export const settings = {
    mode: 'spin',        // 'spin'（侧立自转） | 'drop'（平放落下）
    spinRate: 60.0,      // 自转角速度 (rad/s)，60 rad/s ≈ 9.5 转/秒
    leanDeg: 12.0,       // 自转轴偏离**竖直方向**的角度 (°)：0° = 立正，90° = 平放
    count: 3,            // 硬币数量（各自偏角不同，便于横向对比）
};

const COIN_R = 0.13;
const COIN_H = 0.022;

/**
 * 侧立硬币的初始高度：绕 X 轴转过 flat 之后，轮缘最低点恰好落在 y = 0 上。
 * 最低点在质心下方 R·sin(flat) + (H/2)·cos(flat) 处。
 *
 * ⚠ 这里曾经写反成 R·cos(flat) + (H/2)·sin(flat)（那是「轴与竖直成 flat 角」的
 * 公式），结果直立硬币被放到 0.13 m 高、凭空落下 10 cm —— 整个场景看着像在
 * 演示「硬币自由落体」，与图 15 毫无关系。
 */
function rimLowestOffset(flat) {
    return COIN_R * Math.sin(flat) + (COIN_H / 2) * Math.cos(flat);
}

export default {
    id: 'coin',
    name: '⑦ 硬币的高频运动',
    tag: '图 15 · 圆柱 / 陀螺力矩',
    desc: `侧立高速自转的圆柱硬币：与地面只在一个**轮缘点**接触，且该点以 ωR 的
          速度划过地面。<br>
          需要<b>每个子步用当前位姿重算接触法线</b>（论文 §3.5）+ 子步化才能稳定 ——
          这正是论文图 15 所说的「处理曲面几何与子步化」的能力。<br>
          观察：自转先被摩擦快速啃掉，随后硬币立着小幅高频摆动（<b>越摆越快</b>，
          因为进动频率 ∝ 1/ω），最后失稳倒下。<br>
          <b>对比</b>：把全局参数 → 仿真 → 「陀螺力矩」关掉，硬币会立刻失去进动直接倒下。`,
    camera: { position: [0.95, 0.5, 1.15], target: [0, 0.14, 0] },
    sim: { numSubsteps: 20, numPosIters: 1, showForces: false, showContacts: true },

    create(harness) {
        const world = harness.world;
        const state = { objects: [] };


        const addBody = (opts) => {
            const b = harness.addBody(opts);
            if (b.mesh) state.objects.push(b.mesh);
            return b;
        };

        function rebuild() {
            harness.removeObjects(state.objects);
            state.objects.length = 0;
            world.clear();
            harness.clearWatchedJoints();
            harness.addGroundPlane();   // 物理平面 y = 0（重建时一并加回）

            const colors = [0xffd166, 0xdfe6ee, 0xef8354, 0x8bd3dd];
            for (let i = 0; i < settings.count; i++) {
                const x = (i - (settings.count - 1) / 2) * 0.55;
                // lean 是「轴偏离竖直」的角度，由大到小排开，便于对比谁先倒
                const lean = THREE.MathUtils.degToRad(settings.leanDeg * (i + 1) / settings.count);
                const flat = Math.PI / 2 - lean;    // 相对「平放」的倾角

                let pos, omega;
                if (settings.mode === 'spin') {
                    // 轮缘最低点正好落在 y = 0 上（真正「放上去」而不是「扔下去」）
                    pos = new THREE.Vector3(x, rimLowestOffset(flat), 0);
                    // 自转轴 = 硬币局部 Y 轴
                    const axis = new THREE.Vector3(0, 1, 0).applyQuaternion(
                        new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), flat));
                    omega = axis.multiplyScalar(settings.spinRate);
                } else {
                    pos = new THREE.Vector3(x, 1.2, 0);   // 平放落下
                    omega = new THREE.Vector3(0, 6.0, 0);
                }

                const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), flat);
                const coin = addBody({
                    shape: cylinder(COIN_R, COIN_H),
                    mass: 0.02,
                    position: pos,
                    quaternion: q,
                    color: colors[i % colors.length],
                });
                coin.omega.copy(omega);
                coin.restitution = 0.25;
                coin.dynamicFriction = 0.35;
                coin.staticFriction = 0.5;
            }
        }
        rebuild();

        return {
            onGUI(gui) {
                gui.add(settings, 'mode', {
                    '侧立自转（图 15）': 'spin', '平放落下': 'drop',
                }).name('模式').onChange(() => rebuild());
                gui.add(settings, 'count', 1, 6, 1).name('硬币数')
                    .onChange(() => rebuild());
                gui.add(settings, 'spinRate', 5, 150, 1).name('自转 ω (rad/s)')
                    .onChange(() => rebuild());
                gui.add(settings, 'leanDeg', 1, 40, 0.5).name('轴偏离竖直 (°)')
                    .onChange(() => rebuild());
                gui.add({ go: () => rebuild() }, 'go').name('↻ 重新旋转');
            },

            dispose() { harness.removeObjects(state.objects); },
        };
    },

    buildInfo(harness) {
        const coins = harness.world.bodies.filter((b) => b.isDynamic);
        let html = '<b>硬币状态</b>\n';
        let i = 0;
        for (const b of coins) {
            if (i++ >= 4) break;
            const up = new THREE.Vector3(0, 1, 0).applyQuaternion(b.pose.q);
            // 轴偏离竖直的角度：0° = 立正，90° = 平躺
            const lean = Math.acos(THREE.MathUtils.clamp(Math.abs(up.y), -1, 1)) * 180 / Math.PI;
            // 自转角速度 = 角速度在硬币自转轴上的投影
            const spin = b.omega.dot(up);
            html += `#${i}  轴偏角=${lean.toFixed(1)}°  自转=${spin.toFixed(1)} rad/s  `
                + `|ω|=${b.omega.length().toFixed(2)}\n`;
        }
        html += `\n陀螺力矩：<span class="${harness.world.useGyroscopic ? 'ok' : 'warn'}">`
            + `${harness.world.useGyroscopic ? '开' : '关（硬币会直接倒下）'}</span>\n`
            + `接触点 ${harness.world.contacts.length}  最大穿透 ${harness.world.maxPenetration.toFixed(5)} m`;
        return html;
    },
};

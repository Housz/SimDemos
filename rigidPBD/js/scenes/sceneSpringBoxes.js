// ============================================================================
// 场景 ①：柔度与力 —— 论文图 4 与图 6
//
// 图 4：「力、力矩与伸长量可以在运行时可视化。柔度、力与伸长量始终保持
//        正确的关系，且与子步数、迭代数无关。」
//        —— 这正是 XPBD 相对传统弹簧-阻尼系统的核心优势：compliance α
//           是**物理材料参数**（单位 m/N，即刚度的倒数），而不是依赖
//           时间步长的数值刚度。伸长量恒为 Δ = F·α。
//
// 图 6：「1 克的盒子用距离约束挂在静态天花板下，再通过第二个关节吊住
//        1 千克的盒子。三个关节的柔度从左到右为 0.01、0.001、0 m/N。」
//        —— 1000:1 的质量比 + α=0 的无限硬约束。
//
// 验证：右上角实时显示每个关节的
//        实测力 F = |λ|/h²（论文 Eq. 11）
//        实测伸长 Δ = L − L₀
//        预测伸长 α·F
//       二者应始终吻合 —— 改变子步数/迭代数，这个关系不变。
// ============================================================================

import * as THREE from 'three';
import { Pose } from '../core/math3d.js';
import { boxFromSize } from '../core/shapes.js';
import { DistanceJoint } from '../core/joints.js';

const CEILING_Y = 6.0;
const BASE_LENGTH = 1.2;   // 关节的 rest length（弹簧原长）

// 用户可调参数放在模块级：这样「重置场景」（= 重建整个场景）后不会被打回默认值
export const settings = {
    mode: 'fig4',       // 'fig4' | 'fig6'
    heavyMass: 1.0,     // 图 6 中下方盒子的质量（kg）
};

export default {
    id: 'springBoxes',
    name: '① 柔度与力',
    tag: '图 4 / 图 6 · 距离关节',
    desc: `用<b>距离关节</b>把盒子吊在天花板下，实时显示关节力（论文 Eq. 11：
          <code>f = λ/h²</code>）与伸长量。<br>
          <b>图 4</b>：伸长量恒等于 <code>α·F</code>，与子步数、迭代数无关——
          这是 XPBD 的 compliance 作为物理量（m/N）而非数值刚度的直接体现。<br>
          <b>图 6</b>：1 克盒子吊 1 千克盒子（质量比 1000:1），
          柔度 α 分别为 0.01、0.001、0 m/N，观察 α=0 的无限硬约束。`,
    camera: { position: [6.5, 5.4, 11], target: [0, 4.6, 0] },
    // 默认打开力可视化（论文图 4 的卖点）
    sim: { showForces: true },

    create(harness) {
        const world = harness.world;

        const state = {
            joints: [],         // 参与读数显示的关节
            objects: [],        // 本场景自建的网格（重建时清理）
        };

        // 天花板横梁（纯视觉；物理上用 bodyA = null 表示连到静态世界）
        const beam = new THREE.Mesh(
            new THREE.BoxGeometry(11, 0.12, 0.4),
            new THREE.MeshStandardMaterial({ color: 0x3f4654, metalness: 0.3, roughness: 0.7 }));
        beam.position.set(0, CEILING_Y + 0.1, 0);
        beam.castShadow = true;
        harness.addObject(beam);
        state.objects.push(beam);

        // 新建刚体的辅助：登记网格以便重建时清理
        const addBody = (opts) => {
            const b = harness.addBody(opts);
            if (b.mesh) state.objects.push(b.mesh);
            return b;
        };

        // -------------------------------------------------------------------
        // 图 4：柔度 α 与质量 m 解耦
        //   伸长 Δ = m·g·α —— 同一柔度下质量越大伸长越长，但 Δ/F 恒等于 α
        // -------------------------------------------------------------------
        function buildFig4() {
            const rows = [
                { x: -3.3, m: 1.0, alpha: 0.010, color: 0x6ea8fe },
                { x: -1.1, m: 1.0, alpha: 0.004, color: 0x8bd3dd },
                { x: 1.1, m: 4.0, alpha: 0.010, color: 0xffc857 },
                { x: 3.3, m: 4.0, alpha: 0.004, color: 0xef8354 },
            ];
            for (const r of rows) {
                const half = 0.16 * Math.cbrt(r.m); // 视觉尺寸随质量开立方
                const body = addBody({
                    shape: boxFromSize(half * 2, half * 2, half * 2),
                    mass: r.m,
                    position: new THREE.Vector3(r.x, CEILING_Y - BASE_LENGTH - half, 0),
                    color: r.color,
                });
                const joint = new DistanceJoint(
                    null, body,
                    new Pose(new THREE.Vector3(r.x, CEILING_Y, 0)),
                    new Pose(new THREE.Vector3(0, 0, 0)),
                    { restLength: BASE_LENGTH, compliance: r.alpha, isSpring: true, damping: 0.6 });
                joint.meta = {
                    label: `${r.m}kg α=${r.alpha}`, alpha: r.alpha, mass: r.m, rest: BASE_LENGTH,
                };
                world.addJoint(joint);
                state.joints.push(joint);
                harness.watchJoint(joint);
            }
        }

        // -------------------------------------------------------------------
        // 图 6：大质量比 1 g ↔ 1 kg，柔度 0.01 / 0.001 / 0
        // -------------------------------------------------------------------
        function buildFig6() {
            const cols = [
                { x: -3.0, alpha: 0.010 },
                { x: 0.0, alpha: 0.001 },
                { x: 3.0, alpha: 0.0 },
            ];
            for (const c of cols) {
                // 1 克的小盒子
                const small = addBody({
                    shape: boxFromSize(0.1, 0.1, 0.1),
                    mass: 0.001,
                    position: new THREE.Vector3(c.x, CEILING_Y - BASE_LENGTH, 0),
                    color: 0x8bd3dd,
                });
                const jTop = new DistanceJoint(
                    null, small,
                    new Pose(new THREE.Vector3(c.x, CEILING_Y, 0)),
                    new Pose(new THREE.Vector3(0, 0, 0)),
                    { restLength: BASE_LENGTH, compliance: c.alpha, isSpring: true, damping: 0.6 });

                // 下面吊着的重盒。
                // 高度取「两条关节各自正好等于静止长度」的位置（锚点都在质心），
                // 这样 t = 0 就是平衡态，观众看到的第一帧不会莫名抖一下；
                // 想额外留 0.15m 的初始错位也行，但那会引入与「大质量比」无关的瞬态。
                const big = addBody({
                    shape: boxFromSize(0.3, 0.3, 0.3),
                    mass: settings.heavyMass,
                    position: new THREE.Vector3(c.x, CEILING_Y - 2 * BASE_LENGTH, 0),
                    color: 0xffc857,
                });
                const jBottom = new DistanceJoint(
                    small, big,
                    new Pose(new THREE.Vector3(0, 0, 0)),
                    new Pose(new THREE.Vector3(0, 0, 0)),
                    { restLength: BASE_LENGTH, compliance: c.alpha, isSpring: true, damping: 0.6 });

                for (const [j, tag] of [[jTop, '上'], [jBottom, '下']]) {
                    j.meta = { label: `α=${c.alpha} ${tag}`, alpha: c.alpha, rest: BASE_LENGTH };
                    world.addJoint(j);
                    state.joints.push(j);
                    harness.watchJoint(j);
                }
            }
        }

        function rebuild() {
            harness.removeObjects(state.objects);
            state.objects.length = 0;
            world.clear();
            state.joints.length = 0;
            harness.clearWatchedJoints();
            harness.addObject(beam);
            state.objects.push(beam);
            if (settings.mode === 'fig4') buildFig4(); else buildFig6();
        }

        rebuild();

        return {
            onGUI(gui) {
                gui.add(settings, 'mode', { '图 4：柔度与质量': 'fig4', '图 6：大质量比 1g↔1kg': 'fig6' })
                    .name('模式').onChange(() => rebuild());
                gui.add(settings, 'heavyMass', 0.1, 20.0, 0.1).name('图6 下方质量 (kg)')
                    .onChange(() => { if (settings.mode === 'fig6') rebuild(); });
                gui.add({ r: () => rebuild() }, 'r').name('↻ 重建');
            },

            dispose() { harness.removeObjects(state.objects); },
        };
    },

    buildInfo(harness) {
        let html = '<b>距离关节读数</b>  f = |λ|/h²  (Eq. 11)\n';
        let i = 0;
        for (const j of harness.world.joints) {
            if (!j.meta) continue;
            if (i++ >= 8) break;
            const elong = j.getDistance() - j.meta.rest;
            const F = j.lastForce;
            const predicted = j.meta.alpha * F;
            const tol = Math.max(2e-4, 0.03 * Math.abs(elong));
            const ok = Math.abs(predicted - elong) <= tol;
            html += `${j.meta.label.padEnd(12)} F=${F.toFixed(3)}N `
                + `Δ=${elong.toFixed(4)} α·F=${predicted.toFixed(4)} `
                + `<span class="${ok ? 'ok' : 'warn'}">${ok ? '✓' : '✗'}</span>\n`;
        }
        html += `\n子步数 ${harness.world.numSubsteps} × 迭代 ${harness.world.numPosIters}`
            + ` —— 改变它们，上面的关系不变。`;
        return html;
    },
};

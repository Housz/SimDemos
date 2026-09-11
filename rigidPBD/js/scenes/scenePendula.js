// ============================================================================
// 场景 ②：摆 —— 论文图 8 与图 9
//
// 图 8：「子步化给出了双摆与三摆的正确行为。我们也能轻松模拟闭环摆。」
// 图 9：「三摆模拟中的能量守恒，取决于子步数与求解迭代数。」
//        对比组合为 20×1、10×2、5×4、2×10、1×20 —— 总工作量相同。
//        结论：把迭代换成子步效果远好于反之（Macklin 等 [MSL*19]）。
//
// 本场景把摆做成**铰链关节链**（论文 §3.4.1）：
//   关节坐标系在两侧刚体上各有一个位姿，关节局部 X 轴 = 铰链轴。
//   这里把关节的 X 轴转到世界 Z 轴（绕 Y 转 −90°），于是摆在世界 XY 平面内摆动。
//
// 关节阻尼设为 0，能量曲线才有意义。论文 Table 1：摆类例子用 40 子步。
// ============================================================================

import * as THREE from 'three';
import { Pose } from '../core/math3d.js';
import { boxFromSize } from '../core/shapes.js';
import { HingeJoint } from '../core/joints.js';

const CEILING_Y = 5.4;
const ROD_W = 0.07;         // 杆的截面边长
const ROD_L = 1.0;          // 杆长
const ROD_MASS = 1.0;

// 把关节局部 X 轴映射到世界 Z 轴（铰链轴），使摆在 XY 平面内摆动
const AXIS_TO_Z = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -Math.PI / 2);

export const settings = {
    mode: 'triple',     // single | double | triple | closed
};

export default {
    id: 'pendula',
    name: '② 单/双/三摆与闭环',
    tag: '图 8 / 图 9 · 铰链关节',
    desc: `铰链关节链构成的摆。<b>子步化</b>让双摆、三摆呈现出正确的混沌行为，
          闭环四连杆也只是多加几个铰链约束（PBD 天然处理过约束系统）。<br>
          打开<b>能量曲线</b>（显示 → 能量曲线，本场景已默认开启），
          再用下方预设按钮对比 <code>20×1 / 10×2 / 5×4 / 2×10 / 1×20</code>：
          总工作量相同，但把迭代换成子步的能量守恒明显更好——
          这正是论文图 9 的结论。`,
    camera: { position: [0.5, 3.4, 8.5], target: [0, 2.6, 0] },
    // 论文 Table 1：摆类例子用 40 子步；showEnergy 是图 9 的主角
    sim: { numSubsteps: 40, numPosIters: 1, showEnergy: true, showForces: false },

    create(harness) {
        const world = harness.world;
        const state = { joints: [], objects: [], phase: 0 };

        const addBody = (opts) => {
            const b = harness.addBody(opts);
            if (b.mesh) state.objects.push(b.mesh);
            return b;
        };

        // 天花板横梁（视觉）
        const beam = new THREE.Mesh(
            new THREE.BoxGeometry(6, 0.10, 0.3),
            new THREE.MeshStandardMaterial({ color: 0x3f4654, metalness: 0.3, roughness: 0.7 }));
        beam.position.set(0, CEILING_Y + 0.09, 0);
        beam.castShadow = true;
        harness.addObject(beam);
        state.objects.push(beam);

        /**
         * 建一根杆：顶端位于 topWorld，绕 Z 轴转过 theta 弧度。
         * @returns {{body, top, bottom}}
         */
        function makeRod(topWorld, theta, length, color) {
            const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), theta);
            const down = new THREE.Vector3(0, -length, 0).applyQuaternion(q);
            const body = addBody({
                shape: boxFromSize(ROD_W, length, ROD_W),
                mass: ROD_MASS,
                position: topWorld.clone().addScaledVector(down, 0.5),
                quaternion: q,
                color,
            });
            return { body, top: topWorld.clone(), bottom: topWorld.clone().add(down) };
        }

        /** 铰链关节：把前一节杆的下端与后一节杆的上端连起来 */
        function hinge(prevRod, rod, index) {
            const localA = prevRod
                ? new Pose(new THREE.Vector3(0, -ROD_L / 2, 0), AXIS_TO_Z)  // 前杆下端
                : new Pose(rod.top, AXIS_TO_Z);                              // 天花板锚点
            const localB = new Pose(new THREE.Vector3(0, ROD_L / 2, 0), AXIS_TO_Z); // 本杆上端
            const j = new HingeJoint(prevRod ? prevRod.body : null, rod.body, localA, localB, {
                compliance: 0.0,        // 无限硬（PBD）
                damping: 0.0,           // 无阻尼 → 能量曲线才有意义
                minAngle: -Math.PI,
                maxAngle: Math.PI,
            });
            j.meta = { label: `铰链 ${index}` };
            world.addJoint(j);
            state.joints.push(j);
            harness.watchJoint(j, { torque: true });
            return j;
        }

        // -------------------------------------------------------------------
        // 各种摆的初始位形
        // -------------------------------------------------------------------
        function buildSingle() {
            const t = Math.PI * 0.44;   // 接近水平释放
            const rod = makeRod(new THREE.Vector3(0, CEILING_Y, 0), t, ROD_L * 1.6, 0x6ea8fe);
            hinge(null, rod, 0);
        }

        function buildChain(n, angles) {
            let top = new THREE.Vector3(0, CEILING_Y, 0);
            let prev = null;
            for (let i = 0; i < n; i++) {
                const rod = makeRod(top, angles[i], ROD_L, i % 2 ? 0x8bd3dd : 0x6ea8fe);
                hinge(prev, rod, i);
                prev = rod;
                // 注意：下一根杆的顶端接在本杆**绕自身朝向**的下端
                top = rod.bottom;
            }
        }

        /**
         * 闭环四连杆：两根悬挂杆 + 底部连杆构成平行四边形机构。
         * 4 个铰链约束在运动学上彼此相关（过约束），PBD 的 Gauss-Seidel
         * 会稳定地把它解出来 —— 这正是论文强调的“通用性”。
         */
        function buildClosed() {
            const span = 2.0;          // 两悬挂点间距
            const rodLen = 1.2;
            const barLen = 1.6;
            // 静止位形：两杆内摆 φ，使下端间距恰好等于连杆长度
            //   2·span/2 − 2·rodLen·sinφ = barLen
            const sinPhi = (span - barLen) / (2 * rodLen);
            const phi = Math.asin(THREE.MathUtils.clamp(sinPhi, -1, 1));

            const topL = new THREE.Vector3(-span / 2, CEILING_Y - 0.9, 0);
            const topR = new THREE.Vector3(span / 2, CEILING_Y - 0.9, 0);
            const rodL = makeRod(topL, -phi, rodLen, 0x6ea8fe);
            const rodR = makeRod(topR, +phi, rodLen, 0x8bd3dd);

            // 底部连杆（水平）
            const barCenter = new THREE.Vector3(0, rodL.bottom.y, 0);
            const bar = addBody({
                shape: boxFromSize(barLen, ROD_W, ROD_W),
                mass: 2.0,
                position: barCenter,
                color: 0xffc857,
            });

            // 悬挂点 → 两杆
            world.addJoint(new HingeJoint(null, rodL.body,
                new Pose(rodL.top, AXIS_TO_Z), new Pose(new THREE.Vector3(0, rodLen / 2, 0), AXIS_TO_Z),
                { damping: 0 }));
            world.addJoint(new HingeJoint(null, rodR.body,
                new Pose(rodR.top, AXIS_TO_Z), new Pose(new THREE.Vector3(0, rodLen / 2, 0), AXIS_TO_Z),
                { damping: 0 }));
            // 两杆下端 → 连杆两端
            world.addJoint(new HingeJoint(rodL.body, bar,
                new Pose(new THREE.Vector3(0, -rodLen / 2, 0), AXIS_TO_Z),
                new Pose(new THREE.Vector3(-barLen / 2, 0, 0), AXIS_TO_Z), { damping: 0 }));
            world.addJoint(new HingeJoint(rodR.body, bar,
                new Pose(new THREE.Vector3(0, -rodLen / 2, 0), AXIS_TO_Z),
                new Pose(new THREE.Vector3(barLen / 2, 0, 0), AXIS_TO_Z), { damping: 0 }));

            // 给一点初速度让它摆起来
            bar.vel.set(1.6, 0, 0);
        }

        function rebuild() {
            harness.removeObjects(state.objects);
            state.objects.length = 0;
            world.clear();
            state.joints.length = 0;
            harness.clearWatchedJoints();
            harness.addObject(beam);
            state.objects.push(beam);

            switch (settings.mode) {
                case 'single': buildSingle(); break;
                case 'double': buildChain(2, [Math.PI * 0.35, Math.PI * 0.25]); break;
                case 'triple': buildChain(3, [Math.PI * 0.5, 0.0, 0.0]); break;
                case 'closed': buildClosed(); break;
            }
        }
        rebuild();

        /** 论文图 9 的对比组合：总工作量相同（子步数 × 迭代数 = 20） */
        const presets = {
            '20 子步 × 1 迭代': [20, 1],
            '10 子步 × 2 迭代': [10, 2],
            '5 子步 × 4 迭代': [5, 4],
            '2 子步 × 10 迭代': [2, 10],
            '1 子步 × 20 迭代': [1, 20],
        };

        return {
            onGUI(gui) {
                gui.add(settings, 'mode', {
                    '单摆': 'single', '双摆': 'double', '三摆': 'triple', '闭环四连杆': 'closed',
                }).name('摆型').onChange(() => rebuild());

                const f = gui.addFolder('图 9：子步 vs 迭代（等总工作量）');
                for (const [label, [n, it]] of Object.entries(presets)) {
                    f.add({
                        go: () => {
                            // setParams：登记为显式覆盖，重置场景时不会被 40×1 打回去
                            harness.setParams({ numSubsteps: n, numPosIters: it });
                            harness.resetScene();
                        },
                    }, 'go').name(label);
                }
            },

            dispose() { harness.removeObjects(state.objects); },
        };
    },

    buildInfo(harness) {
        const { kinetic, potential } = harness.computeEnergy();
        const total = kinetic + potential;
        return `<b>能量守恒（论文图 9）</b>\n`
            + `动能 T   = ${kinetic.toFixed(4)} J\n`
            + `势能 V   = ${potential.toFixed(4)} J\n`
            + `总能量 E = ${total.toFixed(4)} J\n`
            + `求解配置 ${harness.world.numSubsteps} 子步 × ${harness.world.numPosIters} 迭代`;
    },
};

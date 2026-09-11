// ============================================================================
// 场景 ⑤：扭转的绳子 —— 论文图 18
//
// 图 18：「分别处理 swing 与 twist 限制，使我们能模拟一根扭转的绳子。
//         它只是一串由**球窝关节**连接的 100 个胶囊。」
//
// 球窝关节本身不限制任何转动自由度，扭转之所以能沿绳子传播并形成螺旋，
// 完全依赖论文 Eqs. 23-25 的**扭转解耦**：
//     n  ← (a₁ + a₂)/|a₁ + a₂|          （公共的“绳轴”）
//     n₁ ← b₁ − (n·b₁)n ,  n₂ ← b₂ − (n·b₂)n   （把扭转从摆动里剥出来）
// 于是 swing 与 twist 可以各自独立地设定限制角与柔度。
//
// 操作：拖动「末端扭矩」滑杆给绳子的自由端加一个绕自身轴的力矩，
//       扭转会沿绳向上传播；松开后绳子缓慢回弹。
// ============================================================================

import * as THREE from 'three';
import { Pose } from '../core/math3d.js';
import { capsule, boxFromSize } from '../core/shapes.js';
import { SphericalJoint } from '../core/joints.js';

// 胶囊轴向 = 其局部 Y 轴；球窝关节的局部 X 轴要沿绳轴，故映射 X → Y
const AXIS_TO_Y = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 2);
const deg = THREE.MathUtils.degToRad;

const TOP_Y = 5.0;
const SEG_LEN = 0.12;      // 关节之间的间距（= 胶囊圆柱段长度）
const CAP_R = 0.035;

/**
 * 量一个球窝关节当前的 twist 角（论文 Eqs. 23-25 的 twist 那一半）：
 * 把两侧的 b 轴投影到垂直于关节轴 n 的平面上再比较。
 *
 * 要点是**带符号**。`acos(b₁·b₂)` 只有大小没有方向，相邻关节一正一负的滚转
 * 会被累加成两倍，看起来像绳子自己扭了一大圈。这里用
 * `atan2((b₁×b₂)·n, b₁·b₂)` 取绕 n 的有符号转角，沿绳链累加才是「绳子总扭转」。
 *
 * @returns {{n: THREE.Vector3|null, angle: number}} n 为 null 表示退化（两轴反向共线）
 */
function measureTwist(joint) {
    joint.updateGlobalPoses();
    const a1 = new THREE.Vector3(1, 0, 0).applyQuaternion(joint.globalPoseA.q);
    const a2 = new THREE.Vector3(1, 0, 0).applyQuaternion(joint.globalPoseB.q);
    const n = new THREE.Vector3().addVectors(a1, a2);
    if (n.lengthSq() < 1e-12) return { n: null, angle: 0 };
    n.normalize();
    const b1 = new THREE.Vector3(0, 1, 0).applyQuaternion(joint.globalPoseA.q);
    b1.addScaledVector(n, -n.dot(b1));
    const b2 = new THREE.Vector3(0, 1, 0).applyQuaternion(joint.globalPoseB.q);
    b2.addScaledVector(n, -n.dot(b2));
    if (b1.lengthSq() < 1e-12 || b2.lengthSq() < 1e-12) return { n: null, angle: 0 };
    b1.normalize(); b2.normalize();
    const sin = new THREE.Vector3().crossVectors(b1, b2).dot(n);
    return { n, angle: Math.atan2(sin, b1.dot(b2)) };
}

export const settings = {
    segments: 24,          // 胶囊数量（论文用 100；浏览器里默认 24 更流畅）
    swingDeg: 22.0,
    twistDeg: 35.0,
    twistAlpha: 0.004,     // twist 限制的柔度（比 swing 软，扭转能逐渐积累）
    swingAlpha: 0.0,
    torque: 0.0,           // 末端扭矩 (N·m)
    torqueMode: false,
};

export default {
    id: 'rope',
    name: '⑤ 扭转的绳子',
    tag: '图 18 · 球窝 swing/twist',
    desc: `一串由<b>球窝关节</b>连接的胶囊。球窝关节允许任意转动，
          绳子的扭转行为完全来自<b>swing / twist 解耦</b>（论文 Eqs. 23-25）
          加上各自独立的限制与柔度。<br>
          拖动「末端扭矩」给自由端加绕轴力矩，观察扭转沿绳向上传播、
          以及 swing 限制如何阻止绳子被拉直。<br>
          <b>鼠标左键可直接抓拽绳子</b>（抓取本身就是一个柔度可调的约束，
          在每个子步都被求解）。`,
    camera: { position: [3.2, 3.4, 4.6], target: [0, 3.0, 0] },
    sim: { numSubsteps: 20, numPosIters: 1, showForces: false },

    create(harness) {
        const world = harness.world;
        const state = { objects: [], time: 0 };


        const addBody = (opts) => {
            const b = harness.addBody(opts);
            if (b.mesh) state.objects.push(b.mesh);
            return b;
        };

        // 顶部固定横梁（视觉）
        const beam = new THREE.Mesh(
            new THREE.BoxGeometry(1.6, 0.10, 0.3),
            new THREE.MeshStandardMaterial({ color: 0x3f4654, metalness: 0.3, roughness: 0.7 }));
        beam.position.set(0, TOP_Y + 0.09, 0);
        beam.castShadow = true;
        harness.addObject(beam);
        state.objects.push(beam);

        const bodies = [];
        let prev = null;

        for (let i = 0; i < settings.segments; i++) {
            const cy = TOP_Y - (i + 0.5) * SEG_LEN;
            const body = addBody({
                shape: capsule(CAP_R, SEG_LEN),
                mass: 0.05,
                position: new THREE.Vector3(0, cy, 0),
                color: i % 2 ? 0xd8b384 : 0xc99a63,
            });
            bodies.push(body);

            const localA = prev
                ? new Pose(new THREE.Vector3(0, -SEG_LEN / 2, 0), AXIS_TO_Y)
                : new Pose(new THREE.Vector3(0, TOP_Y, 0), AXIS_TO_Y);
            const localB = new Pose(new THREE.Vector3(0, SEG_LEN / 2, 0), AXIS_TO_Y);

            const joint = new SphericalJoint(prev, body, localA, localB, {
                compliance: 0.0,
                swingLimit: [-deg(settings.swingDeg), deg(settings.swingDeg)],
                swingCompliance: settings.swingAlpha,
                twistLimit: [-deg(settings.twistDeg), deg(settings.twistDeg)],
                twistCompliance: settings.twistAlpha,
                damping: 0.02,
            });
            world.addJoint(joint);
            prev = body;
        }

        const last = bodies[bodies.length - 1];
        const tipBody = addBody({
            shape: boxFromSize(0.10, 0.10, 0.10),
            mass: 0.15,
            position: new THREE.Vector3(0, TOP_Y - settings.segments * SEG_LEN - 0.07, 0),
            color: 0xef476f,
        });
        world.addJoint(new SphericalJoint(last, tipBody,
            new Pose(new THREE.Vector3(0, -SEG_LEN / 2, 0), AXIS_TO_Y),
            new Pose(new THREE.Vector3(0, 0.05, 0), AXIS_TO_Y),
            {
                compliance: 0.0,
                swingLimit: [-deg(settings.swingDeg), deg(settings.swingDeg)],
                twistLimit: [-deg(settings.twistDeg), deg(settings.twistDeg)],
                twistCompliance: settings.twistAlpha,
                damping: 0.02,
            }));

        // 给一点初始扰动，让绳子不是完美的直线（否则 swing 平面退化）
        bodies[Math.floor(bodies.length / 2)].vel.set(0.3, 0, 0.15);

        return {
            preStep(dt) {
                state.time += dt;
                if (settings.torqueMode) settings.torque = 2.0 * Math.sin(state.time * 0.8);

                // 在自由端施加绕**该胶囊自身轴**的力矩。
                // World.step() 会在所有子步中持续施加外力，帧末才清零，
                // 所以这里设置的力矩等价于每个子步都加一次（论文 Algorithm 2）。
                tipBody.torque.set(0, 0, 0);
                if (settings.torque !== 0.0) {
                    const axis = new THREE.Vector3(0, 1, 0).applyQuaternion(tipBody.pose.q);
                    tipBody.torque.copy(axis).multiplyScalar(settings.torque);
                }
            },

            onGUI(gui) {
                gui.add(settings, 'torque', -3.0, 3.0, 0.01).name('末端扭矩 (N·m)');
                gui.add(settings, 'torqueMode').name('扭矩自动往复');
                gui.add(settings, 'segments', 6, 40, 1).name('胶囊数量')
                    .onChange(() => harness.resetScene());
                const f = gui.addFolder('关节限制');
                f.add(settings, 'swingDeg', 0, 90, 1).name('swing 限制 (°)')
                    .onChange((v) => {
                        for (const j of harness.world.joints) {
                            if (j.swingLimit) j.swingLimit = [-deg(v), deg(v)];
                        }
                    });
                f.add(settings, 'twistDeg', 0, 180, 1).name('twist 限制 (°)')
                    .onChange((v) => {
                        for (const j of harness.world.joints) {
                            if (j.twistLimit) j.twistLimit = [-deg(v), deg(v)];
                        }
                    });
                f.add(settings, 'twistAlpha', 0.0, 0.02, 0.0005).name('twist 柔度 α')
                    .onChange((v) => {
                        for (const j of harness.world.joints) {
                            if (j.twistLimit) j.twistCompliance = v;
                        }
                    });
                f.add(settings, 'swingAlpha', 0.0, 0.02, 0.0005).name('swing 柔度 α')
                    .onChange((v) => {
                        for (const j of harness.world.joints) {
                            if (j.swingLimit) j.swingCompliance = v;
                        }
                    });
                gui.add({ go: () => { settings.torque = 0.0; harness.refreshGUI(); } }, 'go')
                    .name('↻ 撤销扭矩');
            },

            dispose() { harness.removeObjects(state.objects); },
        };
    },

    buildInfo(harness) {
        const joints = harness.world.joints;
        let limited = 0;
        let totalTwist = 0;     // 带符号求和：相邻关节反向滚转应当相互抵消
        let maxJoint = 0;
        for (const j of joints) {
            if (!j.twistLimit) continue;
            const { n, angle } = measureTwist(j);
            if (!n) continue;
            totalTwist += angle;                    // 沿绳链累加 = 绳子的总扭转
            maxJoint = Math.max(maxJoint, Math.abs(angle));
            if (Math.abs(angle) >= deg(settings.twistDeg) - deg(2.0)) limited++;
        }
        return `<b>绳子状态</b>\n`
            + `段数 ${joints.length}\n`
            + `总扭转 ${deg(totalTwist).toFixed(1)}°（沿链带符号累加）\n`
            + `单关节最大 |twist| ${deg(maxJoint).toFixed(1)}°\n`
            + `顶到 twist 限制的关节 ${limited} 个\n`
            + `末端扭矩 ${settings.torque.toFixed(2)} N·m\n`
            + `\n限制角 swing ±${settings.swingDeg}°  twist ±${settings.twistDeg}°`;
    },
};

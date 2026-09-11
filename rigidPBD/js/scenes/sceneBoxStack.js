// ============================================================================
// 场景 ⑧：盒子堆叠 —— 论文 Table 1 的基准场景
//
// Table 1 的第一行就是它：
//     Example    substeps  iters/substep  time (ms/frame)
//     3 Boxes        20          1              0.34
//     7 Boxes        20          1              0.44
//   （全部例子都用 Δt = 1/60 s）
//
// 堆叠是检验位置层最严苛的场景：每个盒子有多个接触点，
// 位置投影必须**逐个接触用当前状态重算法线**（论文 §3.5），
// 否则多点接触会把旋转越推越大，最终发散。
// 静摩擦（Eq. 28-29）则在 λt < μs·λn 的摩擦锥内阻止盒子滑开。
//
// 打开「接触点」可以看到 box-box SAT 生成的多个接触点；
// 右上角的最大穿透量应当稳定在很小的量级（论文 §4 用约束伸长/穿透衡量精度）。
// ============================================================================

import * as THREE from 'three';
import { boxFromSize } from '../core/shapes.js';

export const settings = {
    count: 7,            // 堆叠数量（论文基准：3 / 7）
    jitter: 0.012,       // 初始水平随机偏移（m），制造非完美堆叠
    gap: 0.004,          // 初始竖直间隙（m）
    friction: 0.6,
    restitution: 0.0,
};

const BOX = 0.5;         // 立方体边长

export default {
    id: 'boxStack',
    name: '⑧ 盒子堆叠',
    tag: '表 1 · box-box SAT + 摩擦',
    desc: `论文 Table 1 的基准场景（3 盒 / 7 盒）。<br>
          每个盒子对之间有多个接触点，<b>静摩擦</b>（Eqs. 28-29）在
          <code>λt &lt; μs·λn</code> 的摩擦锥内阻止滑动；位置投影逐点进行，
          每次都用<b>当前位姿重新计算法线</b>——这是堆叠不发散的关键。<br>
          右上角的最大穿透量应始终保持在很小的量级。<br>
          <b>注意</b>：本项目在论文的 20 子步 × 1 迭代下只能稳住 3 盒，
          7 盒需要 3 次迭代（根因见 README「盒子堆叠与 Table 1 的偏差」）。`,
    camera: { position: [3.4, 3.0, 4.4], target: [0, 1.5, 0] },

    // ⚠ 与论文 Table 1 的偏差：论文的 3 Boxes / 7 Boxes 用的是 **20 子步 × 1 迭代**，
    // 本实现在这个配置下只能稳住 3 盒（7 盒会横向“剪切”倒塌，实测 |x| 漂到 0.49 m）。
    // 根因见 contacts.js solvePos 的注释：一个平面接触片上的 k>3 个共面点是线性相关的，
    // 而 Gauss-Seidel 的遍历顺序不对称，逐点全量投影会留下人造横向位移并逐子步累积。
    // 这里提高到 3 次迭代把它压下去（实测 7 盒 |x|=0.027 m、最大速度 0.25 m/s）。
    // 实测的稳定上限：3 盒 @1 迭代、7–8 盒 @3 迭代、9 盒以上需要更大子步数或更低摩擦。
    // GUI 里「子步数/迭代数」是全局参数，读者可以自己调回 20×1 复现论文设置并观察差异。
    sim: { numSubsteps: 20, numPosIters: 3, showContacts: true, showForces: false },

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
            harness.addGroundPlane();

            // 用固定种子的伪随机，保证“重置”后位形一致（便于对比参数影响）
            let seed = 12345;
            const rand = () => {
                seed = (seed * 1664525 + 1013904223) % 4294967296;
                return seed / 4294967296 - 0.5;
            };

            const colors = [0x6ea8fe, 0x8bd3dd, 0xa78bfa, 0xffc857, 0xef8354,
                            0x9ecbff, 0xd8b384, 0x7ee787];

            for (let i = 0; i < settings.count; i++) {
                const y = BOX / 2 + i * BOX + settings.gap * i;
                const body = addBody({
                    shape: boxFromSize(BOX, BOX, BOX),
                    mass: 1.0,
                    position: new THREE.Vector3(
                        rand() * 2 * settings.jitter,
                        y,
                        rand() * 2 * settings.jitter),
                    color: colors[i % colors.length],
                });
                body.staticFriction = settings.friction;
                body.dynamicFriction = settings.friction;
                body.restitution = settings.restitution;
            }
        }
        rebuild();

        return {
            onGUI(gui) {
                // 上限取 8：实测 9 盒以上在 μ=0.6 会失稳（见 create() 里的说明），
                // 把滑杆限制在已经验证过的范围内，避免演示直接给出错误结论。
                gui.add(settings, 'count', 1, 8, 1).name('盒子数量')
                    .onChange(() => rebuild());
                gui.add(settings, 'friction', 0.0, 1.2, 0.05).name('摩擦系数 μ')
                    .onChange((v) => {
                        for (const b of harness.world.bodies) {
                            if (!b.isDynamic) continue;
                            b.staticFriction = v;
                            b.dynamicFriction = v;
                        }
                    });
                gui.add(settings, 'restitution', 0.0, 0.9, 0.05).name('恢复系数 e')
                    .onChange((v) => {
                        for (const b of harness.world.bodies) {
                            if (b.isDynamic) b.restitution = v;
                        }
                    });
                const f = gui.addFolder('初始条件');
                f.add(settings, 'jitter', 0.0, 0.06, 0.002).name('水平随机偏移 (m)')
                    .onChange(() => rebuild());
                f.add({ go: () => rebuild() }, 'go').name('↻ 重新堆叠');
            },

            dispose() { harness.removeObjects(state.objects); },
        };
    },

    buildInfo(harness) {
        const w = harness.world;
        const boxes = w.bodies.filter((b) => b.isDynamic);
        let maxSpeed = 0, topY = -Infinity;
        for (const b of boxes) {
            maxSpeed = Math.max(maxSpeed, b.vel.length());
            topY = Math.max(topY, b.pose.p.y);
        }
        const stable = maxSpeed < 0.05;
        return `<b>堆叠状态</b>（论文 Table 1）\n`
            + `盒子数 ${boxes.length}   质心最高 y=${topY.toFixed(3)} m\n`
            + `最大速度 ${maxSpeed.toFixed(4)} m/s  `
            + `<span class="${stable ? 'ok' : 'warn'}">${stable ? '✓ 稳定' : '… 仍在运动'}</span>\n`
            + `接触点 ${w.contacts.length}   候选对 ${w.pairsCount}\n`
            + `最大穿透 ${w.maxPenetration.toFixed(5)} m\n`
            + `摩擦 μ=${settings.friction.toFixed(2)}  恢复 e=${settings.restitution.toFixed(2)}\n`
            + `求解 ${w.numSubsteps} 子步 × ${w.numPosIters} 迭代`;
    },
};

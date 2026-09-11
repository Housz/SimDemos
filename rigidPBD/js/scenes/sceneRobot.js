// ============================================================================
// 场景 ④：机械臂逆向运动学 —— 论文图 17
//
// 图 17：「我们的方法可以求解**过约束**系统的逆向运动学问题，系统甚至可以
//         有冗余自由度，例如这条机械臂。当目标不可达时会出现过约束问题，
//         我们的方法能优雅地处理：把夹爪移动到尽可能接近目标位姿的地方。」
//
// 这里的做法完全是论文式的，没有任何专门 IK 求解器：
//   1. 一条**位置约束**把夹爪锚点拉向目标盒（柔度 α 控制“跟随力度”）
//      —— 论文 §3.3.1 的位置投影，bodyA 为运动学目标（静态体）。
//   2. 每个铰链关节的**目标角度**约束（Eq. 22）设成较小的柔度，
//      把冗余自由度往“自然姿态”上拉 —— 这就是正则化，避免零空间漂移。
//   3. 每个铰链关节的**角度限制**（Algorithm 3）保证解始终在可达范围内。
//
// 三者都是同一套 XPBD 投影，Gauss-Seidel 一次扫描就把它们全部满足，
// 目标不可达时自然收敛到“最接近”的位姿。
//
// ---------------------------------------------------------------------------
// 关于俯仰铰链的轴（踩过的坑，值得记下来）
//
// 铰链关节的旋转轴 = **关节坐标系的局部 X 轴**，角度参考方向 = 局部 Y 轴
// （见 joints.js HingeJoint.getCurrentAngle / solvePos）。因此轴必须与连杆
// 方向**垂直**：如果轴沿连杆（identity 姿态下局部 X 恰好就是连杆轴向），
// 关节就退化成“只能绕自身轴自转”，手臂既伸不直也折不起来 —— 整条臂变成
// 一根刚性杆，IK 只在奇异位形上原地震荡（力和力矩上千牛，位置却不动）。
// ============================================================================

import * as THREE from 'three';
import { Pose } from '../core/math3d.js';
import { boxFromSize, cylinder } from '../core/shapes.js';
import { HingeJoint, DistanceJoint } from '../core/joints.js';

const AXIS_Y = new THREE.Vector3(0, 1, 0);
const AXIS_Z = new THREE.Vector3(0, 0, 1);

// 立柱底部偏航：局部 X → 世界 Y（轴竖直）
const YAW_TO_Y = new THREE.Quaternion().setFromAxisAngle(AXIS_Z, Math.PI / 2);
// 肩/肘俯仰：局部 X → 局部 Z（轴水平、垂直于连杆，手臂在 XY 平面内摆动）
const PITCH_TO_Z = new THREE.Quaternion().setFromAxisAngle(AXIS_Y, -Math.PI / 2);

const deg = THREE.MathUtils.degToRad;

// 连杆长度（可达范围 = |1.4 − 1.1| ~ 1.4 + 1.1 = 0.3 ~ 2.5 m）
const UP_LEN = 1.4;
const FORE_LEN = 1.1;

export const settings = {
    auto: true,          // 目标盒自动运动
    ikAlpha: 0.0003,     // IK 位置约束柔度（越小跟得越紧）
    poseAlpha: 0.006,    // 目标角（冗余正则化）柔度
    tx: 1.60, ty: 1.72, tz: 0.0,
    showTargetPath: true,
};

/**
 * 平面两连杆解析 IK：由夹爪相对肩部的目标点 (px, py) 反解
 * 肩角 ψ（上臂与水平线的夹角）与肘角 φ（前臂相对上臂的折角）。
 *
 * 只在**建场景时**用一次：让初始姿态与关节的 targetAngle 严格自洽，
 * 保证 t = 0 时不会“啪”地弹一下（运行时仍然是纯约束求解，没有解析 IK）。
 *
 * 余弦定理：d² = a² + b² + 2ab·cos φ（φ = 0 时两杆共线，d = a + b）
 * 正弦定理：sin β / b = sin(π − φ) / d，β = 肩点处「夹爪方向」与上臂的夹角
 * 取 ψ = γ − β 即“肘朝下”的那组解。
 */
function solvePlanarIK(px, py) {
    const reach = UP_LEN + FORE_LEN;
    const d = THREE.MathUtils.clamp(Math.hypot(px, py), Math.abs(UP_LEN - FORE_LEN) + 1e-3, reach - 1e-3);
    const cosPhi = (d * d - UP_LEN * UP_LEN - FORE_LEN * FORE_LEN) / (2 * UP_LEN * FORE_LEN);
    const phi = Math.acos(THREE.MathUtils.clamp(cosPhi, -1, 1));
    const sinBeta = (FORE_LEN * Math.sin(Math.PI - phi)) / d;
    const beta = Math.asin(THREE.MathUtils.clamp(sinBeta, -1, 1));
    const gamma = Math.atan2(py, px);
    return { psi: gamma - beta, phi };
}

export default {
    id: 'robot',
    name: '④ 机械臂 IK',
    tag: '图 17 · 过约束 / 冗余自由度',
    desc: `夹爪跟随灰色目标盒。全部由<b>同一个位置/角度投影</b>完成，
          没有专门的 IK 求解器：位置约束把夹爪拉向目标（Eqs. 2-10），
          铰链的<b>目标角度</b>约束（Eq. 22）负责消解冗余自由度，
          <b>角度限制</b>（Algorithm 3）保证关节不越界。<br>
          目标不可达时，系统自动收敛到“尽可能接近”的位姿——这就是论文说的
          <i>gracefully handles overconstrained problems</i>。`,
    camera: { position: [3.8, 2.6, 4.6], target: [0.9, 1.3, 0] },
    sim: { numSubsteps: 20, numPosIters: 1, showForces: true },

    create(harness) {
        const world = harness.world;
        const state = { objects: [], t: 0 };

        const addBody = (opts) => {
            const b = harness.addBody(opts);
            if (b.mesh) state.objects.push(b.mesh);
            return b;
        };

        // --- 立柱尺寸与肩点 ---
        const colLen = 1.3, colTop = 0.22 + colLen;   // 顶端高度 = 肩高
        const bracketX = 0.30;                        // 肩点前移，避免上臂扫过立柱本体
        const shoulder = new THREE.Vector3(bracketX, colTop, 0);

        // --- 底座（静态）---
        addBody({
            shape: cylinder(0.45, 0.22), isStatic: true,
            position: new THREE.Vector3(0, 0.11, 0), color: 0x3f4654,
        });

        // --- 立柱：底部偏航铰链，顶端与上臂铰接 ---
        const column = addBody({
            shape: boxFromSize(0.30, colLen, 0.30),
            mass: 3.0,
            position: new THREE.Vector3(0, 0.22 + colLen / 2, 0),
            color: 0x6ea8fe,
        });

        // 肩部托架（纯视觉，挂在立柱网格下；铰链轴就落在它的前端）
        const bracket = new THREE.Mesh(
            new THREE.BoxGeometry(bracketX + 0.12, 0.18, 0.24),
            new THREE.MeshStandardMaterial({ color: 0x4a5568, metalness: 0.6, roughness: 0.35 }));
        bracket.position.set((bracketX + 0.12) / 2 - 0.06, colLen / 2 - 0.09, 0);
        bracket.castShadow = true;
        column.mesh.add(bracket);

        // --- 目标轨迹：以肩点为原点的极坐标，全程落在可达范围内 ---
        // 半径 1.05~1.55 m（远小于 2.5 m 全伸），高度 ±0.35 m，方位角 ±26°。
        // 手臂因此**始终是折起来的**，绝不会落到“完全伸直 + 目标在轴向”的奇异位形上。
        const pathAt = (t, out = new THREE.Vector3()) => {
            const az = 0.45 * Math.sin(0.45 * t);          // 方位角（绕立柱摆动）
            const rho = 1.30 + 0.25 * Math.sin(0.7 * t);   // 平面内水平距离
            const dy = 0.35 * Math.sin(1.3 * t + 0.6);     // 相对肩部的高度
            return out.set(
                shoulder.x + rho * Math.cos(az),
                shoulder.y + dy,
                shoulder.z + rho * Math.sin(az));
        };

        // --- 由解析 IK 反解初始姿态，保证与关节 targetAngle 自洽 ---
        const restTarget = pathAt(0);
        const { psi, phi } = solvePlanarIK(restTarget.x - shoulder.x, restTarget.y - shoulder.y);

        const dirUpper = new THREE.Vector3(Math.cos(psi), Math.sin(psi), 0);
        const dirFore = new THREE.Vector3(Math.cos(psi + phi), Math.sin(psi + phi), 0);
        const elbow = shoulder.clone().addScaledVector(dirUpper, UP_LEN);

        // 上臂：左端（−X 端）与立柱顶端铰接
        const upper = addBody({
            shape: boxFromSize(UP_LEN, 0.22, 0.22),
            mass: 2.0,
            position: shoulder.clone().addScaledVector(dirUpper, UP_LEN / 2),
            quaternion: new THREE.Quaternion().setFromAxisAngle(AXIS_Z, psi),
            color: 0x8bd3dd,
        });

        // 前臂：左端与上臂右端铰接
        const fore = addBody({
            shape: boxFromSize(FORE_LEN, 0.18, 0.18),
            mass: 1.2,
            position: elbow.clone().addScaledVector(dirFore, FORE_LEN / 2),
            quaternion: new THREE.Quaternion().setFromAxisAngle(AXIS_Z, psi + phi),
            color: 0xffc857,
        });

        // 夹爪（纯视觉，挂在前臂网格下作为子节点）
        const gripper = new THREE.Group();
        for (const s of [-1, 1]) {
            const finger = new THREE.Mesh(
                new THREE.BoxGeometry(0.22, 0.05, 0.05),
                new THREE.MeshStandardMaterial({ color: 0xef476f, metalness: 0.4, roughness: 0.4 }));
            finger.position.set(0.11, s * 0.10, 0);
            finger.castShadow = true;
            gripper.add(finger);
        }
        gripper.position.set(FORE_LEN / 2, 0, 0);
        fore.mesh.add(gripper);

        // --- 关节 ---
        // A：偏航（轴 = 世界 Y）—— 关节局部 X 轴转到 Y。
        //    这是整条臂唯一的冗余自由度：目标绕立柱摆动时由它跟上。
        //    它的目标角约束比其他关节更软，免得把手臂硬拽回正面。
        const jointA = new HingeJoint(null, column,
            new Pose(new THREE.Vector3(0, 0.22, 0), YAW_TO_Y),
            new Pose(new THREE.Vector3(0, -colLen / 2, 0), YAW_TO_Y),
            {
                minAngle: deg(-170), maxAngle: deg(170),
                targetAngle: 0.0, targetCompliance: settings.poseAlpha * 2.5,
                damping: 0.05,
            });

        // B：肩部俯仰（轴 = 水平且垂直于连杆，手臂在 XY 平面内摆动）
        const jointB = new HingeJoint(column, upper,
            new Pose(new THREE.Vector3(bracketX, colLen / 2, 0), PITCH_TO_Z),
            new Pose(new THREE.Vector3(-UP_LEN / 2, 0, 0), PITCH_TO_Z),
            {
                minAngle: deg(-80), maxAngle: deg(45),
                targetAngle: psi, targetCompliance: settings.poseAlpha,
                damping: 0.05,
            });

        // C：肘部俯仰（轴同上）
        const jointC = new HingeJoint(upper, fore,
            new Pose(new THREE.Vector3(UP_LEN / 2, 0, 0), PITCH_TO_Z),
            new Pose(new THREE.Vector3(-FORE_LEN / 2, 0, 0), PITCH_TO_Z),
            {
                minAngle: deg(-10), maxAngle: deg(140),
                targetAngle: phi, targetCompliance: settings.poseAlpha,
                damping: 0.05,
            });

        jointB.meta = { role: 'shoulder' };
        for (const j of [jointA, jointB, jointC]) {
            world.addJoint(j);
            harness.watchJoint(j, { torque: true });
        }

        // --- 目标盒（运动学：静态体，由脚本直接给定位姿）---
        const target = addBody({
            shape: boxFromSize(0.26, 0.26, 0.26),
            isStatic: true,
            position: restTarget.clone(),
            color: 0x9aa4b2,
        });

        // 目标轨迹的参考曲线（视觉）
        const pathPts = [];
        const T = 14.0;
        for (let i = 0; i <= 260; i++) pathPts.push(pathAt((i / 260) * T));
        const pathLine = new THREE.Line(
            new THREE.BufferGeometry().setFromPoints(pathPts),
            new THREE.LineDashedMaterial({ color: 0x445069, dashSize: 0.12, gapSize: 0.09 }));
        pathLine.computeLineDistances();
        harness.addObject(pathLine);
        state.objects.push(pathLine);

        // --- IK 约束：把夹爪锚点拉向目标盒（论文 §3.3.1 的位置投影）---
        const ik = new DistanceJoint(
            target, fore,
            new Pose(new THREE.Vector3(0, 0, 0)),
            new Pose(new THREE.Vector3(FORE_LEN / 2, 0, 0)),  // 前臂末端 = 夹爪
            { isSpring: true, restLength: 0.0, compliance: settings.ikAlpha, damping: 3.0 });
        ik.meta = { role: 'ik', tipLocal: new THREE.Vector3(FORE_LEN / 2, 0, 0) };
        world.addJoint(ik);

        return {
            preStep(dt) {
                state.t += dt;
                if (settings.auto) pathAt(state.t, target.pose.p);
                else target.pose.p.set(settings.tx, settings.ty, settings.tz);
                // 运动学体：直接给定速度，让接触/摩擦层看到合理的速度值
                target.vel.set(0, 0, 0);
                target.omega.set(0, 0, 0);
            },

            onGUI(gui) {
                gui.add(settings, 'auto').name('目标自动运动');
                gui.add(settings, 'ikAlpha', 0.00002, 0.005, 0.00002).name('IK 柔度 α')
                    .onChange((v) => { ik.compliance = v; });
                gui.add(settings, 'poseAlpha', 0.0005, 0.05, 0.0005).name('目标角柔度 α（正则化）')
                    .onChange((v) => {
                        jointA.targetCompliance = v * 2.5;   // 偏航更软，见关节 A 注释
                        jointB.targetCompliance = v;
                        jointC.targetCompliance = v;
                    });
                const f = gui.addFolder('手动目标位置');
                for (const k of ['tx', 'ty', 'tz']) {
                    f.add(settings, k, -4, 4, 0.05).name(k.toUpperCase())
                        .onChange(() => { settings.auto = false; harness.refreshGUI(); });
                }
                f.add({ go: () => { settings.auto = true; harness.refreshGUI(); } }, 'go')
                    .name('恢复自动');
                gui.add(settings, 'showTargetPath').name('显示目标轨迹')
                    .onChange((v) => { pathLine.visible = v; });
            },

            dispose() { harness.removeObjects(state.objects); },
        };
    },

    buildInfo(harness) {
        let html = '<b>IK 状态</b>\n';
        for (const j of harness.world.joints) {
            if (!j.meta || j.meta.role !== 'ik') continue;
            j.updateGlobalPoses();
            const tip = j.bodyB.toWorld(j.meta.tipLocal.clone());
            const tgt = j.globalPoseA.p;
            const err = tip.distanceTo(tgt);
            html += `目标盒 (${tgt.x.toFixed(2)}, ${tgt.y.toFixed(2)}, ${tgt.z.toFixed(2)})\n`;
            html += `夹  爪 (${tip.x.toFixed(2)}, ${tip.y.toFixed(2)}, ${tip.z.toFixed(2)})\n`;
            html += `跟随误差 ${err.toFixed(4)} m `
                + `<span class="${err < 0.03 ? 'ok' : 'warn'}">`
                + `${err < 0.03 ? '✓ 可达' : '△ 过约束 → 取最接近位姿'}</span>\n`;
        }
        for (const j of harness.world.joints) {
            if (j.meta && j.meta.role === 'shoulder') {
                html += `肩部角度 ${(j.getCurrentAngle() * 180 / Math.PI).toFixed(1)}° `
                    + `限制 [${(j.minAngle * 180 / Math.PI).toFixed(0)}, ${(j.maxAngle * 180 / Math.PI).toFixed(0)}]°\n`;
            }
        }
        html += `\nIK 柔度 α=${ikAlphaOf(harness).toFixed(5)} m/N（越小跟得越紧）`;
        return html;
    },
};

/** 从世界里取 IK 约束的当前柔度，仅用于读数显示 */
function ikAlphaOf(harness) {
    for (const j of harness.world.joints) if (j.meta && j.meta.role === 'ik') return j.compliance;
    return 0;
}

// ============================================================================
// 场景 ③：关节类型展示 —— 论文图 7
//
// 图 7：「用我们的方法可以创建各种关节类型，带目标角度、软硬不同的关节限制。」
//
// 从左到右：
//   1. 门     —— 铰链关节（§3.4.1）：轴对齐 Eq. 21 + 角度限制 Algorithm 3
//                + 目标角度 Eq. 22（柔度可调 = 力矩可调）
//   2. 滑台   —— 棱柱关节（§3.4.2）：沿滑动轴的自由度 + 逐轴上下限 + 马达
//   3. 球窝   —— 球窝关节（§3.4.1）：swing / twist 限制分别独立设置
//                （论文 Eqs. 23-25 的扭转解耦）
//
// 重点体验：把「限制柔度」从 0 调到 0.01，硬限制会变成软限制；
//          把「目标角柔度」调大，马达就“没力气”，托不住门的重力。
// ============================================================================

import * as THREE from 'three';
import { Pose } from '../core/math3d.js';
import { boxFromSize, sphere } from '../core/shapes.js';
import { HingeJoint, PrismaticJoint, SphericalJoint } from '../core/joints.js';

export const settings = {
    doorTargetDeg: 0.0,      // 目标角度（度）
    doorTargetAlpha: 0.0005, // 目标角柔度 m/N
    doorLimitDeg: 85.0,      // 铰链限制角（度）
    doorLimitAlpha: 0.0,     // 限制柔度（0 = 硬限制）
    slideAlpha: 0.0,
    slideAuto: true,         // 滑台自动往复
    swingLimitDeg: 50.0,
    twistLimitDeg: 30.0,
    swingAlpha: 0.0,
    twistAlpha: 0.002,       // 扭转限制更软
};

// 铰链轴 = 世界 Y（绕 Z 转 +90° 把关节 X 轴映射到 Y）
const AXIS_TO_Y = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 2);

export default {
    id: 'jointTypes',
    name: '③ 关节类型展示',
    tag: '图 7 · 铰链 / 棱柱 / 球窝',
    desc: `从左到右：<b>门</b>（铰链 + 角度限制 Algorithm 3 + 目标角度 Eq. 22）、
          <b>滑台</b>（棱柱关节 + 逐轴限制 + 位置马达）、
          <b>球窝</b>（swing / twist 限制独立解耦，Eqs. 23-25）。<br>
          调节「限制柔度」把硬限制变成软限制；调大目标角柔度，
          马达就托不住门的重量——XPBD 里<b>柔度直接就是物理量</b>。`,
    camera: { position: [1.5, 4.2, 12], target: [0, 1.6, 0] },
    sim: { numSubsteps: 20, numPosIters: 1, showForces: true },

    create(harness) {
        const world = harness.world;
        const state = { objects: [], joints: {} };

        const addBody = (opts) => {
            const b = harness.addBody(opts);
            if (b.mesh) state.objects.push(b.mesh);
            return b;
        };
        const addStatic = (shape, pos, color = 0x3f4654) => {
            const b = harness.addBody({ shape, isStatic: true, position: pos, color });
            if (b.mesh) state.objects.push(b.mesh);
            return b;
        };

        // ===================================================================
        // 1) 门：铰链关节 + 角度限制 + 目标角度
        // ===================================================================
        const doorX = -4.2;              // 铰链（门的左边缘）
        addStatic(boxFromSize(0.3, 2.8, 0.3), new THREE.Vector3(doorX - 0.15, 1.4, 0));
        addStatic(boxFromSize(2.4, 0.25, 0.3), new THREE.Vector3(doorX + 1.0, 2.75, 0));

        const door = addBody({
            shape: boxFromSize(1.8, 2.4, 0.09),
            mass: 8.0,
            position: new THREE.Vector3(doorX + 0.9, 1.2, 0),
            color: 0xb98a5a,
        });
        const doorHinge = new HingeJoint(null, door,
            new Pose(new THREE.Vector3(doorX, 1.2, 0), AXIS_TO_Y),
            new Pose(new THREE.Vector3(-0.9, 0, 0), AXIS_TO_Y),
            {
                compliance: 0.0,
                limitCompliance: settings.doorLimitAlpha,
                minAngle: -THREE.MathUtils.degToRad(settings.doorLimitDeg),
                maxAngle: THREE.MathUtils.degToRad(settings.doorLimitDeg),
                targetAngle: THREE.MathUtils.degToRad(settings.doorTargetDeg),
                targetCompliance: settings.doorTargetAlpha,
            });
        world.addJoint(doorHinge);
        state.joints.door = doorHinge;
        harness.watchJoint(doorHinge);

        // ===================================================================
        // 2) 滑台：棱柱关节 + 逐轴限制 + 位置马达
        //    关节坐标系原点设在轨道中心；滑块上的关节原点与之重合时 offset = 0。
        //    于是限制可以直接写成 [[±行程], [0,0], [0,0]]（另两轴锁死）。
        // ===================================================================
        const railY = 1.2;
        addStatic(boxFromSize(3.6, 0.16, 0.5), new THREE.Vector3(0, railY, 0));
        const slider = addBody({
            shape: boxFromSize(0.42, 0.42, 0.42),
            mass: 2.0,
            position: new THREE.Vector3(0, railY + 0.29, 0),
            color: 0x8bd3dd,
        });
        const slideJoint = new PrismaticJoint(null, slider,
            new Pose(new THREE.Vector3(0, railY, 0)),
            new Pose(new THREE.Vector3(0, 0.29, 0)),
            {
                compliance: 0.0,
                limits: [[-1.5, 1.5], [0, 0], [0, 0]],
                targetSlide: 0.0,
                targetCompliance: settings.slideAlpha,
                motorVelocity: 0.0,
            });
        world.addJoint(slideJoint);
        state.joints.slide = slideJoint;
        harness.watchJoint(slideJoint);

        // ===================================================================
        // 3) 球窝：swing / twist 限制独立设置
        // ===================================================================
        const postTop = new THREE.Vector3(4.2, 2.6, 0);
        addStatic(boxFromSize(0.28, 2.6, 0.28), new THREE.Vector3(4.2, 1.3, 0));

        const rodLen = 1.7;
        const rod = addBody({
            shape: boxFromSize(0.12, rodLen, 0.12),
            mass: 1.5,
            position: postTop.clone().add(new THREE.Vector3(0, -rodLen / 2, 0)),
            color: 0xffc857,
        });
        const ball = new THREE.Mesh(
            new THREE.SphereGeometry(0.16, 20, 14),
            new THREE.MeshStandardMaterial({ color: 0xef476f, metalness: 0.4, roughness: 0.35 }));
        ball.position.copy(postTop);
        harness.addObject(ball);
        state.objects.push(ball);

        // 关节局部 X 轴必须沿**杆的轴向**（此处世界 Y）：
        //   swing 限制的是两侧 X 轴的夹角 → 只对摆动敏感；
        //   twist 则是在垂直于 X 的平面内、把 Y 轴投影后比较 → 只对自转敏感。
        // 若把 X 轴放成世界 X，绕杆自转也会改变 X 轴夹角，swing 会与 twist 串扰。
        const ballJoint = new SphericalJoint(null, rod,
            new Pose(postTop.clone(), AXIS_TO_Y),
            new Pose(new THREE.Vector3(0, rodLen / 2, 0), AXIS_TO_Y),
            {
                compliance: 0.0,
                swingLimit: [-THREE.MathUtils.degToRad(settings.swingLimitDeg),
                             THREE.MathUtils.degToRad(settings.swingLimitDeg)],
                swingCompliance: settings.swingAlpha,
                twistLimit: [-THREE.MathUtils.degToRad(settings.twistLimitDeg),
                             THREE.MathUtils.degToRad(settings.twistLimitDeg)],
                twistCompliance: settings.twistAlpha,
                damping: 0.05,
            });
        world.addJoint(ballJoint);
        state.joints.ball = ballJoint;

        // 初始：给它一个侧向摆动 + 绕自身轴的扭转，把两个限制都激发出来
        rod.omega.set(0.0, 4.0, 2.5);

        const deg = THREE.MathUtils.degToRad;
        const clampDeg = (v) => THREE.MathUtils.clamp(v, -179, 179);

        return {
            preStep(h) {
                // 滑台自动往复：到达行程端点就反向
                if (settings.slideAuto) {
                    const d = slideJoint.targetSlide;
                    if (d > 1.45) slideJoint.motorVelocity = -1.2;
                    else if (d < -1.45) slideJoint.motorVelocity = 1.2;
                    else if (slideJoint.motorVelocity === 0.0) slideJoint.motorVelocity = 1.2;
                } else {
                    slideJoint.motorVelocity = 0.0;
                }
            },

            onGUI(gui) {
                const f1 = gui.addFolder('门：铰链 + 限制 + 目标角');
                f1.add(settings, 'doorTargetDeg', -85, 85, 1).name('目标角度 (°)')
                    .onChange((v) => { doorHinge.targetAngle = deg(v); });
                f1.add(settings, 'doorTargetAlpha', 0.0, 0.02, 0.0001).name('目标角柔度 α')
                    .onChange((v) => { doorHinge.targetCompliance = v; });
                f1.add(settings, 'doorLimitDeg', 5, 179, 1).name('限制角度 (°)')
                    .onChange((v) => {
                        doorHinge.minAngle = -deg(v);
                        doorHinge.maxAngle = deg(v);
                    });
                f1.add(settings, 'doorLimitAlpha', 0.0, 0.02, 0.0001).name('限制柔度 α')
                    .onChange((v) => { doorHinge.limitCompliance = v; });

                const f2 = gui.addFolder('滑台：棱柱 + 马达');
                f2.add(settings, 'slideAuto').name('自动往复');
                f2.add(settings, 'slideAlpha', 0.0, 0.02, 0.0001).name('位置柔度 α')
                    .onChange((v) => { slideJoint.compliance = v; });
                f2.add({ go: () => { slideJoint.targetSlide = 0.0; } }, 'go').name('复位滑台');

                const f3 = gui.addFolder('球窝：swing / twist 限制');
                f3.add(settings, 'swingLimitDeg', 0, 170, 1).name('swing 限制 (°)')
                    .onChange((v) => { ballJoint.swingLimit = [-deg(v), deg(v)]; });
                f3.add(settings, 'swingAlpha', 0.0, 0.02, 0.0001).name('swing 柔度 α')
                    .onChange((v) => { ballJoint.swingCompliance = v; });
                f3.add(settings, 'twistLimitDeg', 0, 170, 1).name('twist 限制 (°)')
                    .onChange((v) => { ballJoint.twistLimit = [-deg(v), deg(v)]; });
                f3.add(settings, 'twistAlpha', 0.0, 0.02, 0.0001).name('twist 柔度 α')
                    .onChange((v) => { ballJoint.twistCompliance = v; });
                f3.add({ go: () => { rod.omega.set(0, 4, 2.5); rod.vel.set(0, 0, 0); } }, 'go')
                    .name('↻ 重新激发摆动');
            },

            dispose() { harness.removeObjects(state.objects); },
        };
    },

    buildInfo(harness) {
        const j = harness.world.joints;
        const door = j.find((x) => x instanceof HingeJoint);
        const ball = j.find((x) => x instanceof SphericalJoint);
        const slide = j.find((x) => x instanceof PrismaticJoint);
        const deg = (r) => (r * 180 / Math.PI).toFixed(1);

        let html = '<b>关节状态</b>\n';
        if (door) {
            html += `门  角度=${deg(door.getCurrentAngle())}°  `
                + `限制=[${deg(door.minAngle)}, ${deg(door.maxAngle)}]°  `
                + `力矩 τ=|λ|/h²=${door.lastTorque.toFixed(2)} N·m\n`;
        }
        if (slide) {
            html += `滑台 位移=${slide.targetSlide.toFixed(3)} m  `
                + `推力=${slide.debugForce.length().toFixed(2)} N\n`;
        }
        if (ball && ball.bodyB) {
            // swing 角 = 两关节 X 轴（此处即世界 Y）的夹角
            ball.updateGlobalPoses();
            const a1 = new THREE.Vector3(1, 0, 0).applyQuaternion(ball.globalPoseA.q);
            const a2 = new THREE.Vector3(1, 0, 0).applyQuaternion(ball.globalPoseB.q);
            const swing = Math.acos(THREE.MathUtils.clamp(a1.dot(a2), -1, 1));
            html += `球窝 swing=${deg(swing)}°  `
                + `twist 限制=±${deg(ball.twistLimit[1])}°\n`;
        }
        return html;
    },
};

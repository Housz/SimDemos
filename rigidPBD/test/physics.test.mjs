// ============================================================================
// XPBD 刚体核心库的无头数值测试
// 用 node --experimental-loader ./loader.mjs test.mjs 运行
// ============================================================================
import * as THREE from 'three';

const CORE = new URL('../js/core/', import.meta.url).href;
const { RigidBody } = await import(CORE + 'rigidBody.js');
const { World } = await import(CORE + 'world.js');
const { sphere, box, plane, cylinder, capsule } = await import(CORE + 'shapes.js');
const { Pose } = await import(CORE + 'math3d.js');
const J = await import(CORE + 'joints.js');

let pass = 0, fail = 0;
function check(name, cond, detail = '') {
    if (cond) { pass++; console.log(`  ✓ ${name} ${detail}`); }
    else { fail++; console.log(`  ✗ ${name} ${detail}`); }
}
function approx(a, b, tol) { return Math.abs(a - b) <= tol; }

const GRAV = new THREE.Vector3(0, -10, 0);

// ---------------------------------------------------------------------------
console.log('\n[1] 自由落体：位置 x = ½gt², 速度 v = gt');
{
    const w = new World({ gravity: GRAV, numSubsteps: 20 });
    const ball = w.createBody({ shape: sphere(0.1), mass: 1, position: new THREE.Vector3(0, 100, 0) });
    const dt = 1 / 60;
    for (let i = 0; i < 60; i++) w.step(dt);
    const t = 1.0;
    check('1 秒后位移', approx(ball.pose.p.y, 100 - 0.5 * 10 * t * t, 0.02), `y=${ball.pose.p.y.toFixed(4)} 期望 ${(100 - 5).toFixed(4)}`);
    check('1 秒后速度', approx(ball.vel.y, -10 * t, 0.02), `v=${ball.vel.y.toFixed(4)} 期望 -10`);
}

// ---------------------------------------------------------------------------
console.log('\n[2] 球静止在平面上：应停于 y = r，无穿透（论文 Fig 13 的“温和推起”）');
{
    const w = new World({ gravity: GRAV, numSubsteps: 20 });
    w.createBody({ shape: plane(new THREE.Vector3(0, 1, 0), 0), isStatic: true });
    const ball = w.createBody({
        shape: sphere(0.5), mass: 1, position: new THREE.Vector3(0, 2.0, 0),
        mesh: null,
    });
    ball.restitution = 0.0;
    for (let i = 0; i < 240; i++) w.step(1 / 60);
    check('高度收敛到半径', approx(ball.pose.p.y, 0.5, 0.01), `y=${ball.pose.p.y.toFixed(5)}`);
    check('速度收敛到 0', Math.abs(ball.vel.y) < 0.05, `v=${ball.vel.y.toFixed(5)}`);
    check('无穿透', ball.pose.p.y >= 0.5 - 0.01, `y=${ball.pose.p.y.toFixed(5)}`);
}

// ---------------------------------------------------------------------------
console.log('\n[3] 初始穿透：论文 Fig 13 —— 不应产生弹跳速度');
{
    const w = new World({ gravity: GRAV, numSubsteps: 20 });
    w.createBody({ shape: plane(new THREE.Vector3(0, 1, 0), 0), isStatic: true });
    const ball = w.createBody({ shape: sphere(0.5), mass: 1, position: new THREE.Vector3(0, 0.2, 0) });
    for (let i = 0; i < 120; i++) w.step(1 / 60);
    check('被温和推起到表面', approx(ball.pose.p.y, 0.5, 0.02), `y=${ball.pose.p.y.toFixed(4)}`);
    check('没有大速度弹飞', Math.abs(ball.vel.y) < 1.0, `v=${ball.vel.y.toFixed(4)}`);
}

// ---------------------------------------------------------------------------
console.log('\n[4] 恢复系数 e=1：弹回接近原高度（论文 Fig 12 的冲量传递基础）');
{
    const w = new World({ gravity: GRAV, numSubsteps: 40 });
    w.createBody({ shape: plane(new THREE.Vector3(0, 1, 0), 0), isStatic: true });
    const ball = w.createBody({ shape: sphere(0.5), mass: 1, position: new THREE.Vector3(0, 2.0, 0) });
    ball.restitution = 1.0;
    let maxAfter = 0;
    let hit = false;
    for (let i = 0; i < 400; i++) {
        w.step(1 / 60);
        if (ball.pose.p.y < 0.55) hit = true;
        if (hit) maxAfter = Math.max(maxAfter, ball.pose.p.y);
    }
    check('弹回高度 > 1.5 (初高 2.0)', maxAfter > 1.5, `最高 ${maxAfter.toFixed(3)}`);
}

// ---------------------------------------------------------------------------
console.log('\n[5] 距离关节 + 柔度：伸长量 = 力 × 柔度（论文 Fig 4，α=0.01 m/N）');
{
    const w = new World({ gravity: GRAV, numSubsteps: 20 });
    const anchor = w.createBody({ shape: sphere(0.01), isStatic: true, position: new THREE.Vector3(0, 10, 0) });
    const m = 1.0;
    // 初始位置取“恰好等于原长”：锚点(0,10,0) 到 体上锚点 (y+0.25) 的距离 = 1.0 → y = 8.75
    const body = w.createBody({ shape: box(0.25, 0.25, 0.25), mass: m, position: new THREE.Vector3(0, 8.75, 0) });
    // 加线性关节阻尼（论文 Eq. 32）使无阻尼振荡收敛到静平衡，否则采样时刻的伸长量无意义
    const joint = new J.DistanceJoint(anchor, body,
        new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0.25, 0),
        { compliance: 0.01, restLength: 1.0, isSpring: true, damping: 4.0 });
    w.addJoint(joint);
    for (let i = 0; i < 300; i++) w.step(1 / 60);
    const dist = joint.getDistance();
    // 静止时弹力 = mg = 10 N，柔度 α=0.01 m/N → 伸长 0.1 m
    check('伸长量 ≈ 0.1 m', approx(dist, 1.1, 0.01), `L=${dist.toFixed(5)} 期望 1.1`);
    check('约束力 ≈ mg = 10 N', approx(joint.lastForce, 10.0, 0.5), `f=${joint.lastForce.toFixed(3)}`);
}

// ---------------------------------------------------------------------------
console.log('\n[6] 单摆（铰链关节）+ 能量守恒：论文 Fig 8/9');
{
    const w = new World({ gravity: GRAV, numSubsteps: 40 });
    const anchor = w.createBody({ shape: sphere(0.02), isStatic: true, position: new THREE.Vector3(0, 5, 0) });
    const rod = w.createBody({ shape: box(0.02, 1.0, 0.02), mass: 1, position: new THREE.Vector3(1.0, 5, 0) });
    // 铰链轴 = 世界 Z，位于杆的左端
    const hingeA = new Pose(new THREE.Vector3(0, 0, 0));
    const hingeB = new Pose(new THREE.Vector3(-1.0, 0, 0));
    hingeA.q.setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 2);
    hingeB.q.setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 2);
    const joint = new J.HingeJoint(anchor, rod, hingeA, hingeB, { damping: 0.0 });
    w.addJoint(joint);
    const e0 = (() => { w.step(1 / 600); return w.energy; })();
    for (let i = 0; i < 600; i++) w.step(1 / 600);
    const e1 = w.energy;
    check('铰链点保持重合', joint.globalPoseA.p.distanceTo(joint.globalPoseB.p) < 0.01,
        `d=${joint.globalPoseA.p.distanceTo(joint.globalPoseB.p).toFixed(5)}`);
    check('能量守恒 (10 秒内漂移 < 5%)', Math.abs((e1 - e0) / e0) < 0.05,
        `E0=${e0.toFixed(4)} E1=${e1.toFixed(4)} 漂移 ${(100 * (e1 - e0) / e0).toFixed(2)}%`);
}

// ---------------------------------------------------------------------------
console.log('\n[7] 盒-盒堆叠：3 个盒子应稳定堆叠（论文 Table 1 的 Boxes 场景）');
{
    const w = new World({ gravity: GRAV, numSubsteps: 20 });
    w.createBody({ shape: plane(new THREE.Vector3(0, 1, 0), 0), isStatic: true });
    const boxes = [];
    for (let i = 0; i < 3; i++) {
        const b = w.createBody({
            shape: box(0.5, 0.5, 0.5), mass: 1,
            position: new THREE.Vector3(0, 0.5 + i * 1.0 + 0.02, 0),
        });
        b.staticFriction = 1.0; b.dynamicFriction = 0.8; b.restitution = 0.0;
        boxes.push(b);
    }
    for (let i = 0; i < 600; i++) w.step(1 / 60);
    const heights = boxes.map((b) => b.pose.p.y);
    check('盒子 0 高度 ≈ 0.5', approx(heights[0], 0.5, 0.03), `y=${heights[0].toFixed(4)}`);
    check('盒子 1 高度 ≈ 1.5', approx(heights[1], 1.5, 0.05), `y=${heights[1].toFixed(4)}`);
    check('盒子 2 高度 ≈ 2.5', approx(heights[2], 2.5, 0.08), `y=${heights[2].toFixed(4)}`);
    const drift = boxes.map((b) => Math.hypot(b.pose.p.x, b.pose.p.z));
    check('无明显侧向漂移', Math.max(...drift) < 0.05, `max|xz|=${Math.max(...drift).toFixed(4)}`);
}

// ---------------------------------------------------------------------------
console.log('\n[8] 盒-平面：立方体倾斜落地应稳定（多点接触）');
{
    const w = new World({ gravity: GRAV, numSubsteps: 20 });
    const ground = w.createBody({ shape: plane(new THREE.Vector3(0, 1, 0), 0), isStatic: true });
    ground.staticFriction = 1.0; ground.dynamicFriction = 1.0;
    const b = w.createBody({ shape: box(0.5, 0.5, 0.5), mass: 1, position: new THREE.Vector3(0, 1.0, 0) });
    b.pose.q.setFromAxisAngle(new THREE.Vector3(1, 0, 0), 0.3);
    b.staticFriction = 1.0; b.dynamicFriction = 1.0; b.restitution = 0.0;
    for (let i = 0; i < 600; i++) w.step(1 / 60);
    const corners = [];
    for (let i = 0; i < 8; i++) {
        const c = new THREE.Vector3((i & 1) ? 0.5 : -0.5, (i & 2) ? 0.5 : -0.5, (i & 4) ? 0.5 : -0.5);
        corners.push(c.applyQuaternion(b.pose.q).add(b.pose.p).y);
    }
    check('最低角 ≈ 0（贴地）', approx(Math.min(...corners), 0, 0.02), `min=${Math.min(...corners).toFixed(4)}`);
    check('最高角 ≈ 1（未穿透）', approx(Math.max(...corners), 1.0, 0.05), `max=${Math.max(...corners).toFixed(4)}`);
    check('旋转量已停止', b.omega.length() < 0.1, `|ω|=${b.omega.length().toFixed(4)}`);
}

// ---------------------------------------------------------------------------
console.log('\n[9] 球-盒运动学一致性 & NaN 检查（长时间稳定性）');
{
    const w = new World({ gravity: GRAV, numSubsteps: 20 });
    w.createBody({ shape: plane(new THREE.Vector3(0, 1, 0), 0), isStatic: true });
    w.createBody({ shape: box(1, 0.2, 1), mass: 5, position: new THREE.Vector3(0, 0.2, 0) });
    for (let i = 0; i < 8; i++) {
        const b = w.createBody({
            shape: sphere(0.2 + 0.05 * i), mass: 0.5,
            position: new THREE.Vector3((Math.random() - 0.5) * 0.5, 2 + i * 0.6, (Math.random() - 0.5) * 0.5),
        });
        b.restitution = 0.2;
    }
    let nan = false;
    for (let i = 0; i < 1200; i++) {
        w.step(1 / 60);
        for (const b of w.bodies) {
            if (!isFinite(b.pose.p.x + b.pose.p.y + b.pose.p.z + b.vel.x + b.omega.x)) nan = true;
        }
    }
    check('20 秒 8 球 + 平板无 NaN/发散', !nan);
}

// ---------------------------------------------------------------------------
console.log('\n[10] 圆柱（硬币）落在平面上：论文 Fig 15');
{
    const w = new World({ gravity: GRAV, numSubsteps: 20 });
    const ground = w.createBody({ shape: plane(new THREE.Vector3(0, 1, 0), 0), isStatic: true });
    ground.staticFriction = 1.0; ground.dynamicFriction = 0.8;
    const coin = w.createBody({ shape: cylinder(0.15, 0.02), mass: 0.01, position: new THREE.Vector3(0, 1.0, 0) });
    coin.pose.q.setFromAxisAngle(new THREE.Vector3(1, 0, 0), 1.2);
    coin.staticFriction = 1.0; coin.dynamicFriction = 0.8;
    for (let i = 0; i < 900; i++) w.step(1 / 60);
    check('硬币未穿透地面', coin.pose.p.y > 0.0, `y=${coin.pose.p.y.toFixed(4)}`);
    check('硬币最终趋于静止', coin.vel.length() < 0.5 && coin.omega.length() < 2.0,
        `|v|=${coin.vel.length().toFixed(3)} |ω|=${coin.omega.length().toFixed(3)}`);
}

// ---------------------------------------------------------------------------
console.log('\n[11] 静摩擦：斜面角 0.3 rad，μs=1.0 > tanθ → 盒子不下滑');
{
    const th = 0.3;
    const w = new World({ gravity: GRAV, numSubsteps: 20 });
    const n = new THREE.Vector3(Math.sin(th), Math.cos(th), 0);
    const ground = w.createBody({ shape: plane(n, 0), isStatic: true });
    ground.staticFriction = 1.0; ground.dynamicFriction = 1.0;
    const b = w.createBody({ shape: box(0.5, 0.5, 0.5), mass: 1, position: n.clone().multiplyScalar(0.5) });
    // 绕 Z 转 −θ 才能把局部 Y 轴转到 (sinθ, cosθ, 0) = 平面法线（转 +θ 得到的是 (−sinθ,cosθ,0)，方向相反）
    b.pose.q.setFromAxisAngle(new THREE.Vector3(0, 0, 1), -th);
    b.staticFriction = 1.0; b.dynamicFriction = 1.0; b.restitution = 0.0;
    const p0 = b.pose.p.clone();
    for (let i = 0; i < 300; i++) w.step(1 / 60);
    // 沿坡面的切向位移（n 方向的分量应近似不变）
    const tangent = new THREE.Vector3(Math.cos(th), -Math.sin(th), 0);
    const slide = Math.abs(b.pose.p.clone().sub(p0).dot(tangent));
    check('沿斜面滑移 < 0.02 m', slide < 0.02, `滑移=${slide.toFixed(5)} 阈值 tanθ=${Math.tan(th).toFixed(3)}<μs=1.0`);
    check('未穿透斜面', b.pose.p.dot(n) > 0.45, `n·p=${b.pose.p.dot(n).toFixed(4)}`);
}

// ---------------------------------------------------------------------------
console.log('\n[12] 动摩擦：水平面上以初速 5 m/s 滑行的滑块（论文 Eq. 31）');
{
    const w = new World({ gravity: GRAV, numSubsteps: 20 });
    const ground = w.createBody({ shape: plane(new THREE.Vector3(0, 1, 0), 0), isStatic: true });
    ground.staticFriction = 1.0; ground.dynamicFriction = 0.5;
    const b = w.createBody({ shape: box(0.5, 0.5, 0.5), mass: 1, position: new THREE.Vector3(0, 0.5, 0) });
    b.staticFriction = 1.0; b.dynamicFriction = 0.5; b.restitution = 0.0;
    b.vel.set(5, 0, 0);
    for (let i = 0; i < 600; i++) {
        w.step(1 / 60);
        if (Math.abs(b.vel.x) < 1e-3) break;
    }
    // 库仑动摩擦：滑行距离 = v²/(2·μd·g) = 25/(2·0.5·10) = 2.5 m，且最终必须停下
    const d = b.pose.p.x;
    check('滑块最终停下', b.vel.length() < 1e-2, `vx=${b.vel.x.toFixed(5)}`);
    check('滑行距离 ≈ v²/(2μd·g) = 2.5 m', approx(d, 2.5, 0.5), `滑行 ${d.toFixed(3)} m`);
}

// ---------------------------------------------------------------------------
console.log('\n[13] 球-胶囊轨道（论文图 12/13 的弹珠轨道基本单元）');
{
    const w = new World({ gravity: GRAV, numSubsteps: 20 });
    // 静态水平胶囊当作轨道
    const track = w.createBody({ shape: capsule(0.05, 4.0), isStatic: true, position: new THREE.Vector3(0, 0, 0) });
    track.staticFriction = 0.5; track.dynamicFriction = 0.4;
    // 胶囊沿 Y 轴，绕 Z 转 90° 变成沿 X 的水平轨道
    track.pose.q.setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 2);
    const ball = w.createBody({ shape: sphere(0.1), mass: 1, position: new THREE.Vector3(0, 0.16, 0) });
    ball.restitution = 0.0; ball.staticFriction = 0.5; ball.dynamicFriction = 0.4;
    for (let i = 0; i < 300; i++) w.step(1 / 60);
    check('球停在轨道表面 y ≈ 0.15', approx(ball.pose.p.y, 0.15, 0.01), `y=${ball.pose.p.y.toFixed(5)}`);
    check('未穿透轨道', ball.pose.p.y > 0.14, `y=${ball.pose.p.y.toFixed(5)}`);
}

// ---------------------------------------------------------------------------
console.log('\n[14] 弹性碰撞传递（论文图 12）：等高球心正碰，动量应传递到最右侧');
{
    // 零重力、无地面：只观察冲量链本身（否则球会在 2 秒内落到 y=0 的地面上互相干扰）
    const w = new World({ gravity: new THREE.Vector3(0, 0, 0), numSubsteps: 40 });
    const R = 0.1;
    const gap = 0.006; // 三球之间留小缝，使冲量逐子步依次传递（论文图 12 的“逐个传递”）
    const balls = [];
    for (let i = 0; i < 3; i++) {
        const b = w.createBody({ shape: sphere(R), mass: 1, position: new THREE.Vector3(i * (2 * R + gap), 5, 0) });
        b.restitution = 1.0; b.staticFriction = 0; b.dynamicFriction = 0;
        balls.push(b);
    }
    const hitter = w.createBody({ shape: sphere(R), mass: 1, position: new THREE.Vector3(-1.0, 5, 0) });
    hitter.restitution = 1.0; hitter.staticFriction = 0; hitter.dynamicFriction = 0;
    hitter.vel.set(4, 0, 0);
    for (let i = 0; i < 120; i++) w.step(1 / 60);
    const last = balls[2].vel.x;
    check('最右球获得接近全部的动量', last > 3.0, `v_last=${last.toFixed(4)} (入射 4 m/s)`);
    check('入射球基本停下', Math.abs(hitter.vel.x) < 1.0, `v_hitter=${hitter.vel.x.toFixed(4)}`);
}

console.log(`\n===== 结果：${pass} 通过 / ${fail} 失败 =====\n`);
process.exit(fail > 0 ? 1 : 0);

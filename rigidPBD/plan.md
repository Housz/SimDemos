XPBD 刚体模拟（论文复现）实施计划
Context（背景与目标）
任务：完整分析 D:\Projects\SimDemos\rigidPBD\ 下的论文 《Detailed Rigid Body Simulation with Extended Position Based Dynamics》（Müller/Macklin/Chentanez/Jeschke/Kim, SCA 2020, CGF 39(8)），用现代 JavaScript 实现其核心算法库，并复现论文中的重要关节与核心演示场景。
用户已确认的决策：
核心场景集（8 个场景；RC 小车、滚珠雕塑为后续扩展，不在本次范围）
单页 index.html + 场景切换器（共享渲染框架）
中文注释（代码注释、README、UI 用中文；标识符保持英文与论文术语对应）
技术约束：现代 JS（ES modules）、不引入前端框架；三维可视化与数学库统一用 three.js，通过 import map + unpkg URL 引入（仓库现代约定：unpkg three@0.156.0 + three/addons/，参照 4.Pendulum3dPBD/index.html 与 script.js）。
说明：PDF 本身有密码保护无法直接读取，规划阶段已用 pypdfium2 提取全文到 rigidPBD/paper_text.txt（UTF-8，含全部公式与 Algorithm 1–3），实施时以此文本为公式依据，保留该文件作为参考。
论文算法要点（实现依据，页码/公式号见 paper_text.txt）
主循环（Algorithm 2）：h = Δt/numSubsteps；每子步：① 显式积分（x += h v；v += h fext/m；q += h/2 [ω,0] q 后归一化；ω += h I⁻¹(τext − ω×(Iω)) 陀螺力矩）；② SolvePositions（numPosIters = 1，非线性投影 Gauss-Seidel）；③ 由位置差分导出速度（v = (x−xprev)/h；Δq = q qprev⁻¹；ω = 2[Δq.xyz]/h，Δq.w < 0 时取反）；④ SolveVelocities（动摩擦、恢复系数、关节阻尼）。
两个基本投影操作（全库基石，Eqs. 2–18）：
位置约束：锚点 r1/r2（体 rest 系），方向 n、量值 C；广义逆质量 w = 1/m + (r×n)ᵀ I⁻¹ (r×n)；XPBD：Δλ = (−C − α̃λ)/(w1+w2+α̃)，α̃ = α/h²，λ 累计；p = Δλ n；x1 += p/m1，x2 −= p/m2；四元数更新 q1 += 1/2 [I1⁻¹(r1×p),0] q1，q2 −= 1/2 [I2⁻¹(r2×p),0] q2。约束力 f = λn/h²（用于可视化与摩擦）。
角度约束：旋转向量 Δq（轴 n、角 θ）；w = nᵀ I⁻¹ n；λ 同式以 θ 代 C；仅更新朝向。力矩 τ = λn/h²。
惯性张量处理：I 随朝向变化 → 把 n、r、p 投影到体 rest 系计算，rest 系下 I0 对角化为 Vector3。
关节：固定（q = q1 q2⁻¹，Δqfixed = 2 q.xyz）；铰链（Δq = a1×a2 + 锚点重合）；目标角/马达（b1 绕 a1 转 α 得 btarget，Δq = btarget×b2，马达每子步 α += h·v）；距离关节（α>0 为弹簧；dmax 为绳约束；目标偏移可做恒力驱动器）；棱柱（逐轴上下限单次投影 + 锁定相对转动）；Algorithm 3 LimitAngle（φ ← arcsin((n1×n2)·n)，n1·n2<0 时 φ ← 2π−φ，wrap 到 (−π,π]，越界则 clamp 后旋转 n1、施加 Δq = n1×n2）；球窝 swing（n = a1×a2）与 twist（n = normalize(a1+a2)，b 投影到 ⊥n 平面）分离限制。
接触：broad phase 每帧一次（AABB 按 k·Δt·|v|（k=2）扩展）；每子步 narrow phase → 法线 n + 局部锚点 r1/r2 + λn/λt（每子步清零）；穿透 d = (p1−p2)·n > 0 时施加 Δx = d n（α=0），每次投影前用当前状态重算法线（论文核心卖点）；静摩擦施加 Δx = Δpt（Δpt = Δp − (Δp·n)n，Δp 为本子步接触点位移），仅当 λt < μs λn。
速度层：相对速度 v = (v1+ω1×r1) − (v2+ω2×r2)；动摩擦 Δv = −(vt/|vt|) min(h μd fn, |vt|)，fn = λn/h²（显式+Gauss-Seidel 逐次钳制 → 无条件稳定）；阻尼 Δv = (v2−v1) min(μlin h,1)、Δω = (ω2−ω1) min(μang h,1)；应用：p = Δv/(w1+w2)，v1 += p/m1，ω1 += I1⁻¹(r1×p)；恢复系数 Δv = n(−vn + max(−e v̄n, 0))，|vn| ≤ 2|g|h 时 e=0 防抖（v̄n 用子步积分前的速度快照计算）。
软硬兼施：compliance α 为柔度（m/N）；α=0 为无限硬约束（PBD 退化）。
参考资料（已调研）
论文全文：rigidPBD/paper_text.txt（已提取）；PDF 原文 rigidPBD/Detailed Rigid Body Simulation with XPBD.pdf。
作者官方配套代码：Ten Minute Physics 教程 22-rigidBodies.html（PBD 刚体） 与 25-joints.html（XPBD 关节）。
本仓库现成参考实现：PBDRigidBody/PBD.js——已完整实现 Body/Joint（SPHERICAL/HINGE/FIXED）+ compliance 求解 + 关节限制 + 阻尼，数学约定（rest 系惯性处理、applyBodyPairCorrection、limitAngle 与 Algorithm 3 等价、速度层钳制、旋转钳制 maxRotationPerSubstep=0.5）经运行验证，新库直接沿用；但无接触/摩擦/恢复系数。它是本计划的数学约定基准。
相关文献：XPBD 原始论文（MMC16）、Small Steps（MSL*19）、Deul 等 PBRBD（DCB14）——论文参考文献列表见 paper_text.txt 末页，README 中会做简要综述。
社区完整实现（GJK/EPA 版，仅作参考，不在本实现采用）：Holger Klein 的 XPBD 刚体贡献。
目录结构（全部新建于 rigidPBD/ 下）
rigidPBD/
├── index.html                 # 单页入口：import map + 侧边栏场景切换器 + 画布
├── README.md                  # 中文说明：算法综述、论文公式↔代码映射、运行方式
├── paper_text.txt             # （已存在）论文全文提取，实现依据
└── js/
    ├── harness.js             # 演示框架：renderer/camera/灯光/网格地面/OrbitControls、
    │                          #   lil-gui 全局参数(重力/子步数/dt/暂停/慢放/重置)、
    │                          #   FPS、能量监视、关节力/力矩可视化开关、鼠标抓取(临时球窝关节)
    ├── core/
    │   ├── math3d.js          # three.js 上的薄封装：四元数轴提取、线性化四元数积分、
    │   │                      #   向量绕四元数旋转、rest 系变换（参照 PBD.js getQuatAxis*）
    │   ├── rigidBody.js       # RigidBody：pose/prevPose、v/ω、invMass、invI0(Vector3 对角)、
    │   │                      #   integrate(h,gravity)(含陀螺力矩开关)、update(h)(速度导出)、
    │   │                      #   getGeneralizedInvMass(n, r)、applyPositionalImpulse(p, r)、
    │   │                      #   applyAngularImpulse(p)、applyVelocityImpulse(p, r)；
    │   │                      #   static 体 invMass=0/invInertia=0；旋转钳制 0.5 rad/子步
    │   ├── shapes.js          # 形状定义与惯性：box/sphere/capsule/cylinder 解析惯性(密度→质量+invI0)、
    │   │                      #   plane/segment/arc 静态碰撞形状；rest 姿态与主轴对齐保证 I0 对角
    │   ├── constraints.js     # XPBD 基类(λ、compliance、solve(dt) 两体投影，Eqs. 2–18)+
    │   │                      #   PositionalConstraint(弹簧/dmax 绳约束/逐轴上下限/目标偏移驱动器)、
    │   │                      #   AngularConstraint(轴对齐/目标角)
    │   ├── joints.js          # DistanceJoint / HingeJoint(限制+目标角马达) / SphericalJoint(swing+twist 限制，
    │   │                      #   含 gimbal-lock maxCorr 处理，参照 PBD.js) / PrismaticJoint(逐轴限制+转动锁定) /
    │   │                      #   FixedJoint；每关节 = 若干原语约束按序投影；关节阻尼(速度层)
    │   ├── contacts.js        # narrow-phase：sphere↔plane、sphere↔sphere、sphere↔segment/arc、
    │   │                      #   box↔plane、box↔box(SAT)、cylinder↔plane、capsule↔plane
    │   │                      #   （均返回 {pos, normal, depth, r1, r2}，单接触点）；
    │   │                      #   ContactSolver：穿透求解(每次投影重算法线)、静摩擦(λn/λt 簿记)、
    │   │                      #   速度层动摩擦+恢复系数(v̄n 用子步前快照，e 阈值 2|g|h)
    │   ├── broadphase.js      # 每帧 AABB 对收集(k·Δt·|v| 扩展, k=2)，对缓存跨子步复用；
    │   │                      #   简单实现(n≤200 体 O(n²) 带早退)；留接口供后续接 Morton BVH
    │   └── world.js           # World：bodies/constraints/contacts/重力/numSubsteps/numPosIters；
    │                          #   step(dt) = Algorithm 2 全流程；关节阻尼；λ→f/τ 查询
    └── scenes/
        ├── registry.js        # 场景注册表 {id, 中文名, create(harness)→Scene}
        ├── sceneSpringBoxes.js  # 图4：天花板挂盒(距离关节 α=0.01 m/N, 质量比 1:1/8) + 力/伸长可视化
        ├── scenePendula.js      # 图8：单/双/三摆 + 闭环四连杆摆(铰链)，子步数滑杆
        ├── sceneJointTypes.js   # 图7：门(铰链+限制)、滑台(棱柱+限制+马达)、球窝(swing/twist 限制)
        ├── sceneRobot.js        # 图17：机械臂(铰链马达+限制) + 目标盒跟随(刚性位置约束做 IK)
        ├── sceneRope.js         # 图18：胶囊链(球窝关节 + swing/twist 限制)，一端可拖拽扭转
        ├── sceneMarbles.js      # 图12/13：弹珠轨道(静态 segment/arc) + 碰撞传递(e=1) + 初始穿透无弹跳
        ├── sceneCoin.js         # 图15：硬币(圆柱)在平面上的高频旋转直至静止
        └── sceneBoxStack.js     # 表1：3/7 盒子堆叠(box-box SAT + 摩擦)
数学约定（严格遵循，实现前先读 PBDRigidBody/PBD.js）
纠正向量约定：corr 为"期望修正量"（方向 n̂，量值 C），body0 沿 +n̂ 移动、body1 沿 −n̂ 移动；λ = −C/(w + α̃)（λ 初值 0，numPosIters=1 时不累计，但接触 λn/λt 需跨子步内簿记）。
rest 系惯性：r_loc = q⁻¹(r_world)，w = 1/m + Σᵢ (r_loc×n_loc)ᵢ² · invI0ᵢ；位置冲量角速度 ω_loc = I0⁻¹ (r_loc×p_loc) → 转回世界系 ω_world = q·ω_loc → 线性化更新 q += 1/2 [ω_world,0] q 后归一化（PBD.js applyCorrection 已验证）。
速度层不更新四元数：v += p·invM；ω += I_world⁻¹(r×p)（即 rest 系 I0⁻¹ 变换回来）。
角度约束：w = Σᵢ n_locᵢ² · invI0ᵢ，冲量 p = Δλ n，仅四元数更新。
积分：ω 世界系 + 陀螺力矩 ω += h·I_world⁻¹(τext − ω×(I_world ω))（开关控制，默认开，比 PBD.js 更贴合论文）；四元数线性化积分 + 归一化。
速度导出：dq = q·qprev⁻¹；ω = 2[dq.x,dq.y,dq.z]/h；dq.w < 0 取反。
LimitAngle：PBD.js 的 π − φ + wrap 与论文 2π − φ + wrap 等价，沿用 PBD.js（含 gimbal-lock 的 maxCorr 钳制）。
静态体：body 传 null 或 invMass=0，w 贡献 0。
关节构成（每关节 = 位置层原语约束序列 + 可选速度层阻尼）
关节	位置层投影序列（每子步）	关键参数
DistanceJoint	1 个 PositionalConstraint（锚点 r̄1/r̄2 在各自 rest 系）	α=0 刚性；α>0 弹簧；dmax>0 绳约束；目标偏移+α 恒力驱动器
FixedJoint	AngularConstraint(Δqfixed) + 位置吸附	α
HingeJoint	AngularConstraint(a1×a2, α=0) + LimitAngle([a1,b1,b2]) + 位置吸附	限制角、目标角/马达(α += h·v)、软限制 α
SphericalJoint	位置吸附 + swing 限制([a1×a2,a1,a2]) + twist 限制(去耦轴)	swing/twist 角度与软硬度
PrismaticJoint	逐轴 PositionalConstraint（滑动轴设 [amin,amax]，其余轴上下限=0）+ 转动锁定（a 轴对齐 + b 轴对齐或 FixedOrientation 可选）	滑程、马达目标偏移
接触管线（每子步）
每帧：broadphase 收集 AABB 对（扩展 k·Δt·|v|）→ 缓存。
每子步：对每对执行 narrow-phase → 生成/更新接触（法线用当前姿态重算、r1/r2 局部锚点、λn/λt 清零）。
位置层（SolvePositions 内，与关节约束交错 GS 求解）：穿透 d=(p1−p2)·n，d>0 → Δx = d n（α=0）；静摩擦：Δpt 投影，λt < μs·λn 时施加。
速度层：动摩擦（min 钳制）→ 恢复系数（v̄n 子步前快照，e 阈值）→ 关节阻尼。
演示框架（harness.js + index.html）
index.html：import map（"three": "https://unpkg.com/three@0.156.0/build/three.module.js"、"three/addons/": ".../examples/jsm/"）+ 左侧场景列表（中文名）+ 右侧渲染区；<script type="module" src="./js/harness.js">。
harness：OrbitControls、lil-gui（全局：重力/子步数/暂停/慢放/重置；每场景自注册参数）、FPS、能量监视曲线（复现论文图 9 的能量守恒对比）、关节力/力矩箭头可视化（复现图 4/5）、鼠标抓取。
场景模块统一生命周期：{ id, name, create(harness) → { update(dt), onGUI(gui), reset() } }；物理体 ↔ three.js mesh 由 createBodyMesh(shape) 辅助绑定，物理更新后同步 transform。
模拟节奏：每动画帧模拟一次 Δt=1/60、N 子步（论文 Table 1 约定，默认 20 子步 1 迭代）。
实施里程碑（每步可独立验证）
M1 骨架：index.html + harness + 场景注册器 + core/math3d.js、core/rigidBody.js、core/world.js + sphere↔plane 接触最小闭环；临时场景：球落地面弹跳验证积分/速度导出/接触正确。
M2 约束与关节：constraints.js + joints.js（全部 5 类关节 + LimitAngle）；场景：弹簧挂盒（验证伸长 = F·α）、双/三/闭环摆（验证能量近似守恒、子步数影响）。
M3 接触完备：box↔box(SAT)、box↔plane、cylinder↔plane、capsule↔plane、sphere↔segment/arc + 静/动摩擦 + 恢复系数；场景：盒子堆叠、硬币、弹珠（碰撞传递 + 初始穿透）。
M4 复杂关节场景 + 可视化：机器人臂 IK、绳索 swing/twist、关节类型展示；力/力矩可视化、能量监视、中文 README、运行说明。
M5（本次范围外，留扩展）：RC 小车、滚珠雕塑（曲线样条）、Morton BVH broad phase（可复用 LearnTenMinutePhysics/bvh.html）、capsule-capsule 自碰撞。
验证方式
运行：rigidPBD/ 下起静态服务（ES module 经 file:// 会被 CORS 拦截）：python -m http.server 8000（或 npx serve），浏览器打开 http://localhost:8000/；逐场景切换验证。
物理数值校验（内置于场景/调试面板）：
弹簧挂盒：α=0.01 m/N、m=1kg、g=10 → 伸长 0.1m、f=10N（与 λ 导出力对比）；
三摆能量监视：40 子步 1 迭代 vs 1 子步 20 迭代的能量衰减对比（复现图 9 结论）；
铰链杆（图 5）：目标角 0、零柔度，挂重物验证力矩正确；
弹珠：e=1 碰撞动量传递（图 12）；初始穿透无大速度跳起（图 13）；
硬币：旋转衰减直至静止、无穿透抖动。
代码级对照：paper_text.txt 中每一条公式（Eqs. 1–18、Alg 1–3）在 js/core/ 中有对应注释标注（如 // 论文 Eq.4: Δλ = (−C − α̃λ)/(w1+w2+α̃)）。
无构建/测试框架（纯浏览器 ES modules）；验证以视觉行为 + 调试面板数值为准。
风险与对策
gimbal-lock（twist 限制 a1+a2≈0）：沿用 PBD.js maxCorr 钳制。
四元数漂移：每次更新后归一化；速度导出用线性化公式。
GS 顺序依赖：约束顺序固定（关节先于接触或按注册序），子步化已大幅降低影响（论文结论）。
接触法线跨子步翻转：每子步重算并缓存于 pair；λ 每子步清零。
性能（100 体 × 20 子步）：JS 下 8 个场景规模（≤60 体）无压力；broadphase O(n²) 早退足够，必要时接 BVH。
three.js r156 API：用 THREE.Vector3/Quaternion/Matrix3、applyQuaternion、setFromAxisAngle 等稳定 API，避开实验性 API。
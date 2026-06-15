import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { GUI } from 'three/addons/libs/lil-gui.module.min.js';

import { robotParser } from './robotParser.js';
import { robotCreator, robotCreatorWithObjText } from './robotCreator.js';
import { robotGUICreator } from './robotGUICreator.js';
import { robotWorkSpaceCreator } from './robotWorkSpaceCreator.js';
import { robotIKHandler } from './robotIKHandler.js';
import { robotRHIKHandler } from './robotRHIKHandler.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { robotConstraintHandler } from './robotConstraintHandler.js';

import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { Line2 } from 'three/addons/lines/Line2.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import { LineGeometry } from 'three/addons/lines/LineGeometry.js';

/* WebXR */
import { VRButton } from 'three/addons/webxr/VRButton.js';
import { XRControllerModelFactory } from 'three/addons/webxr/XRControllerModelFactory.js';
import { OculusHandModel } from 'three/addons/webxr/OculusHandModel.js';

/* ======= 常量（可按需调整） ======= */
const PANEL_DISTANCE   = 0.4;   // 面板到眼睛前向距离（米）
const PANEL_Y_OFFSET   = -0.4; // 面板相对眼睛的垂直偏移

const USER_HEIGHT_OFFS = 1.0;   // 用户初始抬高 1.0m（通过世界整体下移实现）
// 自定义用户想“出生”的世界坐标与朝向
const USER_START = {
  pos: new THREE.Vector3( -3, 0.6, 3 ),        // 想让用户出现在世界中的 (x,y,z)
  yaw: THREE.MathUtils.degToRad(30)                // 初始朝向（绕Y的角度，度数转弧度）
};

const HYD_SPEED        = 0.15;  // 液压缸最大伸缩速度（单位/秒，满杆位时）
const LEVER_DEADZONE   = 0.12;  // 杆位死区（0~1 归一化），小抖动不动作

/* ============ 基础场景 ============ */
const canvas = document.querySelector('#c');
const renderer = new THREE.WebGLRenderer({ antialias: true, canvas });
renderer.shadowMap.enabled = true;
renderer.xr.enabled = true; // WebXR ON

// 非 VR 模式下，把相机初始 y 抬高 0.5
const camera = new THREE.PerspectiveCamera(75, 2, 0.01, 500);
camera.position.set(0, 5.5, 5);

const orbit = new OrbitControls(camera, renderer.domElement);
const transformControl = new TransformControls(camera, renderer.domElement);
transformControl.addEventListener('dragging-changed', (e)=> { orbit.enabled = !e.value; });

const scene = new THREE.Scene();
scene.background = new THREE.Color(0xaaccff);
scene.fog = new THREE.Fog(0xa0a0a0, 10, 200);

// 世界与面板分组：worldGroup 受重力/抬高处理，panelGroup 始终贴脸
const worldGroup = new THREE.Group();
scene.add(worldGroup);
const panelGroup = new THREE.Group();
scene.add(panelGroup);

const pmremGenerator = new THREE.PMREMGenerator(renderer);
pmremGenerator.fromScene(new RoomEnvironment(), 0.04);

worldGroup.add(new THREE.AmbientLight(0xffffff, 1));

const d1 = new THREE.DirectionalLight(0xffffff, 2);
// transformControl.attach(d1); scene.add(transformControl);
d1.position.set(5,5,10); d1.castShadow = true;
d1.shadow.mapSize.set(1024,1024);
d1.shadow.camera.near = 0.5; d1.shadow.camera.far = 50;
worldGroup.add(d1);

const d2 = new THREE.DirectionalLight(0xffffff, 1.5);
d2.position.set(5,10,-10); d2.castShadow = true;
d2.shadow.mapSize.set(1024,1024);
d2.shadow.camera.near = 0.5; d2.shadow.camera.far = 50;
worldGroup.add(d2);

const ground = new THREE.Mesh(
  new THREE.PlaneGeometry(200,200),
  new THREE.MeshStandardMaterial({ color: 0xcccccc, depthWrite: false })
);
ground.rotation.x = -Math.PI/2; ground.receiveShadow = true; worldGroup.add(ground);

const grid = new THREE.GridHelper(200,200);
grid.material.opacity = .8; grid.material.transparent = true; worldGroup.add(grid);

/* ============ 目标与轨迹 ============ */
const targetMat = new THREE.MeshStandardMaterial({ color: 0x03ee00, transparent:true, opacity:.5 });
const targetMesh = new THREE.Mesh(new THREE.SphereGeometry(0.7,16,8), targetMat);
// worldGroup.add(targetMesh);

const toggleBtn = document.createElement('button');
toggleBtn.textContent = '切换目标颜色';
Object.assign(toggleBtn.style, { position:'absolute', left:'10px', top:'10px', zIndex:1000 });
document.body.appendChild(toggleBtn);
let isGreen = true;
toggleBtn.addEventListener('click', ()=> {
  isGreen = !isGreen; targetMesh.material.color.set(isGreen ? 0xffff00 : 0x00ff00);
});

// transformControl.attach(targetMesh);
// scene.add(transformControl);
// transformControl.size = .5;

const keyGroup = new THREE.Group(); worldGroup.add(keyGroup);
let trajectoryCurve;
const ARC_SEGMENTS = 200;

// 加粗折线
const positions = new Float32Array(ARC_SEGMENTS*3);
const lineGeom = new LineGeometry(); lineGeom.setPositions(positions);
const lineMat  = new LineMaterial({ color:0x008811, linewidth:0.7 });
{
  const s = new THREE.Vector2(); renderer.getSize(s);
  const dpr = renderer.getPixelRatio(); lineMat.resolution.set(s.x*dpr, s.y*dpr);
}
const trajectoryLineMesh = new Line2(lineGeom, lineMat);
trajectoryLineMesh.computeLineDistances(); trajectoryLineMesh.scale.set(1,1,1);
worldGroup.add(trajectoryLineMesh);

window.addEventListener('resize', ()=>{
  if (renderer.xr.isPresenting) return; // XR 中不改尺寸
  const s = new THREE.Vector2(); renderer.getSize(s);
  const dpr = renderer.getPixelRatio(); lineMat.resolution.set(s.x*dpr, s.y*dpr);
});

/* ============ 旁置 GLTF ============ */
new GLTFLoader().load('./models/juejinmian.gltf', (gltf)=>{
  const model = gltf.scene;
  model.traverse(c=>{ if(c.isMesh){ c.castShadow = true; c.receiveShadow = true; }});
  model.position.set(-4, model.position.y, 1.6);
  worldGroup.add(model);

  const gui = new GUI();
  gui.domElement.style.position='absolute';
  gui.domElement.style.left='0px';
  gui.domElement.style.opacity='0.8';
  gui.domElement.style.top='650px';
  gui.title('juejinmian Position');
  const f=gui.addFolder('Position');
  const p={x:model.position.x,y:model.position.y,z:model.position.z};
  f.add(p,'x',-10,10).onChange(v=>model.position.x=v);
  f.add(p,'y',-10,10).onChange(v=>model.position.y=v);
  f.add(p,'z',-10,10).onChange(v=>model.position.z=v);
  f.open();
});

/* ============ Robot 载入 + GUI + VR 面板 ============ */
let robot, robotModel;

const constraintGuiMap = new Map(); // name -> { controller, proxy }
const leverItems = [];              // 每根 lever 的状态与映射（不再使用 targetLen）
const activeLeverByPointer = new WeakMap(); // pointer -> leverIndex（独占：最近优先）

async function loadRobotJson(url){
  const r = await fetch(url);
  if(!r.ok) throw new Error('Network error');
  return r.json();
}

loadRobotJson(modelPath).then(data=>{
  robot = robotParser(data);
  robotModel = robotCreator(robot);
  robotModel.traverse(c=>{ if(c.isMesh){ c.castShadow=true; c.receiveShadow=true; }});
  worldGroup.add(robotModel);

  robotGUICreator(robot, robotModel);
  robotWorkSpaceCreator(robot, robotModel);

  // 生成每个液压缸的 GUI 滑块，并缓存 controller
  const guiCtrl = new GUI();
  guiCtrl.domElement.style.position='absolute';
  guiCtrl.domElement.style.left='0px';
  guiCtrl.domElement.style.opacity='0.8';

  robot.constraints.forEach(constraint=>{
    if(constraint.type === 'triangle-prismatic'){
      robotConstraintHandler(robot, robotModel, constraint.name);
      const folder = guiCtrl.addFolder(constraint.name);
      const lower = constraint.limit.lower, upper = constraint.limit.upper;
      const proxy = { length: constraint.length };
      const ctrl  = folder.add(proxy,'length',lower,upper).onChange(v=>{
        // GUI 直接定位：立即生效且限幅
        constraint.length = THREE.MathUtils.clamp(v, lower, upper);
        robotConstraintHandler(robot, robotModel, constraint.name);
      });
      constraintGuiMap.set(constraint.name, { controller: ctrl, proxy });
    }
  });

  const triCons = robot.constraints.filter(c=>c.type==='triangle-prismatic');
  createLeverPanel(triCons); // 面板不在 worldGroup 中
}).catch(console.error);

/* ============ 你的控制 GUI(清/存/播) ============ */
const guiCtl = new GUI();
guiCtl.domElement.style.position='absolute';
guiCtl.domElement.style.left='0px';
guiCtl.domElement.style.opacity='0.8';
guiCtl.domElement.style.top='450px';
guiCtl.title('controller');
const ctlFolder = guiCtl.addFolder('控制');
ctlFolder.add({ clearAllKeys }, 'clearAllKeys');
ctlFolder.add({ saveKeyPosition }, 'saveKeyPosition');
ctlFolder.add({ playTrajectory }, 'playTrajectory');
ctlFolder.add({ toggleVisibility(){ targetMesh.visible=!targetMesh.visible; }}, 'toggleVisibility');
ctlFolder.add({ toggleTransformControl(){ transformControl.visible=!transformControl.visible; }}, 'toggleTransformControl');

/* ============ 轨迹函数 ============ */
function clearAllKeys(){
  keyGroup.clear();
  positions.fill(0);
  lineGeom.setPositions(positions);
  trajectoryLineMesh.computeLineDistances();
}
function saveKeyPosition(){
  const keyMesh = new THREE.Mesh(new THREE.SphereGeometry(0.2,16,8), targetMat);
  keyMesh.position.copy(targetMesh.position);
  keyGroup.add(keyMesh);
  if(keyGroup.children.length>1) updateTrajectoryCurve();
  else { positions.fill(0); lineGeom.setPositions(positions); trajectoryLineMesh.computeLineDistances(); }
}
function updateTrajectoryCurve(){
  trajectoryCurve = new THREE.CatmullRomCurve3(
    keyGroup.children.map(c=>c.position), false, 'catmullrom', 0.05
  );
  const p = new THREE.Vector3();
  for(let i=0;i<ARC_SEGMENTS;i++){
    const t=i/(ARC_SEGMENTS-1);
    trajectoryCurve.getPoint(t,p);
    const b=i*3; positions[b]=p.x; positions[b+1]=p.y; positions[b+2]=p.z;
  }
  lineGeom.setPositions(positions);
  trajectoryLineMesh.computeLineDistances();
}
function playTrajectory(){
  if(!trajectoryCurve) return;
  const speed=0.02; let t=0;
  (function step(){
    t += speed/trajectoryCurve.getLength(); if(t>=1) t=1;
    const p = trajectoryCurve.getPointAt(t);
    targetMesh.position.copy(p);
    robotRHIKHandler(targetMesh, robot, robotModel);
    if(t<1) requestAnimationFrame(step);
  })();
}

/* ============ WebXR：手/手柄/指尖包装 ============ */
const controllerModelFactory = new XRControllerModelFactory();

const controller1 = renderer.xr.getController(0); scene.add(controller1);
const controller2 = renderer.xr.getController(1); scene.add(controller2);

const controllerGrip1 = renderer.xr.getControllerGrip(0);
controllerGrip1.add(controllerModelFactory.createControllerModel(controllerGrip1));
scene.add(controllerGrip1);

const controllerGrip2 = renderer.xr.getControllerGrip(1);
controllerGrip2.add(controllerModelFactory.createControllerModel(controllerGrip2));
scene.add(controllerGrip2);

const hand1 = renderer.xr.getHand(0); const handModel1 = new OculusHandModel(hand1); hand1.add(handModel1); scene.add(hand1);
const hand2 = renderer.xr.getHand(1); const handModel2 = new OculusHandModel(hand2); hand2.add(handModel2); scene.add(hand2);

const _bb = new THREE.Box3(), _v = new THREE.Vector3(), _c = new THREE.Vector3();
class ControllerPointer {
  constructor(controller){
    const tip = new THREE.Mesh(
      new THREE.SphereGeometry(0.008,16,16), // 指尖更小，避免误触
      new THREE.MeshBasicMaterial({ color:0x00ffff, transparent:true, opacity:.12 })
    );
    tip.visible = false; tip.position.set(0,0,-0.025); // 稍短的前伸
    controller.add(tip); this.tip = tip;
  }
  getPointerPosition(){ return this.tip.getWorldPosition(_v); }
  intersectBoxObject(obj){ _bb.setFromObject(obj); return _bb.containsPoint(this.getPointerPosition()); }
}
const ctrlPtr1 = new ControllerPointer(controller1);
const ctrlPtr2 = new ControllerPointer(controller2);
const pointers = [handModel1, handModel2, ctrlPtr1, ctrlPtr2];

document.body.appendChild(VRButton.createButton(renderer, { optionalFeatures: ['hand-tracking'] }));

// 进入/退出 VR：世界整体下移/复位，实现“人抬高 0.5”
// renderer.xr.addEventListener('sessionstart', ()=>{ worldGroup.position.y = -USER_HEIGHT_OFFS; });
renderer.xr.addEventListener('sessionstart', ()=>{
  // 抬高 0.5m 的逻辑可以并入 USER_START.pos.y（或保留你的 USER_HEIGHT_OFFS）
  worldGroup.position.copy(USER_START.pos).multiplyScalar(-1); // 反向平移整个世界
  worldGroup.rotation.set(0, -USER_START.yaw, 0);              // 反向旋转世界，实现用户初始朝向
});
// renderer.xr.addEventListener('sessionend',   ()=>{ worldGroup.position.y = 0; });
renderer.xr.addEventListener('sessionend', ()=>{
  worldGroup.position.set(0,0,0);
  worldGroup.rotation.set(0,0,0);
});

/* ============ VR 面板与操作杆（采用你的 buildLever 参数） ============ */
function createLeverPanel(constraints){
  const PANEL_COLOR = 0xededed;
  const base = new THREE.Mesh(new THREE.BoxGeometry(0.7,0.12,0.2), new THREE.MeshStandardMaterial({ color:PANEL_COLOR }));
  base.castShadow = true; base.receiveShadow = true; panelGroup.add(base);

  const count = constraints.length;
  // const spacing = 0.1;          // 如需更密集，调小这里（如 0.14）
  const spacing = 0.07;          // 如需更密集，调小这里（如 0.14）
  const discLen = 0.1;
  // const startX = -((count-1)*spacing)/2;
  const startX = -((7-1)*spacing)/2;

  for(let i=0;i<count;i++){
    // 奇数杆 
    if (i%2 == 1) {
      continue;
    }
    const cons = constraints[i];
    const lever = buildLever({ x:startX+i*spacing, z:0, parent:base, discLen, panelColor:PANEL_COLOR });
    leverItems.push({
      pivot: lever.pivot,
      grip:  lever.grip,
      minAngle:-Math.PI/3,
      maxAngle: Math.PI/3,
      recovery:3.5,
      constraint: cons,
      lower: cons.limit.lower,
      upper: cons.limit.upper
    });
  }
}

function buildLever({ x=0, z=0, parent, discLen=0.12, panelColor=0xededed }){
  const root = new THREE.Group(); parent.add(root); root.position.set(x, .06, z);

  const LIGHT=0xb5b5b5, DARK=0x1f1f1f, ROD=0xbdbdbd, ORANGE=0xf26722;
  const rOuter=.036, rMid=.036, rInner=.036;

  const makeDisc=(r,len,color,x=0)=>{
    const m=new THREE.Mesh(new THREE.CylinderGeometry(r,r,len,48), new THREE.MeshStandardMaterial({color}));
    m.rotation.z=Math.PI/2; m.position.set(x,0,0); m.castShadow=m.receiveShadow=true; return m;
  };

  root.add(
    makeDisc(rOuter,0.03,LIGHT,-0.03),
    makeDisc(rMid  ,0.03,DARK , 0.00),
    makeDisc(rInner,0.03,LIGHT, 0.03)
  );

  const cover=new THREE.Mesh(
    new THREE.BoxGeometry(discLen+0.004, rOuter+0.002, rOuter*2+0.006),
    new THREE.MeshStandardMaterial({ color: panelColor })
  );
  cover.position.set(0, -(rOuter/2)+0.0006, 0);
  cover.castShadow=false; cover.receiveShadow=false; root.add(cover);

  const pivot=new THREE.Group(); root.add(pivot); pivot.position.set(0,0.02,0);

  const rod=new THREE.Mesh(new THREE.CylinderGeometry(0.008,0.008,0.28,24), new THREE.MeshStandardMaterial({ color: ROD }));
  rod.castShadow=true; rod.position.y=0.14; pivot.add(rod);

  const handle=new THREE.Mesh(new THREE.CylinderGeometry(0.020,0.012,0.10,24), new THREE.MeshStandardMaterial({ color: ORANGE }));
  handle.castShadow=true; handle.position.y=0.30; pivot.add(handle);

  // === 抓握盒（可调） ===
  // const GRIP = { x: 0.10, y: 0.12, z: 0.10 };
  const GRIP = { x: 0.20, y: 0.22, z: 0.20 };
  const grip=new THREE.Mesh(new THREE.BoxGeometry(GRIP.x, GRIP.y, GRIP.z), new THREE.MeshBasicMaterial({ transparent:true, opacity:.04 }));
  grip.visible=false;  // 调参时可设 true
  grip.position.y=0.30;
  pivot.add(grip);

  return { root, pivot, grip };
}

/* ============ lever & 液压缸更新（独占 + 速度控制） ============ */
const globalClock = new THREE.Clock();
const _forward = new THREE.Vector3();

// 每帧把面板带在眼前（只取 yaw）
function updatePanelToUser(){
  if (!renderer.xr.isPresenting) return;
  const xrCam = renderer.xr.getCamera();
  _forward.set(0,0,-1).applyQuaternion(xrCam.quaternion);
  _forward.y = 0; if (_forward.lengthSq()<1e-6) _forward.set(0,0,-1);
  _forward.normalize();
  panelGroup.position.copy(xrCam.position).addScaledVector(_forward, PANEL_DISTANCE);
  panelGroup.position.y = xrCam.position.y + PANEL_Y_OFFSET;
  panelGroup.quaternion.setFromUnitVectors(new THREE.Vector3(0,0,-1), _forward);
}

// 指针 -> 最近 lever 的独占选择，避免“一碰多杆”
function updateActiveLeversForPointers(){
  pointers.forEach(ptr=>{
    let bestIdx = null, bestDist = Infinity;
    leverItems.forEach((it, idx)=>{
      if (!ptr.intersectBoxObject || !ptr.getPointerPosition) return;
      if (!ptr.intersectBoxObject(it.grip)) return;
      const center = it.grip.getWorldPosition(_c);
      const ppos   = ptr.getPointerPosition();
      const d = center.distanceTo(ppos);
      if (d < bestDist){ bestDist = d; bestIdx = idx; }
    });
    if (bestIdx !== null) activeLeverByPointer.set(ptr, bestIdx);
    else activeLeverByPointer.delete(ptr);
  });
}

function applyLeverPhysicsAndHydraulics(delta){
  // 1) 为每个指针选最近 lever（独占）
  updateActiveLeversForPointers();

  // 2) 杆位更新（被选中的指针才能驱动该 lever）
  leverItems.forEach((it, leverIndex)=>{
    let interacting = false;
    let target = it.pivot.rotation.x;

    pointers.forEach(ptr=>{
      if (!ptr || !ptr.intersectBoxObject || !ptr.getPointerPosition) return;
      if (activeLeverByPointer.get(ptr) !== leverIndex) return; // 非“最近”忽略
      if (!ptr.intersectBoxObject(it.grip)) return; // 保险：指针已离开
      interacting = true;
      const local = it.pivot.worldToLocal(ptr.getPointerPosition(_v).clone());
      let theta = Math.atan2(local.z, local.y);
      theta = THREE.MathUtils.clamp(theta, it.minAngle, it.maxAngle);
      target = theta;
    });

    if (interacting){
      it.pivot.rotation.x = THREE.MathUtils.lerp(it.pivot.rotation.x, target, 0.5);
    }else{
      // 未交互：杆位弹回中心，但油缸不回位（见下一个步骤）
      it.pivot.rotation.x += (-it.pivot.rotation.x) * Math.min(1, it.recovery*delta);
    }

    // 3) 杆位 → 速度命令（而非目标长度）
    const maxAbs = Math.max(Math.abs(it.minAngle), Math.abs(it.maxAngle));
    let cmd = THREE.MathUtils.clamp(it.pivot.rotation.x / maxAbs, -1, 1); // [-1,1]
    // 死区：避免轻微抖动
    if (Math.abs(cmd) < LEVER_DEADZONE) cmd = 0;
    else {
      const s = Math.sign(cmd);
      cmd = s * ( (Math.abs(cmd) - LEVER_DEADZONE) / (1 - LEVER_DEADZONE) );
    }

    // 4) 油缸按速度积分（并限幅），松手后保持当前位置
    const cons  = it.constraint;
    const lower = it.lower, upper = it.upper;
    const before = cons.length;
    if (cmd !== 0){
      cons.length += cmd * HYD_SPEED * delta;
      cons.length = THREE.MathUtils.clamp(cons.length, lower, upper);
    }
    // 若长度有变化才驱动与更新 GUI
    if (Math.abs(cons.length - before) > 1e-6){
      robotConstraintHandler(robot, robotModel, cons.name);
      const g = constraintGuiMap.get(cons.name);
      if (g){ g.proxy.length = cons.length; g.controller.updateDisplay(); }
    }
  });
}

function resizeRendererToDisplaySize(renderer){
  if (renderer.xr && renderer.xr.isPresenting) return false; // XR 中不改尺寸
  const c = renderer.domElement;
  const pr = window.devicePixelRatio;
  const w = c.clientWidth * pr | 0;
  const h = c.clientHeight * pr | 0;
  const need = (c.width !== w || c.height !== h);
  if (need) renderer.setSize(w, h, false);
  return need;
}

/* ============ 渲染循环（setAnimationLoop） ============ */
renderer.setAnimationLoop(()=>{
  if (resizeRendererToDisplaySize(renderer)) {
    const c = renderer.domElement;
    camera.aspect = c.clientWidth / c.clientHeight;
    camera.updateProjectionMatrix();
  }

  const delta = globalClock.getDelta();
  applyLeverPhysicsAndHydraulics(delta); // <- 独占 + 速度控制 + 限幅保持
  updatePanelToUser();                   // <- 面板贴脸

  renderer.render(scene, camera);
});

/* ============（可选）暴露接口 ============ */
function readFromText(robotJson){
  console.log(robotJson);
  worldGroup.remove(robotModel);
  let r = robotParser(robotJson);
  robotModel = robotCreator(r);
  worldGroup.add(robotModel);
  robotGUICreator(r, robotModel);
}
function readFromJSONandOBJ(robotJson, robotObjs){
  console.log(robotJson);
  worldGroup.remove(robotModel);
  let r = robotParser(robotJson);
  robotModel = robotCreatorWithObjText(r, robotObjs);
  worldGroup.add(robotModel);
  robotGUICreator(r, robotModel);
}
export { readFromText, readFromJSONandOBJ };

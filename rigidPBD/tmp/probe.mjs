import * as THREE from 'three';
import { World } from 'file:///D:/Projects/SimDemos/rigidPBD/js/core/world.js';
import { RigidBody } from 'file:///D:/Projects/SimDemos/rigidPBD/js/core/rigidBody.js';
import { ShapeType, plane as planeShape } from 'file:///D:/Projects/SimDemos/rigidPBD/js/core/shapes.js';
import { scenes } from 'file:///D:/Projects/SimDemos/rigidPBD/js/scenes/registry.js';
const byId = Object.fromEntries(scenes.map(s=>[s.id,s]));
class FH {
  constructor(){ this.params={gravity:9.81,numSubsteps:20,numPosIters:1,dt:1/60,timeScale:1,paused:false,useGyroscopic:true,showGrid:true,showContacts:false,showForces:false,forceScale:0.02,showEnergy:false};
    this.world=this._w(); this.gui={add:()=>new Proxy({},{get:()=>()=>{}}),addFolder(){return this.gui},controllersRecursive:()=>[],destroy(){}};
    this.currentScene=null;this._h=null;this._root=new THREE.Group();}
  _w(){return new World({gravity:new THREE.Vector3(0,-this.params.gravity,0),numSubsteps:this.params.numSubsteps,numPosIters:this.params.numPosIters,useGyroscopic:this.params.useGyroscopic});}
  meshFor(s){ if(s.type===ShapeType.PLANE) return null; return new THREE.Mesh(new THREE.BoxGeometry(1,1,1), new THREE.MeshBasicMaterial()); }
  addBody({shape,color,...r}){const m=this.meshFor(shape);const b=new RigidBody({shape,mesh:m,...r});this.world.addBody(b);if(m)this._root.add(m);return b;}
  addObject(o){this._root.add(o);return o;} addGroundPlane(){return this.world.createBody({shape:planeShape(new THREE.Vector3(0,1,0),0),isStatic:true,name:'ground'});}
  watchJoint(j){return j;} clearWatchedJoints(){} removeObjects(){} refreshGUI(){} resetScene(){}
  setScene(d){this.currentScene=d;this.world=this._w();this._root=new THREE.Group();this._h=d.create(this)||{};}
  step(dt=1/60){this._h.preStep?.(dt);this.world.step(dt);this._h.postStep?.(dt);}
  computeEnergy(){let k=0,p=0;for(const b of this.world.bodies){if(!b.isDynamic)continue;k+=0.5*b.mass*b.vel.lengthSq();p+=-b.mass*this.world.gravity.dot(b.pose.p);}return{kinetic:k,potential:p};}
}
const h=new FH();

console.log('===== ROBOT =====');
h.setScene(byId.robot);
const joints=h.world.joints;
for(const j of joints) console.log('  joint', j.constructor.name, 'meta=',JSON.stringify(j.meta||null),
  'A=',j.bodyA?j.bodyA.name||j.bodyA.shape.type:'null','B=',j.bodyB?j.bodyB.name||j.bodyB.shape.type:'null',
  'target=',j.targetAngle!==undefined?(j.targetAngle*180/Math.PI).toFixed(1)+'°':'-', 'rest=',j.restLength);
console.log('  bodies:', h.world.bodies.map(b=>(b.name||b.shape.type)+(b.isStatic?'(S)':'')).join(' '));
const ik=joints.find(j=>j.meta?.role==='ik');
console.log('  ik compliance',ik.compliance,'restLength',ik.restLength, 'damping', ik.posDamping);
for(let i=0;i<180;i++){ h.step(); if(i%60===59){ ik.updateGlobalPoses();
  const tip=ik.bodyB.toWorld(ik.meta.tipLocal.clone());
  console.log(`  t=${((i+1)/60).toFixed(1)}s tgt=(${ik.globalPoseA.p.x.toFixed(2)},${ik.globalPoseA.p.y.toFixed(2)},${ik.globalPoseA.p.z.toFixed(2)}) tip=(${tip.x.toFixed(2)},${tip.y.toFixed(2)},${tip.z.toFixed(2)}) err=${ik.getDistance().toFixed(4)} fs=${ik.lastForce.toFixed(2)}N`);}}

console.log('\n===== ROPE =====');
h.setScene(byId.rope);
for(let i=0;i<180;i++) h.step();
const J=h.world.joints.filter(j=>j.twistLimit);
let maxSwing=0, sumPaper=0, sumTrue=0;
for(const j of J){ j.updateGlobalPoses();
  const a1=new THREE.Vector3(1,0,0).applyQuaternion(j.globalPoseA.q);
  const a2=new THREE.Vector3(1,0,0).applyQuaternion(j.globalPoseB.q);
  maxSwing=Math.max(maxSwing,Math.acos(THREE.MathUtils.clamp(a1.dot(a2),-1,1)));
  const n=new THREE.Vector3().addVectors(a1,a2); if(n.lengthSq()<1e-12) continue; n.normalize();
  const b1=new THREE.Vector3(0,1,0).applyQuaternion(j.globalPoseA.q); b1.addScaledVector(n,-n.dot(b1));
  const b2=new THREE.Vector3(0,1,0).applyQuaternion(j.globalPoseB.q); b2.addScaledVector(n,-n.dot(b2));
  if(b1.lengthSq()>1e-12&&b2.lengthSq()>1e-12) sumPaper+=Math.acos(THREE.MathUtils.clamp(b1.normalize().dot(b2.normalize()),-1,1));
  // 真实扭转：相对旋转在绳轴上的分量
  const qa=j.bodyA.pose.q, qb=j.bodyB.pose.q;
  const dq=new THREE.Quaternion(qa.x,qa.y,qa.z,qa.w).multiply(new THREE.Quaternion(-qb.x,-qb.y,-qb.z,qb.w));
  if(dq.w<0) dq.set(-dq.x,-dq.y,-dq.z,-dq.w);
  const ang=2*Math.acos(THREE.MathUtils.clamp(dq.w,-1,1));
  const s=Math.sqrt(Math.max(0,1-dq.w*dq.w));
  const rv=s<1e-9?new THREE.Vector3():new THREE.Vector3(dq.x/s,dq.y/s,dq.z/s).multiplyScalar(ang);
  sumTrue+=Math.abs(rv.dot(a1));
}
console.log('  每关节最大 swing:',(maxSwing*180/Math.PI).toFixed(1),'°  (限制 22°)');
console.log('  论文 Eq.25 投影法总扭转:',(sumPaper*180/Math.PI).toFixed(1),'°');
console.log('  相对旋转投影到绳轴的“真扭转”:',(sumTrue*180/Math.PI).toFixed(1),'°');

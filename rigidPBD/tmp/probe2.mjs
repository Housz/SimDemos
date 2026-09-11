import * as THREE from 'three';
const B='file:///D:/Projects/SimDemos/rigidPBD/js/';
const {World}=await import(B+'core/world.js');
const {RigidBody}=await import(B+'core/rigidBody.js');
const {ShapeType,plane:planeShape}=await import(B+'core/shapes.js');
const {scenes}=await import(B+'scenes/registry.js');
const byId=Object.fromEntries(scenes.map(s=>[s.id,s]));
class FH{constructor(){this.params={gravity:9.81,numSubsteps:20,numPosIters:1,dt:1/60,useGyroscopic:true,showForces:true};
 this.world=this._w();this.gui={add:()=>new Proxy({},{get:()=>()=>{}}),addFolder(){return this.gui},controllersRecursive:()=>[],destroy(){}};this._root=new THREE.Group();}
 _w(){return new World({gravity:new THREE.Vector3(0,-this.params.gravity,0),numSubsteps:20,numPosIters:1,useGyroscopic:true});}
 meshFor(s){return s.type===ShapeType.PLANE?null:new THREE.Mesh(new THREE.BoxGeometry(1,1,1),new THREE.MeshBasicMaterial());}
 addBody({shape,color,...r}){const m=this.meshFor(shape);const b=new RigidBody({shape,mesh:m,...r});this.world.addBody(b);if(m)this._root.add(m);return b;}
 addObject(o){this._root.add(o);return o;}addGroundPlane(){return this.world.createBody({shape:planeShape(new THREE.Vector3(0,1,0),0),isStatic:true});}
 watchJoint(j){return j}clearWatchedJoints(){}removeObjects(){}refreshGUI(){}resetScene(){}
 setScene(d){this.currentScene=d;this.world=this._w();this._root=new THREE.Group();this._h=d.create(this)||{};}
 step(dt=1/60){this._h.preStep?.(dt);this.world.step(dt);this._h.postStep?.(dt);}}

const S=(await import(B+'scenes/sceneRobot.js')).settings;
S.auto=false; S.tx=1.5; S.ty=1.6; S.tz=0.0;
const h=new FH(); h.setScene(byId.robot);
const ik=h.world.joints.find(j=>j.meta?.role==='ik');
const hinges=h.world.joints.filter(j=>j.targetAngle!==undefined);
const fore=ik.bodyB;
for(let i=0;i<600;i++){h.step();
 if(i%60===59){ik.updateGlobalPoses();
  const tip=fore.toWorld(ik.meta.tipLocal.clone());
  console.log(`t=${((i+1)/60).toFixed(1)}s tip=(${tip.x.toFixed(3)},${tip.y.toFixed(3)},${tip.z.toFixed(3)}) err=${ik.getDistance().toFixed(4)} F=${ik.lastForce.toFixed(1)}N`,
   ' hingeAngles=',hinges.map(j=>(j.getCurrentAngle()*180/Math.PI).toFixed(1)+'°['+(j.minAngle*180/Math.PI).toFixed(0)+','+(j.maxAngle*180/Math.PI).toFixed(0)+']').join(' '));}}
console.log('\n--- 可达性分析 ---');
const sh=new THREE.Vector3(0,1.52,0), tgt=new THREE.Vector3(1.5,1.6,0);
console.log('肩到目标距离',tgt.distanceTo(sh).toFixed(3),'m   臂长 1.4 + 1.1 = 2.5 m  最小 0.3 m');

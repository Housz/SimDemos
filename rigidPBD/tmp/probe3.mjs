import * as THREE from 'three';
const B='file:///D:/Projects/SimDemos/rigidPBD/js/';
const {World}=await import(B+'core/world.js');
const {RigidBody}=await import(B+'core/rigidBody.js');
const {ShapeType,plane:planeShape}=await import(B+'core/shapes.js');
const {scenes}=await import(B+'scenes/registry.js');
const byId=Object.fromEntries(scenes.map(s=>[s.id,s]));
class FH{constructor(){this.params={gravity:9.81,numSubsteps:20,numPosIters:1,dt:1/60,useGyroscopic:true};
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
h.step();
const show=(tag)=>{
  const ps=h.world.bodies.filter(b=>b.isDynamic).map(b=>`${b.name||b.shape.type}@(${b.pose.p.x.toFixed(4)},${b.pose.p.y.toFixed(4)},${b.pose.p.z.toFixed(4)}) v=${b.vel.length().toFixed(5)} w=${b.omega.length().toFixed(5)}`);
  console.log(tag, ps.join('\n        '));
  console.log('   接触数', h.world.contacts.length, ' 候选对', h.world.pairsCount, ' maxPen', h.world.maxPenetration.toFixed(5));
};
show('t=1帧:');
for(let i=0;i<120;i++) h.step();
show('t=2s :');
const ik=h.world.joints.find(j=>j.meta?.role==='ik');
console.log('lambda keys', JSON.stringify(ik.lambda), ' lastForce', ik.lastForce.toFixed(2));
const hinges=h.world.joints.filter(j=>j.targetAngle!==undefined);
hinges.forEach((j,i)=>{ j.updateGlobalPoses();
  const a1=new THREE.Vector3(1,0,0).applyQuaternion(j.globalPoseA.q);
  const a2=new THREE.Vector3(1,0,0).applyQuaternion(j.globalPoseB.q);
  console.log(`  铰链${i}: getCurrentAngle=${(j.getCurrentAngle()*180/Math.PI).toFixed(2)}° 轴夹角=${(Math.acos(THREE.MathUtils.clamp(a1.dot(a2),-1,1))*180/Math.PI).toFixed(2)}° target=${(j.targetAngle*180/Math.PI).toFixed(1)}° 锚点距=${j.globalPoseA.p.distanceTo(j.globalPoseB.p).toFixed(6)}`);
});

import * as THREE from 'three';
import { RigidBody } from '../js/core/rigidBody.js';
import { cylinder, plane as planeShape } from '../js/core/shapes.js';
import { collide } from '../js/core/contacts.js';

const R = 0.13, H = 0.022;
const plane = new RigidBody({ shape: planeShape(new THREE.Vector3(0,1,0), 0), isStatic: true });
for (const [tiltDeg, y] of [[8, 0.13028], [8, 0.1290], [8, 0.1288], [0, 0.011], [0, 0.0105]]) {
    const tilt = THREE.MathUtils.degToRad(tiltDeg);
    const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0), tilt);
    const coin = new RigidBody({ shape: cylinder(R,H), mass: 0.02, position: new THREE.Vector3(0,y,0), quaternion: q });
    const c = collide(plane, coin);
    console.log(`tilt=${tiltDeg} y=${y.toFixed(5)} → ${c.length} 个接触`,
        c.map(d => d.normalData.toArray().map(v=>v.toFixed(2)).join(',')).join(' | '));
    const c2 = collide(coin, plane);
    console.log(`    反向顺序 → ${c2.length}`);
}

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { GUI } from 'three/addons/libs/lil-gui.module.min.js';

import { robotParser } from './robotParser.js';
import { robotCreator, robotCreatorWithObjText } from './robotCreator.js';
import { robotGUICreator } from './robotGUICreator.js';
import { robotIKHandler } from './robotIKHandler.js'

/////////////////////////////////// scene{ ///////////////////////////////////
const canvas = document.querySelector('#c');
const renderer = new THREE.WebGLRenderer({ antialias: true, canvas });
renderer.shadowMap.enabled = true;

const fov = 75;
const aspect = 2;
const near = 0.01;
const far = 500;
const camera = new THREE.PerspectiveCamera(fov, aspect, near, far);
camera.position.set(0, 5, 5);

const orbit = new OrbitControls(camera, renderer.domElement);
const transformControl = new TransformControls(camera, renderer.domElement);

transformControl.addEventListener('dragging-changed', function (event) {
	orbit.enabled = !event.value;
});


const scene = new THREE.Scene();

scene.background = new THREE.Color(0xaaccff);
scene.fog = new THREE.Fog(0xa0a0a0, 10, 200);

// camera = new THREE.PerspectiveCamera( 70, window.innerWidth / window.innerHeight, 1, 10000 );
// camera.position.set( 0, 250, 1000 );
// scene.add( camera );

scene.add(new THREE.AmbientLight(0xdddddd, 3));
const light = new THREE.SpotLight(0xffffff, 2);
light.position.set(200, 200, 50);
light.angle = Math.PI * 0.2;
light.decay = 0;
light.castShadow = true;
light.shadow.camera.near = 200;
light.shadow.camera.far = 2000;
light.shadow.bias = - 0.000222;
light.shadow.mapSize.width = 1024;
light.shadow.mapSize.height = 1024;
// scene.add(light);

const dlight = new THREE.DirectionalLight(0xeeeeee, 3);
dlight.position.set(5, 2, 4);
scene.add(dlight);

// const planeGeometry = new THREE.PlaneGeometry(2000, 2000);
// planeGeometry.rotateX(- Math.PI / 2);
// const planeMaterial = new THREE.ShadowMaterial({ color: 0x000000, opacity: 0.2 });
// const plane = new THREE.Mesh(planeGeometry, planeMaterial);
// plane.position.y = - 0.2;
// plane.receiveShadow = true;
// scene.add(plane);

const mesh = new THREE.Mesh(new THREE.PlaneGeometry(200, 200), new THREE.MeshPhongMaterial({ color: 0xcccccc, depthWrite: false }));
mesh.rotation.x = - Math.PI / 2;
mesh.receiveShadow = true;
scene.add(mesh);

const helper = new THREE.GridHelper(200, 200);
helper.material.opacity = 0.8;
helper.material.transparent = true;
scene.add(helper);

// const geometry = new THREE.BoxGeometry(1, 1, 1);
// const material = new THREE.MeshPhongMaterial({ color: 0xeeeeee });
// const cube = new THREE.Mesh(geometry, material);
// cube.position.y = .5;
// cube.castShadow = true;
// scene.add(cube);

// const geometry = new THREE.CylinderGeometry( .2, .2, .5, 16 ); 
// geometry.rotateZ(Math.PI / 2); // axis to [1, 0, 0]
// geometry.applyMatrix4(new THREE.Matrix4().makeTranslation(.5/2, 0, 0));
// const material = new THREE.MeshPhongMaterial( {color: 0xeeeeee} ); 
// const cylinder = new THREE.Mesh( geometry, material ); 
// cylinder.castShadow = true;
// scene.add( cylinder );


/////////////////////////////////// scene} ///////////////////////////////////


/////////////////////////////////// mesh{ ///////////////////////////////////

// target of end effector
let targetGeo = new THREE.SphereGeometry(0.3, 16, 8);
let targetMat = new THREE.MeshPhongMaterial({ color: 0xffff00, transparent: true, opacity: 0.4 });
let targetMesh = new THREE.Mesh(targetGeo, targetMat);
// scene.add(targetMesh);

transformControl.attach(targetMesh);
// scene.add(transformControl);



let robotModel;

let gui = new GUI();

// for trajectory
let keyGroup = new THREE.Group();
scene.add(keyGroup);
let trajectoryCurve; // THREE.CatmullRomCurve3


const ARC_SEGMENTS = 200;
let lineMat = new THREE.LineBasicMaterial({ color: 0x008811 })
const geometry = new THREE.BufferGeometry();
geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(ARC_SEGMENTS * 3), 3));

let trajectoryLineMesh = new THREE.Line(
	geometry,
	lineMat
); // THREE.Line
scene.add(trajectoryLineMesh);


// load robot json
function loadRobotJson(url) {
	return new Promise(function (resolve, reject) {
		fetch(url)
			.then(response => {
				if (!response.ok) {
					throw new Error('Network response was not ok');
				}
				return response.json();
			})
			.then(jsonData => {
				resolve(jsonData);
			})
			.catch(error => {
				reject(error);
			});
	});
}

loadRobotJson(modelPath)
// loadRobotJson("./robot.json")
// loadRobotJson("./models/UR5/UR5.json")
	// loadRobotJson("./models/stanford/StanfordRRP.json")
	.then(data => {

		let robot = robotParser(data);

		// console.log("robot");
		// console.log(robot);

		robotModel = robotCreator(robot);

		// transformControl.setMode( 'rotate' );
		// transformControl.attach(robotModel.children[0].children[0].children[0].children[0].children[0].children[0].children[0]);
		// scene.add(transformControl);

		console.log("robotModel");
		console.log(robotModel);

		// robotModel.scale.set(10, 10, 10);

		scene.add(robotModel);


		robotGUICreator(robot, robotModel);

		// controller
		// const gui = new GUI();

		// gui.title("controller");
		// let controllerFolder = gui.addFolder("控制");
		// let controllerObject = {
		// 	clearAllKeys: () => { clearAllKeys(); },
		// 	saveKeyPosition: () => { saveKeyPosition(); },
		// 	playTrajectory: () => { playTrajectory(); }
		// };

		// controllerFolder.add(controllerObject, 'clearAllKeys');
		// controllerFolder.add(controllerObject, 'saveKeyPosition');
		// controllerFolder.add(controllerObject, 'playTrajectory');

		// transformControl.addEventListener('change', function (event) {

		// 	robotIKHandler(targetMesh, robotModel);

		// });





	})
	.catch(error => {
		console.error('Error loading JSON:', error);
	});


/**
 * online upload customise json
 */
function readFromText(robotJson) {
	console.log(robotJson);

	scene.remove(robotModel);

	let robot = robotParser(robotJson);

	robotModel = robotCreator(robot);
	// robotModel = robotCreatorWithObjText(robot, objTexts)

	console.log("robotModel");
	console.log(robotModel);


	scene.add(robotModel);

	robotGUICreator(robot, robotModel);
}


function readFromJSONandOBJ(robotJson, robotObjs) {
	console.log(robotJson);

	scene.remove(robotModel);

	let robot = robotParser(robotJson);

	// robotModel = robotCreator(robot);
	robotModel = robotCreatorWithObjText(robot, robotObjs)

	console.log("robotModel");
	console.log(robotModel);


	scene.add(robotModel);

	robotGUICreator(robot, robotModel);
}





function clearAllKeys() {
	keyGroup.clear();
	// console.log(trajectoryLineMesh.geometry.attributes.position);

	trajectoryLineMesh.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(ARC_SEGMENTS * 3), 3));

}

function saveKeyPosition() {
	const geometry = new THREE.SphereGeometry(0.3, 16, 8);
	const keyMesh = new THREE.Mesh(geometry, targetMat);


	console.log(targetMesh.position);

	keyMesh.position.set(targetMesh.position.x, targetMesh.position.y, targetMesh.position.z);

	keyGroup.add(keyMesh);

	if (keyGroup.children.length > 1) {
		updateTrajectoryCurve();
	}

}




function updateTrajectoryCurve() {
	trajectoryCurve = new THREE.CatmullRomCurve3(
		keyGroup.children.map((child) => child.position)
	);
	console.log(trajectoryCurve);

	// const points = trajectoryCurve.getPoints(ARC_SEGMENTS);
	// trajectoryLineMesh = new THREE.Line(
	// 	new THREE.BufferGeometry().setFromPoints(points),
	// 	new THREE.LineBasicMaterial({ color: 0x008811 })
	// );

	const position = trajectoryLineMesh.geometry.attributes.position;
	let p = new THREE.Vector3();
	for (let i = 0; i < ARC_SEGMENTS; i++) {

		const t = i / (ARC_SEGMENTS - 1);
		trajectoryCurve.getPoint(t, p);
		position.setXYZ(i, p.x, p.y, p.z);

	}

	position.needsUpdate = true;

}

function playTrajectory() {

	let step = 0.001;
	let currT = 0;

	let intervalId = setInterval(() => {

		if (currT + step > 1) {
			clearInterval(intervalId);
			return;
		}

		currT += step;

		let currPosition = trajectoryCurve.getPointAt(currT);

		targetMesh.position.set(currPosition.x, currPosition.y, currPosition.z);

		robotIKHandler(targetMesh, robotModel);

	}, 10);


	// for ( let i = 0; i < ARC_SEGMENTS; i ++ ) {

	// 	const t = i / ( ARC_SEGMENTS - 1 );
	// 	trajectoryCurve.getPoint( t, p );

	// 	targetMesh.position.set(p.x, p.y, p.z);

	// 	robotIKHandler(targetMesh, robotModel);

	// }

}

/////////////////////////////////// mesh} ///////////////////////////////////




function resizeRendererToDisplaySize(renderer) {

	const canvas = renderer.domElement;
	const pixelRatio = window.devicePixelRatio;
	const width = canvas.clientWidth * pixelRatio | 0;
	const height = canvas.clientHeight * pixelRatio | 0;
	const needResize = canvas.width !== width || canvas.height !== height;
	if (needResize) {
		renderer.setSize(width, height, false);
	}

	return needResize;
}

function render() {

	if (resizeRendererToDisplaySize(renderer)) {

		const canvas = renderer.domElement;
		camera.aspect = canvas.clientWidth / canvas.clientHeight;
		camera.updateProjectionMatrix();

	}

	renderer.render(scene, camera);

	requestAnimationFrame(render);

}

requestAnimationFrame(render);



/////////////////////////////////// Read JSON Text ///////////////////////////////////
// function loadFile(event) {
//     const file = event.target.files[0];
//     const reader = new FileReader();
//     reader.onload = function(event) {
//         const json = JSON.parse(event.target.result);
//         displayContent(json);
//     };
//     reader.readAsText(file);
// }

// function displayContent(json) {
//     const contentDiv = document.getElementById('content');
//     contentDiv.innerHTML = '<pre>' + JSON.stringify(json, null, 2) + '</pre>';

//     readFromText();
// }

export { readFromText, readFromJSONandOBJ }
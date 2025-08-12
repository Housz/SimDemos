import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { GUI } from 'three/addons/libs/lil-gui.module.min.js';

import { robotParser } from './robotParser.js';
import { robotCreator, robotCreatorWithObjText } from './robotCreator.js';
import { robotGUICreator } from './robotGUICreator.js';
import { robotWorkSpaceCreator } from './robotWorkSpaceCreator.js';
import { robotIKHandler } from './robotIKHandler.js'
import { robotRHIKHandler } from './robotRHIKHandler.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

import { Line2 } from 'three/addons/lines/Line2.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import { LineGeometry } from 'three/addons/lines/LineGeometry.js';

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

// const dlight = new THREE.DirectionalLight(0xeeeeee, 3);
// dlight.position.set(5, -5, 10);
// scene.add(dlight);

// scene.add(new THREE.AmbientLight(0xdddddd, 3));
// const light = new THREE.SpotLight(0xffffff, 2);
// light.position.set(200, 200, 50);
// light.angle = Math.PI * 0.2;
// light.decay = 0;
// light.castShadow = true;
// light.shadow.camera.near = 200;
// light.shadow.camera.far = 2000;
// light.shadow.bias = - 0.000222;
// light.shadow.mapSize.width = 1024;
// light.shadow.mapSize.height = 1024;
// // scene.add(light);

let pmremGenerator = new THREE.PMREMGenerator(renderer);
const roomEnv = new RoomEnvironment();
const envMap = pmremGenerator.fromScene(roomEnv, 0.04).texture;
// scene.environment = envMap;

// Add a brighter ambient light to the scene
const ambientLight = new THREE.AmbientLight(0xffffff, 1); // Increase intensity to make the scene brighter
scene.add(ambientLight);

// Add a directional light for additional brightness and enable shadows
const directionalLight = new THREE.DirectionalLight(0xffffff, 2);
transformControl.attach(directionalLight);
scene.add(transformControl);
directionalLight.position.set(5, 5, 10); // Position the light
directionalLight.castShadow = true; // Enable shadows

// Configure shadow properties
directionalLight.shadow.mapSize.width = 1024; // Shadow map resolution width
directionalLight.shadow.mapSize.height = 1024; // Shadow map resolution height
directionalLight.shadow.camera.near = 0.5; // Near clipping plane
directionalLight.shadow.camera.far = 50; // Far clipping plane
scene.add(directionalLight);


// Add another directional light at a different position
const directionalLight2 = new THREE.DirectionalLight(0xffffff, 1.5);
directionalLight2.position.set(5, 10, -10); // Position the second light
directionalLight2.castShadow = true; // Enable shadows for the second light

// Configure shadow properties for the second light
directionalLight2.shadow.mapSize.width = 1024; // Shadow map resolution width
directionalLight2.shadow.mapSize.height = 1024; // Shadow map resolution height
directionalLight2.shadow.camera.near = 0.5; // Near clipping plane
directionalLight2.shadow.camera.far = 50; // Far clipping plane

scene.add(directionalLight2);


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

// for trajectory
// target of end effector
// let targetGeo = new THREE.SphereGeometry(0.3, 16, 8);
let targetGeo = new THREE.SphereGeometry(0.6, 16, 8);
// let targetMat = new THREE.MeshStandardMaterial({ color: 0xffff00, transparent: true, opacity: 0.7 });
let targetMat = new THREE.MeshStandardMaterial({ color: 0x03ee00, transparent: true, opacity: 0.5 });
// let targetMat = new THREE.MeshStandardMaterial({ color: 0x00ff00, transparent: true, opacity: 0.7, wireframe: true });
let targetMesh = new THREE.Mesh(targetGeo, targetMat);
scene.add(targetMesh);


/**
 * Add a button to toggle targetMesh color
 */
const toggleBtn = document.createElement('button');
toggleBtn.textContent = '切换目标颜色';
toggleBtn.style.position = 'absolute';
toggleBtn.style.left = '10px';
toggleBtn.style.top = '10px';
toggleBtn.style.zIndex = 1000;
document.body.appendChild(toggleBtn);

let isGreen = true;
toggleBtn.addEventListener('click', () => {
	isGreen = !isGreen;
	targetMesh.material.color.set(isGreen ? 0xffff00 : 0x00ff00);
});

transformControl.attach(targetMesh);
scene.add(transformControl);
transformControl.size = 0.5;

let keyGroup = new THREE.Group();
scene.add(keyGroup);
let trajectoryCurve; // THREE.CatmullRomCurve3

const ARC_SEGMENTS = 200;


// let lineMat = new THREE.LineBasicMaterial({ color: 0x008811, linewidth: 10 });
// const geometry = new THREE.BufferGeometry();
// geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(ARC_SEGMENTS * 3), 3));

// let trajectoryLineMesh = new THREE.Line(
// 	geometry,
// 	lineMat
// ); // THREE.Line
// scene.add(trajectoryLineMesh);

// ================= Line2: 支持线宽 =================
const positions = new Float32Array(ARC_SEGMENTS * 3); // 扁平化 [x0,y0,z0, x1,y1,z1, ...]
const lineGeom = new LineGeometry();
lineGeom.setPositions(positions);

// 注意：linewidth 为像素宽度，需要设置 resolution
const lineMat = new LineMaterial({
	color: 0x008811,
	linewidth: 4.0,           // 像素，按需调整
	dashed: false,            // 需要虚线可设 true 并调用 computeLineDistances()
	// opacity: 0.95, transparent: true,  // 想半透明可打开
});

// 初始化一次分辨率（DPR 考虑）
{
	const size = new THREE.Vector2();
	renderer.getSize(size);
	const dpr = renderer.getPixelRatio();
	lineMat.resolution.set(size.x * dpr, size.y * dpr);
}

let trajectoryLineMesh = new Line2(lineGeom, lineMat);
trajectoryLineMesh.computeLineDistances(); // 非虚线也可以安全调用
trajectoryLineMesh.scale.set(1, 1, 1);
scene.add(trajectoryLineMesh);

// 窗口尺寸变化时，必须更新 resolution
window.addEventListener('resize', () => {
	const size = new THREE.Vector2();
	renderer.getSize(size);
	const dpr = renderer.getPixelRatio();
	lineMat.resolution.set(size.x * dpr, size.y * dpr);
});



// const loader = new GLTFLoader();
// loader.load('./models/juejinmian.gltf', (gltf) => {
// 	const model = gltf.scene;
// 	model.scale.set(1, 1, 1); // Adjust scale if needed
// 	model.position.set(0, 0, 0); // Adjust position if needed
// 	model.traverse((child) => {
// 		if (child.isMesh) {
// 			child.castShadow = true;
// 			child.receiveShadow = true;
// 		}
// 	});
// 	model.position.set(-4.0, model.position.y, 1.6);
// 	scene.add(model);

// 	// Create GUI for position adjustment
// 	const gui = new GUI();
// 	gui.domElement.style.position = 'absolute';
// 	gui.domElement.style.left = '0px';
// 	gui.domElement.style.opacity = '0.8';
// 	gui.domElement.style.top = '650px';

// 	gui.title("juejinmian Position");
// 	const positionFolder = gui.addFolder("Position");
// 	const positionControls = {
// 		x: model.position.x,
// 		y: model.position.y,
// 		z: model.position.z,
// 	};

// 	positionFolder.add(positionControls, 'x', -10, 10).onChange((value) => {
// 		model.position.x = value;
// 	});
// 	positionFolder.add(positionControls, 'y', -10, 10).onChange((value) => {
// 		model.position.y = value;
// 	});
// 	positionFolder.add(positionControls, 'z', -10, 10).onChange((value) => {
// 		model.position.z = value;
// 	});
// 	positionFolder.open();
// }, undefined, (error) => {
// 	console.error('An error occurred while loading the GLTF model:', error);
// });


let robot;
let robotModel;

// let gui = new GUI();


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

		robot = robotParser(data);

		// console.log("robot");
		// console.log(robot);

		robotModel = robotCreator(robot);

		robotModel.traverse((child) => {
			if (child.isMesh) {
				child.castShadow = true;
				child.receiveShadow = true;
			}
		});

		// transformControl.setMode( 'rotate' );
		// transformControl.attach(robotModel.children[0].children[0].children[0].children[0].children[0].children[0].children[0]);
		// scene.add(transformControl);

		console.log("robotModel");
		console.log(robotModel);

		// robotModel.scale.set(10, 10, 10);

		scene.add(robotModel);


		robotGUICreator(robot, robotModel);


		robotWorkSpaceCreator(robot, robotModel);

		// controller
		const gui = new GUI();
		gui.domElement.style.position = 'absolute';
		gui.domElement.style.left = '0px';
		gui.domElement.style.opacity = '0.8';
		gui.domElement.style.top = '500px';

		gui.title("controller");
		let controllerFolder = gui.addFolder("控制");
		let controllerObject = {
			clearAllKeys: () => { clearAllKeys(); },
			saveKeyPosition: () => { saveKeyPosition(); },
			playTrajectory: () => { playTrajectory(); }
		};

		controllerFolder.add(controllerObject, 'clearAllKeys');
		controllerFolder.add(controllerObject, 'saveKeyPosition');
		controllerFolder.add(controllerObject, 'playTrajectory');

		// transformControl.addEventListener('change', function (event) {
		// 	robotIKHandler(targetMesh, robotModel);
		// });


		transformControl.addEventListener('change', function (event) {
			robotRHIKHandler(targetMesh, robot, robotModel);
		});


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





// function clearAllKeys() {
// 	keyGroup.clear();
// 	// console.log(trajectoryLineMesh.geometry.attributes.position);

// 	trajectoryLineMesh.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(ARC_SEGMENTS * 3), 3));

// }

function clearAllKeys() {
	keyGroup.clear();

	// 清空几何：填零并更新
	positions.fill(0);
	lineGeom.setPositions(positions);
	trajectoryLineMesh.computeLineDistances();
}


// function saveKeyPosition() {
// 	const geometry = new THREE.SphereGeometry(0.3, 16, 8);
// 	const keyMesh = new THREE.Mesh(geometry, targetMat);

// 	// console.log(targetMesh.position);

// 	keyMesh.position.set(targetMesh.position.x, targetMesh.position.y, targetMesh.position.z);

// 	keyGroup.add(keyMesh);

// 	if (keyGroup.children.length > 1) {
// 		updateTrajectoryCurve();
// 	}

// }

function saveKeyPosition() {
	const geometry = new THREE.SphereGeometry(0.3, 16, 8);
	const keyMesh = new THREE.Mesh(geometry, targetMat);

	keyMesh.position.copy(targetMesh.position);
	keyGroup.add(keyMesh);

	if (keyGroup.children.length > 1) {
		updateTrajectoryCurve();
	} else {
		// 只有一个点时，线为空
		positions.fill(0);
		lineGeom.setPositions(positions);
		trajectoryLineMesh.computeLineDistances();
	}
}


// function updateTrajectoryCurve() {
// 	trajectoryCurve = new THREE.CatmullRomCurve3(
// 		keyGroup.children.map((child) => child.position),
// 		false,  // Closed
// 		"catmullrom",  // Type
// 		0.2  // Tension
// 	);
// 	// console.log(trajectoryCurve);

// 	// const points = trajectoryCurve.getPoints(ARC_SEGMENTS);
// 	// trajectoryLineMesh = new THREE.Line(
// 	// 	new THREE.BufferGeometry().setFromPoints(points),
// 	// 	new THREE.LineBasicMaterial({ color: 0x008811 })
// 	// );

// 	const position = trajectoryLineMesh.geometry.attributes.position;
// 	let p = new THREE.Vector3();
// 	for (let i = 0; i < ARC_SEGMENTS; i++) {

// 		const t = i / (ARC_SEGMENTS - 1);
// 		trajectoryCurve.getPoint(t, p);
// 		position.setXYZ(i, p.x, p.y, p.z);

// 	}

// 	position.needsUpdate = true;

// }

function updateTrajectoryCurve() {
	trajectoryCurve = new THREE.CatmullRomCurve3(
		keyGroup.children.map((child) => child.position),
		false,             // Closed
		'catmullrom',      // Type
		0.2                // Tension
	);

	// 采样 ARC_SEGMENTS 个点并写入扁平化数组
	const p = new THREE.Vector3();
	for (let i = 0; i < ARC_SEGMENTS; i++) {
		const t = i / (ARC_SEGMENTS - 1);
		trajectoryCurve.getPoint(t, p);
		const base = i * 3;
		positions[base + 0] = p.x;
		positions[base + 1] = p.y;
		positions[base + 2] = p.z;
	}

	// 更新几何
	lineGeom.setPositions(positions);
	trajectoryLineMesh.computeLineDistances(); // 如果设 dashed:true 必须调用；否则也可保留
}

function playTrajectory() {

	let speed = 0.02; // Fixed speed for the animation
	let t = 0; // Start at the beginning of the curve

	function animate() {
		t += speed / trajectoryCurve.getLength(); // Increment t based on speed and curve length

		if (t >= 1) {
			t = 1; // Clamp to 1 to ensure the animation ends cleanly
		}

		let currPosition = trajectoryCurve.getPointAt(t);
		targetMesh.position.set(currPosition.x, currPosition.y, currPosition.z);

		// robotIKHandler(targetMesh, robotModel);
		robotRHIKHandler(targetMesh, robot, robotModel);

		if (t < 1) {
			requestAnimationFrame(animate); // Continue animation
		}
	}

	animate(); // Start the animation

	// const p = new THREE.Vector3();

	// let ARC_SEGMENTS = 100; 
	// for ( let i = 0; i < ARC_SEGMENTS; i ++ ) {

	// 	const t = i / ( ARC_SEGMENTS - 1 );
	// 	trajectoryCurve.getPoint( t, p );

	// 	targetMesh.position.set(p.x, p.y, p.z);

	// 	// robotIKHandler(targetMesh, robotModel);
	// 	robotRHIKHandler(targetMesh, robot, robotModel);

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
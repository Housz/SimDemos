import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { GUI } from 'three/addons/libs/lil-gui.module.min.js';


let threeScene;
let renderer;
let camera;
let cameraControl;

let gravity = -10.0;
let dt = 1.0 / 60.0;
let numSubSteps = 100000;


class Pendulum {
	constructor() {
		this.ws = [0.0, 1.0, 1.0, 1.0]; // inverse of masses
		this.length = 1;
		this.pos = [
			{ x: 0.0, y: 0.0, z: 0.0 },
			{ x: 1.0, y: 0.0, z: 0.0 },
			{ x: 1.0, y: 1.0, z: 0.0 },
			{ x: 1.0, y: 1.0, z: 1.0 }
		];
		this.prevPos = [
			{ x: 0.0, y: 0.0, z: 0.0 },
			{ x: 1.0, y: 0.0, z: 0.0 },
			{ x: 1.0, y: 1.0, z: 0.0 },
			{ x: 1.0, y: 1.0, z: 1.0 }
		];
		this.vel = [
			{ x: .0, y: .0, z: .0 },
			{ x: .0, y: .0, z: .0 },
			{ x: .0, y: .0, z: .0 },
			{ x: .0, y: .0, z: .0 }
		];

		// visualization
		this.visMeshes = [];
		// ball
		for (let i = 0; i <= 3; i++) {
			let ball = new THREE.SphereGeometry(0.1, 32, 32);
			let material = new THREE.MeshPhongMaterial({ color: 0xff0000 });
			this.visMeshes.push(new THREE.Mesh(ball, material));
			threeScene.add(this.visMeshes[i]);
		};

		// link
		let linkPoints = [];
		let material = new THREE.LineBasicMaterial({ color: 0xcccccc });
		let geometry = new THREE.BufferGeometry().setFromPoints(linkPoints);
		this.link = new THREE.Line(geometry, material);
		threeScene.add(this.link);
	}

	simulate(sdt) {
		// update position (caused by external forces)
		for (let i = 1; i <= 3; i++) {
			this.vel[i].y += sdt * gravity;

			this.prevPos[i].x = this.pos[i].x;
			this.prevPos[i].y = this.pos[i].y;
			this.prevPos[i].z = this.pos[i].z;

			this.pos[i].x += sdt * this.vel[i].x;
			this.pos[i].y += sdt * this.vel[i].y;
			this.pos[i].z += sdt * this.vel[i].z;
		}

		// solve constraint
		for (let i = 1; i <= 3; i++) {
			let dx = this.pos[i].x - this.pos[i - 1].x;
			let dy = this.pos[i].y - this.pos[i - 1].y;
			let dz = this.pos[i].z - this.pos[i - 1].z;
			let d = Math.sqrt(dx * dx + dy * dy + dz * dz);

			let w0 = this.ws[i - 1];
			let w1 = this.ws[i];
			let corr = (this.length - d) / d / (w0 + w1);

			this.pos[i - 1].x -= w0 * corr * dx;
			this.pos[i - 1].y -= w0 * corr * dy;
			this.pos[i - 1].z -= w0 * corr * dz;

			this.pos[i].x += w1 * corr * dx;
			this.pos[i].y += w1 * corr * dy;
			this.pos[i].z += w1 * corr * dz;
		}

		// update velocity
		for (let i = 1; i <= 3; i++) {
			this.vel[i].x = (this.pos[i].x - this.prevPos[i].x) / sdt;
			this.vel[i].y = (this.pos[i].y - this.prevPos[i].y) / sdt;
			this.vel[i].z = (this.pos[i].z - this.prevPos[i].z) / sdt;
		}
	}
	draw() {

		//draw ball
		for (let i = 0; i <= 3; i++) {
			let posThree = new THREE.Vector3(this.pos[i].x, this.pos[i].y, this.pos[i].z);
			this.visMeshes[i].position.copy(posThree);
		}

		// draw link
		threeScene.remove(this.link);
		let linkPoints = [];
		for (let i = 0; i <= 3; i++) {
			linkPoints.push(new THREE.Vector3(this.pos[i].x, this.pos[i].y, this.pos[i].z));
		}
		let material = new THREE.LineBasicMaterial({ color: 0xcccccc });
		let geometry = new THREE.BufferGeometry().setFromPoints(linkPoints);
		this.link = new THREE.Line(geometry, material);
		threeScene.add(this.link);
	}
}



function initThreeScene() {
	threeScene = new THREE.Scene();

	threeScene.background = new THREE.Color(0xaaccff);

	// Lights

	threeScene.add(new THREE.AmbientLight(0x505050));
	threeScene.fog = new THREE.Fog(0x000000, 0, 15);

	var spotLight = new THREE.SpotLight(0xffffff, 3);
	spotLight.angle = Math.PI / 5;
	spotLight.penumbra = 0.2;
	spotLight.position.set(0, 3, 0);
	spotLight.castShadow = true;
	spotLight.shadow.camera.near = 3;
	spotLight.shadow.camera.far = 10;
	spotLight.shadow.mapSize.width = 1024;
	spotLight.shadow.mapSize.height = 1024;
	threeScene.add(spotLight);

	var dirLight = new THREE.DirectionalLight(0xffffff, 3);
	dirLight.position.set(5, 5, 5);
	dirLight.castShadow = true;
	dirLight.shadow.camera.near = 1;
	dirLight.shadow.camera.far = 10;

	dirLight.shadow.camera.right = 1;
	dirLight.shadow.camera.left = - 1;
	dirLight.shadow.camera.top = 1;
	dirLight.shadow.camera.bottom = - 1;

	dirLight.shadow.mapSize.width = 1024;
	dirLight.shadow.mapSize.height = 1024;
	threeScene.add(dirLight);

	// Geometry

	// var ground = new THREE.Mesh(
	// 	new THREE.PlaneBufferGeometry(20, 20, 1, 1),
	// 	new THREE.MeshPhongMaterial({ color: 0xa0adaf, shininess: 150 })
	// );

	// ground.rotation.x = - Math.PI / 2; // rotates X/Y to X/Z
	// ground.receiveShadow = true;
	// threeScene.add(ground);

	var helper = new THREE.GridHelper(20, 20);
	helper.material.opacity = 1.0;
	helper.material.transparent = true;
	helper.position.set(0, 0.002, 0);
	threeScene.add(helper);

	// Renderer

	renderer = new THREE.WebGLRenderer();
	renderer.shadowMap.enabled = true;
	renderer.setPixelRatio(window.devicePixelRatio);
	renderer.setSize(0.8 * window.innerWidth, 0.8 * window.innerHeight);
	window.addEventListener('resize', onWindowResize, false);
	container.appendChild(renderer.domElement);

	// Camera

	camera = new THREE.PerspectiveCamera(70, window.innerWidth / window.innerHeight, 0.01, 100);
	camera.position.set(0, 1, 4);
	camera.updateMatrixWorld();

	threeScene.add(camera);

	cameraControl = new OrbitControls(camera, renderer.domElement);
	cameraControl.zoomSpeed = 2.0;
	cameraControl.panSpeed = 0.4;
}

function onWindowResize() {
	camera.aspect = window.innerWidth / window.innerHeight;
	camera.updateProjectionMatrix();
	renderer.setSize(window.innerWidth, window.innerHeight);
}

document.getElementById("stepsSlider").oninput = function () {
	var steps = [1, 5, 10, 100, 1000, 10000, 100000];
	numSubSteps = steps[Number(this.value)];
	document.getElementById("steps").innerHTML = numSubSteps.toString();
}



function simulate() {
	let sdt = dt / numSubSteps;
	for (let step = 0; step < numSubSteps; step++) {
		pendulum.simulate(sdt);
	}
	pendulum.draw();
}

function update() {

	simulate();
	renderer.render(threeScene, camera);
	cameraControl.update();

	requestAnimationFrame(update);

}


initThreeScene();
let pendulum = new Pendulum();
update();

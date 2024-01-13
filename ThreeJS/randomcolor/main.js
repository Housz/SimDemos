/**
 * Housz
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';


/////////////////////////////////// scene ///////////////////////////////////
const canvas = document.querySelector('#c');
const renderer = new THREE.WebGLRenderer({ antialias: true, canvas });

const fov = 75;
const aspect = 2;
const near = 0.01;
const far = 1000;
const camera = new THREE.PerspectiveCamera(fov, aspect, near, far);
camera.position.z = 10;

const controls = new OrbitControls(camera, renderer.domElement);

const scene = new THREE.Scene();
{
	const color = 0xFFFFFF;
	const intensity = 3;
	const light = new THREE.DirectionalLight(color, intensity);
	light.position.set(100, 100, 100);
	scene.add(light);
	scene.add(new THREE.AmbientLight(0xffffff));
}
/////////////////////////////////// scene ///////////////////////////////////


/////////////////////////////////// mesh ///////////////////////////////////

const geometry = new THREE.IcosahedronGeometry(0.5, 3);
const material = new THREE.MeshPhongMaterial({ color: 0xffffff });

let amount = 10;
const count = Math.pow(amount, 3);
let mesh = new THREE.InstancedMesh(geometry, material, count);

let i = 0;
const offset = (amount - 1) / 2;

const matrix = new THREE.Matrix4();

for (let x = 0; x < amount; x++) {

	for (let y = 0; y < amount; y++) {

		for (let z = 0; z < amount; z++) {

			matrix.setPosition(offset - x, offset - y, offset - z);

			let color = new THREE.Color().setHSL(Math.random(), 0.5, 0.5);

			mesh.setMatrixAt(i, matrix);
			mesh.setColorAt(i, color);

			i++;

		}
	}
}

scene.add( mesh );

/////////////////////////////////// mesh ///////////////////////////////////




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

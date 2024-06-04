import * as THREE from 'three';
import { robotConstraintHandler } from './robotConstraintHandler.js'
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';

const objloader = new OBJLoader();


function robotCreator(robot) {

	let robotModel = new THREE.Group(); // base_link
	robotModel.name = robot.name;

	let base_link = new THREE.Group();
	base_link.name = "base_link";
	robotModel.add(base_link);

	robot.joints.forEach(joint => {

		let jointModel = createJointModel(joint);

		// 1. parent link
		let parentLinkInRobot = getChildByName(robotModel, joint.parent);

		if (parentLinkInRobot == null) {

			// create joint.parent link
			let parentLink = robot.linkMap.get(joint.parent);

			let parentLinkModelPromise = createLinkModel(parentLink);

			parentLinkModelPromise.then
				(
					result => {
						let parentLinkModel = result;

						parentLinkModel.name = parentLink.name;

						parentLinkModel.add(jointModel);
						robotModel.add(parentLinkModel);
					}

				);
		}
		else {
			parentLinkInRobot.add(jointModel);
		}

		// 2. child link
		let childLinkInRobot = getChildByName(robotModel, joint.child);
		if (childLinkInRobot == null) {

			// create joint.child link
			let childLink = robot.linkMap.get(joint.child);

			let childLinkModelPromise = createLinkModel(childLink);

			childLinkModelPromise.then
				(
					result => {
						let childLinkModel = result;
						
						childLinkModel.name = childLink.name;

						jointModel.add(childLinkModel);

					}
				);


		} else {
			console.error("存在闭环：" + childLink.name);
		}

	});

	// constraints init
	robotConstraintHandler(robot, robotModel, null);


	return robotModel;

}

function getChildByName(object, childName) {

	let child = null;

	object.traverse(currChild => {
		if (currChild.name == childName) {
			if (child == null) {
				child = currChild;
			}
			else {
				console.error("存在重复节点：" + childName);
			}
		}
	});

	return child;

}

function createJointModel(joint) {

	if (joint.type == "revolute") {
		// const geometry = new THREE.CylinderGeometry(.2, .2, .5, 16);
		const geometry = new THREE.CylinderGeometry(.0002, .0002, .0005, 16);
		geometry.rotateZ(Math.PI / 2); // axis to [1, 0, 0]

		let currAxis = new THREE.Vector3(1, 0, 0);
		let targetAxis = new THREE.Vector3(joint.axis.x, joint.axis.y, joint.axis.z);

		// angle between [1, 0, 0] and targetAxis
		let rotationAngle = currAxis.angleTo(targetAxis);
		// rotation axis from currAxis to targetAxis
		let rotationAxis = new THREE.Vector3();
		rotationAxis.crossVectors(currAxis, targetAxis);
		rotationAxis.normalize();
		// console.log(rotationAngle);
		// console.log(rotationAxis);

		let mat4 = new THREE.Matrix4();
		mat4.makeRotationAxis(rotationAxis, rotationAngle);

		geometry.applyMatrix4(mat4);
		const material = new THREE.MeshPhongMaterial({ color: 0xeeeeee, transparent: true, opacity: 0. });
		const cylinder = new THREE.Mesh(geometry, material);
		cylinder.castShadow = true;

		// console.log(joint);

		cylinder.rotation.set(joint.origin_orientation.x, joint.origin_orientation.y, joint.origin_orientation.z);
		cylinder.position.set(joint.origin_translation.x, joint.origin_translation.y, joint.origin_translation.z);


		// cylinder.setRotationFromEuler(new THREE.Euler(joint.axis.x, joint.axis.y, joint.axis.z));


		cylinder.name = joint.name;

		return cylinder;
	}

	else if (joint.type == "fixed") {

		let fixedJointModel = new THREE.Object3D();
		fixedJointModel.name = joint.name;

		fixedJointModel.rotation.set(joint.origin_orientation.x, joint.origin_orientation.y, joint.origin_orientation.z);
		fixedJointModel.position.set(joint.origin_translation.x, joint.origin_translation.y, joint.origin_translation.z);

		return fixedJointModel;

	}

	else {
		// todo
		const geometry = new THREE.CylinderGeometry(.2, .2, .5, 16);
		const material = new THREE.MeshPhongMaterial({ color: 0xeeeeee });
		const cylinder = new THREE.Mesh(geometry, material);
		cylinder.rotateZ(Math.PI / 2);
		cylinder.castShadow = true;

		return cylinder;
	}

}

async function createLinkModel(link) {

	let visual = link.visual;

	// visual offset
	// todo
	let visual_origin_translation = visual.origin_translation;
	let visual_origin_orientation = visual.origin_orientation;

	// geometry
	let geometry_type = visual.geometry.type;

	if (geometry_type == "box") {
		let x = visual.geometry.x;
		let y = visual.geometry.y;
		let z = visual.geometry.z;

		// material
		let color = new THREE.Color(visual.material.color[0] / 255, visual.material.color[1] / 255, visual.material.color[2] / 255);
		const material = new THREE.MeshPhongMaterial();
		material.color = color;
		if (visual.material.opacity) {
			material.transparent = true;
			material.opacity = visual.material.opacity;
		}

		const geometry = new THREE.BoxGeometry(x, y, z);
		geometry.applyMatrix4(new THREE.Matrix4().makeTranslation(x / 2, 0, 0));
		const box = new THREE.Mesh(geometry, material);
		box.castShadow = true;

		box.name = link.name;

		return box;
	}
	else if (geometry_type == "cylinder") {

		let r1 = visual.geometry.r1;
		let r2 = visual.geometry.r2;
		let h = visual.geometry.h;

		const geometry = new THREE.CylinderGeometry(r1, r2, h, 16);
		geometry.rotateZ(Math.PI / 2); // axis to [1, 0, 0]

		geometry.applyMatrix4(new THREE.Matrix4().makeTranslation(h / 2, 0, 0));

		// material
		let color = new THREE.Color(visual.material.color[0] / 255, visual.material.color[1] / 255, visual.material.color[2] / 255);
		const material = new THREE.MeshPhongMaterial();
		material.color = color;
		if (visual.material.opacity) {
			material.transparent = true;
			material.opacity = visual.material.opacity;
		}

		const cylinder = new THREE.Mesh(geometry, material);

		cylinder.name = link.name;

		return cylinder;

	}
	else if (geometry_type == "mesh") {

		let meshURL = visual.geometry.url;

		let container = new THREE.Group();

		objloader.load(
			meshURL,
			(object) => {

				let color = new THREE.Color(visual.material.color[0] / 255, visual.material.color[1] / 255, visual.material.color[2] / 255);
				const material = new THREE.MeshPhongMaterial();
				material.color = color;

				object.children[0].material = material;

				object.children[0].rotation.set(visual_origin_orientation[0], visual_origin_orientation[1], visual_origin_orientation[2]);
				object.children[0].position.set(visual_origin_translation[0], visual_origin_translation[1], visual_origin_translation[2]);

				// visual_origin_orientation
				object.children[0].name = link.name;

				container.add(object.children[0]);

				// return container;

			}
		);
		return container;
	}
	else {
		// console.error("不支持的关节几何类型：" + geometry_type);
	}





}

export { robotCreator }
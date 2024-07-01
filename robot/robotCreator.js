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

	// fixedJointModel.position.set();

	/**
	 * 1. build topology tree
	 */

	let linkModels = [];

	robot.joints.forEach(joint => {



		let jointModel = createJointModel(joint);

		// 1. parent link
		let parentLinkInRobot = getChildByName(robotModel, joint.parent);

		if (parentLinkInRobot == null) {

			let parentLinkModel = new THREE.Mesh();

			parentLinkModel.isLink = true;

			parentLinkModel.name = joint.parent;

			parentLinkModel.add(jointModel);

			robotModel.add(parentLinkModel);

			linkModels.push(parentLinkModel);
		}
		else {
			parentLinkInRobot.add(jointModel);
		}


		// 2. child link
		let childLinkInRobot = getChildByName(robotModel, joint.child);
		if (childLinkInRobot == null) {

			let childLinkModel = new THREE.Mesh();
			childLinkModel.isLink = true;

			childLinkModel.name = joint.child;

			jointModel.add(childLinkModel);

			linkModels.push(childLinkModel);
		}
		else {
			console.error("存在闭环：" + childLink.name);
		}

	});



	/**
	 * 2. creat / load visual meshes of links
	 */

	// robotModel.traverse(async currChild => {
	// 	if (currChild.isLink) {
	// 		console.log(currChild);

	// 		let linkName = currChild.name;
	// 		let link = robot.linkMap.get(linkName);

	// 		let LinkModelPromise = createLinkModel(link);

	// 		let result = await LinkModelPromise;

	// 		let linkModelParent = currChild.parent;
	// 		let linkModelChildren = currChild.children;

	// 		// result.children = linkModelChildren;
	// 		linkModelChildren.forEach(child => {
	// 			result.add(child);
	// 		});

	// 		linkModelParent.add(result);

	// 		linkModelParent.remove(currChild);
	// 	}
	// });



	linkModels.forEach(async linkModel => {

		let linkName = linkModel.name;
		let link = robot.linkMap.get(linkName);

		// if (link.visual.geometry.type == 'mesh') {
		let LinkModelPromise = createLinkModel(link);

		let result = await LinkModelPromise;

		let visual = link.visual;

		// visual offset
		let visual_origin_translation = visual.origin_translation;
		let visual_origin_orientation = visual.origin_orientation;

		// console.log(result);
		// console.log(result.rotation);
		// console.log(result.position);



		linkModel.geometry = result.geometry.clone();
		linkModel.material = result.material.clone();
		linkModel.rotation.set(visual_origin_orientation[0], visual_origin_orientation[1], visual_origin_orientation[2]);
		linkModel.position.set(visual_origin_translation[0], visual_origin_translation[1], visual_origin_translation[2]);

		// console.log(linkModel);

		robotModel.updateWorldMatrix(true, true);

		// robotModel.updateMatrixWorld(true);


	});


	// let linkModelParent = linkModel.parent;
	// let linkModelChildren = linkModel.children;

	// result.children = linkModelChildren;

	// // linkModelChildren.forEach(child => {
	// // 	result.add(child);
	// // });

	// linkModelParent.add(result);

	// linkModelParent.remove(linkModel);


	/**
	 * constraints init
	 */
	// init jointL_base_angle
	robot.constraints.forEach(constraint => {
		
		// let jointA = robot.jointMap.get(constraint.jointA);
		let jointAModel = getChildByName(robotModel, constraint.jointA);

		// let jointB = robot.jointMap.get(constraint.jointB);
		let jointBModel = getChildByName(robotModel, constraint.jointB);

		// let jointL = robot.jointMap.get(constraint.jointL);
		let jointLModel = getChildByName(robotModel, constraint.jointL);


		let A_world = new THREE.Vector3();
		jointAModel.getWorldPosition(A_world);
		// console.log(A_world);

		let B_world = new THREE.Vector3();
		jointBModel.getWorldPosition(B_world);

		let L_world = new THREE.Vector3();
		jointLModel.getWorldPosition(L_world);

		let LB = new THREE.Vector3();
		LB.subVectors(B_world, L_world);
		let LA = new THREE.Vector3();
		LA.subVectors(A_world, L_world);

		constraint.jointL_base_angle = LA.angleTo(LB);

		console.log(constraint.name);
		console.log(constraint.jointL_base_angle);
		
	});

	// robotConstraintHandler(robot, robotModel, null);

	return robotModel;

}

function getChildByName(object, childName) {

	// console.log("--find--" + childName);

	let child = null;

	object.traverse(currChild => {
		// console.log(currChild.name);
		if (currChild.name == childName) {

			// console.log(">");
			// console.log(currChild.name);

			if (child == null) {
				child = currChild;
			}
			else {
				console.error("存在重复节点：" + childName);
			}
		}
	});

	// console.log("||||||||||||||||||||||||||");


	return child;

}

function createJointModel(joint) {


	if (joint.type == "fixed") {

		let fixedJointModel = new THREE.Object3D();
		fixedJointModel.name = joint.name;

		fixedJointModel.rotation.set(joint.origin_orientation.x, joint.origin_orientation.y, joint.origin_orientation.z);
		fixedJointModel.position.set(joint.origin_translation.x, joint.origin_translation.y, joint.origin_translation.z);

		fixedJointModel.isJoint = true;



		return fixedJointModel;

	}

	else if (joint.type == "revolute") {
		
		const geometry = new THREE.CylinderGeometry(.2, .2, .51, 16);
		// const geometry = new THREE.CylinderGeometry(.0002, .0002, .0005, 16);
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
		const material = new THREE.MeshPhongMaterial({ color: 0xeeeeee, transparent: true, opacity: 0.7 });
		const cylinder = new THREE.Mesh(geometry, material);
		cylinder.castShadow = true;

		// console.log(joint);

		cylinder.rotation.set(joint.origin_orientation.x, joint.origin_orientation.y, joint.origin_orientation.z);
		cylinder.position.set(joint.origin_translation.x, joint.origin_translation.y, joint.origin_translation.z);


		// cylinder.setRotationFromEuler(new THREE.Euler(joint.axis.x, joint.axis.y, joint.axis.z));

		cylinder.name = joint.name;

		cylinder.isJoint = true;

		return cylinder;
	}

	else if (joint.type = "prismatic") {

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

		cylinder.isJoint = true;

		return cylinder;


	}

	else {
		// todo
		const geometry = new THREE.CylinderGeometry(.2, .2, .5, 16);
		const material = new THREE.MeshPhongMaterial({ color: 0xeeeeee });
		const cylinder = new THREE.Mesh(geometry, material);
		cylinder.rotateZ(Math.PI / 2);
		cylinder.castShadow = true;

		cylinder.isJoint = true;

		return cylinder;
	}

}

async function createLinkModel(link) {

	let visual = link.visual;

	// visual offset
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

		geometry.translate(visual_origin_translation[0], visual_origin_translation[1], visual_origin_translation[2]);
		// todo geometry rotation

		const box = new THREE.Mesh(geometry, material);
		box.castShadow = true;

		box.name = link.name;

		box.isLink = true;

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

		cylinder.isLink = true;

		return cylinder;

	}
	else if (geometry_type == "mesh") {

		let meshURL = visual.geometry.url;

		let meshScale = visual.geometry.scale;

		let [object] = await Promise.all([objloader.loadAsync(meshURL)]);
		object.children[0].name = link.name;

		let color = new THREE.Color(visual.material.color[0] / 255, visual.material.color[1] / 255, visual.material.color[2] / 255);
		const material = new THREE.MeshPhongMaterial();
		material.color = color;

		object.children[0].material = material;

		object.children[0].geometry.scale(meshScale[0], meshScale[1], meshScale[2]);

		object.children[0].geometry.rotateX(-1.57);


		// object.children[0].rotation.set(visual_origin_orientation[0], visual_origin_orientation[1], visual_origin_orientation[2]);
		// object.children[0].position.set(visual_origin_translation[0], visual_origin_translation[1], visual_origin_translation[2]);

		object.children[0].isLink = true;

		return object.children[0];

	}
	else {
		// console.error("不支持的关节几何类型：" + geometry_type);
	}

}

export { robotCreator }
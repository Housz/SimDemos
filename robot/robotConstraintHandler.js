import * as THREE from 'three';

function robotConstraintHandler(robot, robotModel, constraintName) {

	// drive (Prismatic length change -> jointL angle change)
	if (constraintName) {
		let dirveConstraint = robot.constraintMap.get(constraintName);

		// let a = robot.jointMap.get(dirveConstraint.jointA).origin_translation.length();
		// let b = robot.jointMap.get(dirveConstraint.jointB).origin_translation.length();

		let jointLModel = getChildByName(robotModel, dirveConstraint.jointL);
		let L_world = new THREE.Vector3();
		jointLModel.getWorldPosition(L_world);

		let jointAModel = getChildByName(robotModel, dirveConstraint.jointA);
		let A_world = new THREE.Vector3();
		jointAModel.getWorldPosition(A_world);

		let jointBModel = getChildByName(robotModel, dirveConstraint.jointB);
		let B_world = new THREE.Vector3();
		jointBModel.getWorldPosition(B_world);

		let vec_a = new THREE.Vector3();
		vec_a.subVectors(A_world, L_world);
		let a = vec_a.length();

		let vec_b = new THREE.Vector3();
		vec_b.subVectors(B_world, L_world);
		let b = vec_b.length();

		let l = dirveConstraint.length;
		// let jointL_base_angle = dirveConstraint.jointL_base_angle;
		// let jointL_base_angle = 

		let angleL = computeAngle(a, b, l);
		// angleL -= jointL_base_angle;
		angleL -= dirveConstraint.jointL_base_angle;

		console.log("Constraint:");
		console.log(a);
		console.log(b);
		console.log(l);
		console.log(angleL);
		console.log("");

		let jointL = robot.jointMap.get(dirveConstraint.jointL);
		// let jointLModel = getChildByName(robotModel, dirveConstraint.jointL);


		updateRevoluteAngle(jointL, jointLModel, angleL);
	}

	// jointA and jointB lookAt each other
	robot.constraints.forEach(constraint => {

		let jointA = robot.jointMap.get(constraint.jointA);
		let jointAModel = getChildByName(robotModel, constraint.jointA);

		let jointB = robot.jointMap.get(constraint.jointB);
		let jointBModel = getChildByName(robotModel, constraint.jointB);


		let A_world = new THREE.Vector3();
		jointAModel.getWorldPosition(A_world);
		// console.log(A_world);
		let B_world = new THREE.Vector3();
		jointBModel.getWorldPosition(B_world);
		// console.log(B_world);

		// let AB_world = new THREE.Vector3();
		// AB_world.subVectors(B_world, A_world);
		// let BA_world = AB_world.clone().negate();

		// const origin = A_world;
		// const length = AB_world.length();
		// const hex = 0xffff00;
		// const arrowHelper = new THREE.ArrowHelper( AB_world.normalize(), origin, length, hex );
		// robotModel.add( arrowHelper );

		jointAModel.rotation.set(0, 0, 0);
		jointBModel.rotation.set(0, 0, 0);

		let B_local_in_A = jointAModel.worldToLocal(B_world);
		let A_local_in_B = jointBModel.worldToLocal(A_world);

		// const origin = new THREE.Vector3(0, 0, 0);
		// const length = B_local_in_A.length();
		// const hex = 0xffff00;
		// const arrowHelper = new THREE.ArrowHelper( B_local_in_A.normalize(), origin, length, hex );
		// jointAModel.add( arrowHelper );

		// rotate jointA
		let currAxis = new THREE.Vector3(1, 0, 0);
		let targetAxis = B_local_in_A;

		let rotationAngle = currAxis.angleTo(targetAxis);
		let rotationAxis = new THREE.Vector3();
		rotationAxis.crossVectors(currAxis, targetAxis);
		rotationAxis.normalize();
		jointAModel.rotateOnAxis(rotationAxis, rotationAngle);


		// rotate jointB
		currAxis = new THREE.Vector3(1, 0, 0);
		targetAxis = A_local_in_B;

		rotationAngle = currAxis.angleTo(targetAxis);
		rotationAxis = new THREE.Vector3();
		rotationAxis.crossVectors(currAxis, targetAxis);
		rotationAxis.normalize();
		jointBModel.rotateOnAxis(rotationAxis, rotationAngle);


	});


}


function computeAngle(a, b, l) {

	let angleL = Math.acos((a * a + b * b - l * l) / (2 * a * b));

	if (angleL > 0) {
		return angleL;
	}
	else {
		alert("ERROR! 油缸超出运动范围！")
	}

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

function updateRevoluteAngle(joint, jointModel, angle) {

	if (joint.axis.x) {
		jointModel.rotation.set(angle, 0, 0);
	}
	else if (joint.axis.y) {
		jointModel.rotation.set(0, angle, 0);
	}
	else if (joint.axis.z) {
		jointModel.rotation.set(0, 0, angle);
	}
	else {
		console.error("不是Revolute关节!");
	}

}

function updatePrismaticCylinder() {

}


export { robotConstraintHandler };
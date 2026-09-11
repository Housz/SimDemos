import * as THREE from 'three';

function robotIKHandler(targetModel, robotModel) {

	let origin = robotModel.position;
	let target = targetModel.position;

	let endEffector = getChildByName(robotModel, "end_effector");
	let link1_link2_joint = getChildByName(robotModel, "link1_link2_joint");
	let link2_link3_joint = getChildByName(robotModel, "link2_link3_joint");
	let link3_link4_joint = getChildByName(robotModel, "link3_link4_joint");


	let endPos = new THREE.Vector3();
	endEffector.getWorldPosition(endPos);

	let p = new THREE.Vector3();
	p.subVectors(target, origin);

	let d1 = 2;
	let d2 = 1;
    let d3 = 2.15;
	
	// ref: Trajectory tracking of Stanford robot manipulator by fractional-order sliding mode control
	let theta1 = Math.atan2(p.x, p.z) - Math.atan2(1, Math.sqrt(p.x * p.x + p.z * p.z - d2 * d2));

	let theta2 = Math.atan2(d1 - p.y, Math.sqrt(p.x * p.x + p.z * p.z - d2 * d2))

	let theta3 = Math.sqrt((d1 - p.y) * (d1 - p.y) + (p.x * p.x + p.z * p.z - d2 * d2));
	theta3 -= d3;


	if (isNaN(theta1) || isNaN(theta2) || !(theta3 >= -1.5 && theta3 <= 1.5)) {
		// console.log("error");
		targetModel.material.color.setHex(0xaa0000);
	}
	else {
		targetModel.material.color.setHex(0x00aa00);
		link1_link2_joint.rotation.y = theta1;
		link2_link3_joint.rotation.x = theta2;
		link3_link4_joint.position.z = theta3;
	}
}

function vectorMetric(v1, v2) {
	return Math.sqrt((v1.x - v2.x) * (v1.x - v2.x), (v1.y - v2.y) * (v1.y - v2.y), (v1.z - v2.z) * (v1.z - v2.z));
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




export { robotIKHandler };
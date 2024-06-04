import * as THREE from 'three';
import { GUI } from 'three/addons/libs/lil-gui.module.min.js';
import { robotConstraintHandler } from './robotConstraintHandler.js'

function robotGUICreator(robot, robotModel) {

	const gui = new GUI();

	gui.title(robot.name);


	robot.joints.forEach(joint => {

		let jointModel = getChildByName(robotModel, joint.name);

		let folder = gui.addFolder(joint.name + " (" + joint.type + ")");

		let lower = joint.limit.lower;
		let upper = joint.limit.upper;

		if (joint.type == "revolute") {
			// console.log(jointModel.rotation);
			let jointGUIObject = {
				x: jointModel.rotation.x,
				y: jointModel.rotation.y,
				z: jointModel.rotation.z,
			};

			let sliderX = folder.add(jointGUIObject, 'x', lower, upper).onChange(value => {
				jointModel.rotation.x = value;
			});
			sliderX.listen();
			if (joint.axis.x == 0) {
				sliderX.disable();
			}

			let sliderY = folder.add(jointGUIObject, 'y', lower, upper).onChange(value => {
				jointModel.rotation.y = value;
			});
			sliderY.listen();
			if (joint.axis.y == 0) {
				sliderY.disable();
			}

			let sliderZ = folder.add(jointGUIObject, 'z', lower, upper).onChange(value => {
				jointModel.rotation.z = value;
			});
			sliderZ.listen();
			if (joint.axis.z == 0) {
				sliderZ.disable();
			}

		}

		else if(joint.type == "prismatic") {

			let jointGUIObject = {
				x: jointModel.position.x,
				y: jointModel.position.y,
				z: jointModel.position.z,
			};

			let sliderX = folder.add(jointGUIObject, 'x', lower, upper).onChange(value => {
				jointModel.position.x = value;
			});
			sliderX.listen();
			if (joint.axis.x == 0) {
				sliderX.disable();
			}

			let sliderY = folder.add(jointGUIObject, 'y', lower, upper).onChange(value => {
				jointModel.position.y = value;
			});
			sliderY.listen();
			if (joint.axis.y == 0) {
				sliderY.disable();
			}

			let sliderZ = folder.add(jointGUIObject, 'z', lower, upper).onChange(value => {
				jointModel.position.z = value;
			});
			sliderZ.listen();
			if (joint.axis.z == 0) {
				sliderZ.disable();
			}
		}

		else {
			// todo
		}

	});

	robot.constraints.forEach(constraint => {

		if (constraint.type == "triangle-prismatic") {

			let folder = gui.addFolder(constraint.name + " (prismatic)");

			let lower = constraint.limit.lower;
			let upper = constraint.limit.upper;

			let constraintGUIObject = {
				length: constraint.length
			};

			let length = folder.add(constraintGUIObject, 'length', lower, upper).onChange(value => {

				constraint.length = value;
				
				robotConstraintHandler(robot, robotModel, constraint.name);

			});


		}
		else {
			// todo
		}
	});


	// // controller
	// let controllerFolder = gui.addFolder("控制");
	// let controllerObject = {
	// 	saveKeyPosition: function() { console.log("fun"); },
	// 	playTrajectory: function() { console.log("trajectory");}
	// };

	// controllerFolder.add( controllerObject, 'saveKeyPosition' ); 
	// controllerFolder.add( controllerObject, 'playTrajectory' ); 

	// tail


	return gui;

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


export { robotGUICreator };
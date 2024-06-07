import * as THREE from 'three';
import { GUI } from 'three/addons/libs/lil-gui.module.min.js';
import { robotConstraintHandler } from './robotConstraintHandler.js'

function robotGUICreator(robot, robotModel) {

	const gui = new GUI();

	gui.title(robot.name);


	robot.joints.forEach(joint => {

		let jointModel = getChildByName(robotModel, joint.name);

		let folder = gui.addFolder(joint.name + " (" + joint.type + ")");


		if (joint.type == "revolute") {

			let lower = joint.limit.lower;
			let upper = joint.limit.upper;

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
				// robotConstraintHandler(robot, robotModel, robot.constraints[0].name);
			});
			sliderZ.listen();
			if (joint.axis.z == 0) {
				sliderZ.disable();
			}

		}

		else if(joint.type == "prismatic") {

			let lower = joint.limit.lower;
			let upper = joint.limit.upper;

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

		else if(joint.type == "fixed"){
			
			if (joint.parent != "base_link") {
				return;
			}

			folder.title("joint0 (world coordinates)");

			let lower = 0;
			let upper = 5;

			let jointTranslationGUIObject = {
				translationX: jointModel.position.x,
				translationY: jointModel.position.y,
				translationZ: jointModel.position.z,
			};

			let translationSliderX = folder.add(jointTranslationGUIObject, 'translationX', lower, upper).onChange(value => {
				jointModel.position.x = value;
			});
			translationSliderX.listen();

			let translationSliderY = folder.add(jointTranslationGUIObject, 'translationY', lower, upper).onChange(value => {
				jointModel.position.y = value;
			});
			translationSliderY.listen();

			let translationSliderZ = folder.add(jointTranslationGUIObject, 'translationZ', lower, upper).onChange(value => {
				jointModel.position.z = value;
			});
			translationSliderZ.listen();

			let jointRotationGUIObject = {
				rotationX: jointModel.rotation.x,
				rotationY: jointModel.rotation.y,
				rotationZ: jointModel.rotation.z,
			};

			let rotationSliderX = folder.add(jointRotationGUIObject, 'rotationX', lower, upper).onChange(value => {
				jointModel.rotation.x = value;
			});
			rotationSliderX.listen();

			let rotationSliderY = folder.add(jointRotationGUIObject, 'rotationY', lower, upper).onChange(value => {
				jointModel.rotation.y = value;
			});
			rotationSliderY.listen();

			let rotationSliderZ = folder.add(jointRotationGUIObject, 'rotationZ', lower, upper).onChange(value => {
				jointModel.rotation.z = value;
			});
			rotationSliderZ.listen();

		}
		else {
			// todo
		}

	});

	robot.constraints.forEach(constraint => {

		if (constraint.type == "triangle-prismatic") {
			
			robotConstraintHandler(robot, robotModel, constraint.name);

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
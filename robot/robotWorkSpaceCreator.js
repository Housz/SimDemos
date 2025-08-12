import * as THREE from 'three';
import { GUI } from 'three/addons/libs/lil-gui.module.min.js';
import { robotConstraintHandler, updateAllConstraints } from './robotConstraintHandler.js'


function robotWorkSpaceCreator(robot, robotModel) {

	let gui = new GUI();
	gui.domElement.style.position = 'absolute';
	gui.domElement.style.left = '0px';
	gui.domElement.style.opacity = '0.8';

	// Make GUI resizable
	gui.domElement.style.resize = 'horizontal';
	gui.domElement.style.overflow = 'auto';

	gui.title("Workspace");


	gui.add({
		getMesh: () => {
			onClick()
		}
	}, 'getMesh').name("Get 'jiegetou' Mesh");


	const head = robotModel.getObjectByName("jiegetou");
	const sphereGeometry = new THREE.SphereGeometry(0.2, 16, 16);
	const sphereMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
	const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
	sphere.position.set(-4.0, 0, 0);
	// robotModel.add(sphere);
	// head.add(sphere);


	let onClick = () => {
		const head = robotModel.getObjectByName("jiegetou");

		// const clonedModel = head.children[1].clone(true);
		const clonedModel = head.clone(true);

		// Get the transformation matrix of the cloned model relative to the robot model
		const clonedModelWorldMatrix = new THREE.Matrix4().copy(clonedModel.matrixWorld);
		const robotModelWorldMatrixInverse = new THREE.Matrix4().copy(robotModel.matrixWorld).invert();
		const relativeMatrix = new THREE.Matrix4().multiplyMatrices(robotModelWorldMatrixInverse, clonedModelWorldMatrix);

		clonedModel.applyMatrix4(relativeMatrix);
		// robotModel.add(clonedModel);


		// Set the cloned model to wireframe mode with red color
		clonedModel.traverse((child) => {
			if (child.isMesh) {
				child.material = new THREE.MeshBasicMaterial({
					color: 0xff0000,
					wireframe: true,
					opacity: 0.5,
					transparent: true
				});
			}
		});



		function addclonedModel(relativeMatrix, curr_j) {

			// const clonedModel = head.clone(true);
			const clonedModel = head.children[0].clone(true);
			clonedModel.applyMatrix4(relativeMatrix);

			function getColorFromX(x, xMin = -0.42, xMax = 0.1) {
				const t = THREE.MathUtils.clamp((x - xMin) / (xMax - xMin), 0, 1);
				const r = 1 - t;
				const g = 0;
				const b = t;
				return new THREE.Color(r, g, b);
			}

			clonedModel.traverse((child) => {
				if (child.isMesh) {
					child.material = new THREE.MeshBasicMaterial({
						color: 0xff0000,
						color: getColorFromX(curr_j),
						wireframe: true,
						opacity: 0.1,
						transparent: true,
						visible: false
					});
				}
			});

			const sphereGeometry = new THREE.SphereGeometry(0.03, 16, 16);
			const sphereMaterial = new THREE.MeshBasicMaterial({ color: getColorFromX(curr_j) });
			const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
			sphere.position.add(new THREE.Vector3(-4.0, 0, 0));

			

			// clonedModel.add(sphere);
			sphere.applyMatrix4(relativeMatrix);
			robotModel.add(sphere);

			robotModel.add(clonedModel);
		}


		// let geo = clonedModel.children[1].geometry;
		// console.log(geo);
		// const edges = new THREE.EdgesGeometry( geo, 10 ); 
		// const line = new THREE.LineSegments(edges, new THREE.LineBasicMaterial( { color: 0xffffff } ) );
		// robotModel.add(line);




		// Horizontal and vertical constraints
		let cst_hori_name = "constraint_base_huizhuanzhongxin1";
		let cst_vert_name = "constraint_huizhuanzhongxin_jiegedianji1";
		const cst_hori = robot.constraints.find(constraint => constraint.name === cst_hori_name);
		const cst_vert = robot.constraints.find(constraint => constraint.name === cst_vert_name);

		cst_hori.length = 2.0;
		// robotConstraintHandler(robot, robotModel, cst_hori_name);

		console.log(cst_hori.limit.lower, cst_hori.limit.upper);

		let lower_h = cst_hori.limit.lower;
		let upper_h = cst_hori.limit.upper;

		let lower_v = cst_vert.limit.lower;
		let upper_v = cst_vert.limit.upper;


		let head_joint_name = "joint_jiegedianji_huajianzhou";
		let head_joint = robot.joints.find(joint => joint.name === head_joint_name);
		let lower_j = head_joint.limit.lower;
		let upper_j = head_joint.limit.upper;
		let jointModel = getChildByName(robotModel, head_joint_name);


		for (let curr_h = lower_h; curr_h <= upper_h; curr_h += 0.05) {

			for (let curr_v = lower_v; curr_v <= upper_v; curr_v += 0.05) {

				for (let curr_j = lower_j; curr_j <= upper_j; curr_j += 0.1) {

					cst_hori.length = curr_h;
					cst_vert.length = curr_v;
					robotConstraintHandler(robot, robotModel, cst_hori_name);
					robotConstraintHandler(robot, robotModel, cst_vert_name);

					jointModel.position.x = curr_j;

					robotModel.updateWorldMatrix(true, true);

					const head = robotModel.getObjectByName("jiegetou");
					const headWorldMatrix = new THREE.Matrix4().copy(head.matrixWorld);
					const relativeMatrix = new THREE.Matrix4().multiplyMatrices(robotModelWorldMatrixInverse, headWorldMatrix);

					addclonedModel(relativeMatrix, curr_j);

				}

			}

		}


	}



	robot.constraints.forEach(constraint => {

		if (constraint.type == "triangle-prismatic") {

			// initialization constraints 
			robotConstraintHandler(robot, robotModel, constraint.name);

			let folder = gui.addFolder(constraint.name);

			let lower = constraint.limit.lower;
			let upper = constraint.limit.upper;

			let constraintGUIObject = {
				length: constraint.length
			};

			let length = folder.add(constraintGUIObject, 'length', lower, upper).onChange(value => {

				constraint.length = value;

				// satisfy constraint on each UI update
				robotConstraintHandler(robot, robotModel, constraint.name);

			});


		}
		else {
			// todo
		}
	});

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


export { robotWorkSpaceCreator };
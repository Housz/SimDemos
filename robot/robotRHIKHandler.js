import * as THREE from 'three';
import { GUI } from 'three/addons/libs/lil-gui.module.min.js';
import { robotConstraintHandler, updateAllConstraints } from './robotConstraintHandler.js'

export function robotRHIKHandler(targetModel, robot, robotModel) {

	let target = targetModel.position;

	// Horizontal and vertical constraints
	let cst_hori_name = "constraint_base_huizhuanzhongxin1";
	let cst_vert_name = "constraint_huizhuanzhongxin_jiegedianji1";
	const cst_hori = robot.constraints.find(constraint => constraint.name === cst_hori_name);
	const cst_vert = robot.constraints.find(constraint => constraint.name === cst_vert_name);

	let lower_h = cst_hori.limit.lower;
	let upper_h = cst_hori.limit.upper;

	let lower_v = cst_vert.limit.lower;
	let upper_v = cst_vert.limit.upper;

	let head_joint_name = "joint_jiegedianji_huajianzhou";
	let head_joint = robot.joints.find(joint => joint.name === head_joint_name);
	let lower_j = head_joint.limit.lower;
	let upper_j = head_joint.limit.upper;

	let jointModel = getChildByName(robotModel, head_joint_name);

	const head = robotModel.getObjectByName("jiegetou");

	/** 目标函数 */
	function loss() {
		const localOffset = new THREE.Vector3(-4.0, 0, 0);
		const headPosition = new THREE.Vector3();
		headPosition.copy(head.position).add(localOffset);
		head.localToWorld(headPosition);
		const difference = new THREE.Vector3();
		difference.subVectors(target, headPosition);
		return difference.length();
	}

	/** 更新函数们 */
	function updateH(length) {
		cst_hori.length = length;
		robotConstraintHandler(robot, robotModel, cst_hori_name);
	}

	function updateV(length) {
		cst_vert.length = length;
		robotConstraintHandler(robot, robotModel, cst_vert_name);
	}

	function updateHead(length) {
		jointModel.position.x = length;
		robotModel.updateWorldMatrix(true, true);
	}

	/**
	 * 通用黄金分割搜索（单峰假设）
	 * @param {number} lower
	 * @param {number} upper
	 * @param {(x:number)=>void} updateFn  给系统施加参数x的回调（内部会调用loss()评估）
	 * @param {number} tolerance
	 * @returns {number} 最优解
	 */
	function goldenSectionSearch(lower, upper, updateFn, tolerance = 1e-2) {
		const phi = (1 + Math.sqrt(5)) / 2; // 黄金比
		let a = lower;
		let b = upper;

		let c = b - (b - a) / phi;
		let d = a + (b - a) / phi;

		// 先各评估一次，后续每轮只新增一次评估，避免重复计算
		updateFn(c);
		let lossC = loss();

		updateFn(d);
		let lossD = loss();

		while (Math.abs(b - a) > tolerance) {
			if (lossC < lossD) {
				// 舍弃 [d, b]
				b = d;
				d = c;
				lossD = lossC;

				c = b - (b - a) / phi;
				updateFn(c);
				lossC = loss();
			} else {
				// 舍弃 [a, c]
				a = c;
				c = d;
				lossC = lossD;

				d = a + (b - a) / phi;
				updateFn(d);
				lossD = loss();
			}
		}

		const optimal = (a + b) / 2;
		updateFn(optimal); // 将系统定格在最优点
		return optimal;
	}

	const optimalH = goldenSectionSearch(lower_h, upper_h, updateH);
	// console.log("Optimal H length:", optimalH);

	const optimalV = goldenSectionSearch(lower_v, upper_v, updateV);
	// console.log("Optimal V length:", optimalV);

	const optimalHead = goldenSectionSearch(lower_j, upper_j, updateHead);
	// console.log("Optimal Head length:", optimalHead);

	if (loss() > 1e-1) {
		targetModel.material.color.set(0xff0000); 
	} else {
		targetModel.material.color.set(0x00ff00);
	}
}


function getChildByName(object, childName) {
	let child = null;
	object.traverse(currChild => {
		if (currChild.name == childName) {
			if (child == null) {
				child = currChild;
			} else {
				console.error("存在重复节点：" + childName);
			}
		}
	});
	return child;
}

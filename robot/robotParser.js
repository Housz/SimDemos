class _Robot {
	constructor(name, links, joints, constraints) {
		this.name = name;
		this.links = links;
		this.joints = joints;
		this.constraints = constraints;

		this.linkMap = new Map();
		this.jointMap = new Map();
		this.constraintMap = new Map();
	}
}

class _Link {
	constructor(name, origin_translation, origin_orientation, visual) {
		this.name = name;
		this.origin_translation = new _Translation(origin_translation[0], origin_translation[1], origin_translation[2]);
		this.origin_orientation = new _Orientation(origin_orientation[0], origin_orientation[1], origin_orientation[2]);
		this.visual = new _Visual(visual.origin_translation, visual.origin_orientation, visual.geometry, visual.material);
	}
}

class _Joint {
	constructor(
		name,
		type,
		parent,
		child,
		origin_translation,
		origin_orientation,
		axis,
		limit
	) {
		this.name = name;
		this.type = type;
		this.parent = parent;
		this.child = child;
		this.origin_translation = new _Translation(origin_translation[0], origin_translation[1], origin_translation[2]);
		this.origin_orientation = new _Orientation(origin_orientation[0], origin_orientation[1], origin_orientation[2]);
		this.axis = new _Axis(axis[0], axis[1], axis[2]);
		this.limit = new _Limit(limit.lower, limit.upper);
	}
}

class _Constraint {
	constructor(
		name,
		type,
		jointL,
		jointA,
		jointB,
		jointL_base_angle,
		length,
		limit
	) {
		this.name = name;
		this.type = type;
		this.jointL = jointL;
		this.jointA = jointA;
		this.jointB = jointB;
		this.jointL_base_angle = jointL_base_angle;
		this.length = length,
			this.limit = new _Limit(limit.lower, limit.upper);
	}
}

class _Limit {
	constructor(lower, upper) {
		this.lower = lower;
		this.upper = upper;
	}
}

class _Axis {
	constructor(x = 0, y = 0, z = 0) {
		this.x = x;
		this.y = y;
		this.z = z;
	}
}

class _Translation {
	constructor(x = 0, y = 0, z = 0) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	length() {
		return Math.sqrt(this.x*this.x + this.y*this.y + this.z*this.z);
	}
}

class _Orientation {
	constructor(x = 0, y = 0, z = 0) {
		this.x = x;
		this.y = y;
		this.z = z;
	}
}

class _Visual {
	constructor(origin_translation, origin_orientation, geometry, material) {
		this.origin_translation = origin_translation;
		this.origin_orientation = origin_orientation;
		this.geometry = geometry;
		this.material = material;
	}
}

function robotParser(robotJson) {

	// construct robot
	let robot = new _Robot(robotJson.name, [], [], []);

	robotJson.links.forEach(linkData => {

		if (linkData.name == "world" || linkData.name == "base_link") {
			return;
		}

		let link = new _Link(
			linkData.name,
			linkData.origin_translation,
			linkData.origin_orientation,
			linkData.visual
		);

		robot.links.push(link);

		robot.linkMap.set(link.name, link);

	});

	robotJson.joints.forEach(jointData => {

		if (jointData.name == "world_joint") {
			return;
		}

		let joint = new _Joint(
			jointData.name,
			jointData.type,
			jointData.parent,
			jointData.child,
			jointData.origin_translation,
			jointData.origin_orientation,
			jointData.axis || [0, 0, 0],
			jointData.limit || {"lower": 0,"upper": 1.57}
		);

		robot.joints.push(joint);

		robot.jointMap.set(joint.name, joint);
	});


	robotJson.constraints.forEach(constraintData => {

		let constraint = new _Constraint(
			constraintData.name,
			constraintData.type,
			constraintData.jointL,
			constraintData.jointA,
			constraintData.jointB,
			constraintData.jointL_base_angle,
			constraintData.length,
			constraintData.limit
		);

		robot.constraints.push(constraint);

		robot.constraintMap.set(constraint.name, constraint);
	});


	return robot;

}

export { robotParser };
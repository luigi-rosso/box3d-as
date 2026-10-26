// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Joints: a chain of capsules on revolute hinges that swings down and hangs, and an elevator platform on a
// motorized prismatic joint that rises to its limit.

import {
	B3_PI,
	b3BodyId,
	b3Body_GetPosition,
	b3Capsule,
	b3CreateBody,
	b3CreateCapsuleShape,
	b3CreateHullShape,
	b3CreatePrismaticJoint,
	b3CreateRevoluteJoint,
	b3CreateWorld,
	b3DefaultBodyDef,
	b3DefaultPrismaticJointDef,
	b3DefaultRevoluteJointDef,
	b3DefaultShapeDef,
	b3DefaultWorldDef,
	b3DestroyWorld,
	b3MakeBoxHull,
	b3MakeQuatFromAxisAngle,
	b3PrismaticJoint_GetTranslation,
	b3Quat_identity,
	b3Transform,
	b3Vec3,
	b3Vec3_axisZ,
	b3World_Step,
	b3_dynamicBody,
} from "../lib/box3d";

export const LINK_COUNT: i32 = 10;
export const LINK_LENGTH: f32 = 1.0;
export const ANCHOR_HEIGHT: f32 = 12.0;

export class JointsResult {
	// Where the chain's last link ends up, the chain hanging from (0, ANCHOR_HEIGHT, 0)
	chainEnd: b3Vec3 = b3Vec3();
	// The elevator's travel along its axis
	elevatorTranslation: f32 = 0;
}

export function joints(): JointsResult {
	let worldId = b3CreateWorld(b3DefaultWorldDef());
	let groundId = b3CreateBody(worldId, b3DefaultBodyDef());

	// The chain starts horizontal along +x. A revolute joint turns about the z axis of its frames, and each link's
	// frames sit at its two ends.
	let half: f32 = 0.5 * LINK_LENGTH;
	let link = b3Capsule(b3Vec3(-half, 0.0, 0.0), b3Vec3(half, 0.0, 0.0), 0.1);
	let bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	// Damping lets the swing settle
	bodyDef.linearDamping = 0.5;
	let shapeDef = b3DefaultShapeDef();

	let jointDef = b3DefaultRevoluteJointDef();
	jointDef.base.localFrameA = b3Transform(b3Vec3(0.0, ANCHOR_HEIGHT, 0.0), b3Quat_identity);
	jointDef.base.localFrameB = b3Transform(b3Vec3(-half, 0.0, 0.0), b3Quat_identity);

	let parentId = groundId;
	let lastId = groundId;
	for (let i = 0; i < LINK_COUNT; ++i) {
		bodyDef.position = b3Vec3((<f32>i + 0.5) * LINK_LENGTH, ANCHOR_HEIGHT, 0.0);
		let linkId = b3CreateBody(worldId, bodyDef);
		b3CreateCapsuleShape(linkId, shapeDef, link);

		jointDef.base.bodyIdA = parentId;
		jointDef.base.bodyIdB = linkId;
		b3CreateRevoluteJoint(worldId, jointDef);

		jointDef.base.localFrameA = b3Transform(b3Vec3(half, 0.0, 0.0), b3Quat_identity);
		parentId = linkId;
		lastId = linkId;
	}

	// The elevator, beside the chain's swing: a prismatic joint slides along the x axis of frame A, turned here to
	// point up
	let up = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 0.5 * B3_PI);
	bodyDef.position = b3Vec3(0.0, 1.0, 5.0);
	bodyDef.linearDamping = 0.0;
	let platformId: b3BodyId = b3CreateBody(worldId, bodyDef);
	let platform = b3MakeBoxHull(1.0, 0.1, 1.0);
	b3CreateHullShape(platformId, shapeDef, platform.base);

	let prismaticDef = b3DefaultPrismaticJointDef();
	prismaticDef.base.bodyIdA = groundId;
	prismaticDef.base.bodyIdB = platformId;
	prismaticDef.base.localFrameA = b3Transform(b3Vec3(0.0, 1.0, 5.0), up);
	prismaticDef.base.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), up);
	prismaticDef.enableLimit = true;
	prismaticDef.lowerTranslation = 0.0;
	prismaticDef.upperTranslation = 3.0;
	prismaticDef.enableMotor = true;
	prismaticDef.motorSpeed = 1.0;
	// Newtons, more than the weight of the 800 kg platform
	prismaticDef.maxMotorForce = 20000.0;
	let elevatorId = b3CreatePrismaticJoint(worldId, prismaticDef);

	let timeStep: f32 = <f32>(1.0 / 60.0);
	for (let i = 0; i < 600; ++i) {
		b3World_Step(worldId, timeStep, 4);
	}

	let result = new JointsResult();
	result.chainEnd = b3Body_GetPosition(lastId);
	result.elevatorTranslation = b3PrismaticJoint_GetTranslation(elevatorId);
	b3DestroyWorld(worldId);
	return result;
}

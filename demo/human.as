// A ragdoll: shared/human.c from the Box3D samples, as verify/scenes/scenes.as ports it.

import { Color } from "rive/color";
import { PhysicsScene } from "./physics";
import {
    B3_DEG_TO_RAD,
    b3BodyId,
    b3Capsule,
    b3CreateCapsuleShape,
    b3CreateFilterJoint,
    b3CreateRevoluteJoint,
    b3CreateSphericalJoint,
    b3DefaultBodyDef,
    b3DefaultFilterJointDef,
    b3DefaultRevoluteJointDef,
    b3DefaultShapeDef,
    b3DefaultSphericalJointDef,
    b3JointId,
    b3JointType,
    b3NormalizeQuat,
    b3OffsetPos,
    b3Pos,
    b3Quat,
    b3Transform,
    b3Vec2,
    b3Vec3,
    b3_dynamicBody,
    b3_nullBodyId,
    b3_nullJointId,
    b3_revoluteJoint,
    b3_sphericalJoint,
} from "../lib/box3d";

// shared/human.h
const bone_pelvis = 0;
const bone_spine_01 = 1;
const bone_spine_03 = 2;
const bone_neck = 3;
const bone_thigh_l = 4;
const bone_calf_l = 5;
const bone_thigh_r = 6;
const bone_calf_r = 7;
const bone_upper_arm_l = 8;
const bone_lower_arm_l = 9;
const bone_upper_arm_r = 10;
const bone_lower_arm_r = 11;
const bone_count = 12;

class SceneBone {
    bodyId: b3BodyId = b3BodyId();
    jointId: b3JointId = b3JointId();
    localFrameA: b3Transform = b3Transform();
    localFrameB: b3Transform = b3Transform();
    referenceFrame: b3Transform = b3Transform();
    jointType: b3JointType = 0;
    swingLimit: f32 = 0;
    twistLimit: b3Vec2 = b3Vec2();
    jointFriction: f32 = 0;
    parentIndex: i32 = 0;
}

// shared/human.c CreateHuman without names and user data, one ink per ragdoll
export function createHuman(
    scene: PhysicsScene,
    position: b3Pos,
    frictionTorque: f32,
    hertz: f32,
    dampingRatio: f32,
    groupIndex: i32,
    ink: Color,
): void {
    let bones = new Array<SceneBone>(0);
    for (let i = 0; i < bone_count; ++i) {
        bones.push(new SceneBone());
    }

    for (let i = 0; i < bone_count; ++i) {
        bones[i].bodyId = b3_nullBodyId;
        bones[i].jointId = b3_nullJointId;
        bones[i].jointFriction = 1.0;
        bones[i].parentIndex = -1;
    }

    let bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;

    let shapeDef = b3DefaultShapeDef();
    let defaultDensity: f32 = shapeDef.density;

    {
        let bone = bones[bone_pelvis];
        bone.parentIndex = -1;

        bone.referenceFrame = b3Transform(b3Vec3(0.000000, 0.996219, -0.023868), b3Quat(b3Vec3(1.000000, 0.000000, 0.000000), 0.000000));
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = scene.addBody(bodyDef, ink);

        shapeDef.filter.groupIndex = 0;

        shapeDef.baseMaterial.friction = 0.6;
        shapeDef.baseMaterial.rollingResistance = 0.1;
        let capsule = b3Capsule(b3Vec3(0.040000, 0.000001, 0.000000), b3Vec3(-0.040000, -0.000001, 0.000000), 0.150000);
        b3CreateCapsuleShape(bone.bodyId, shapeDef, capsule);
    }

    {
        let bone = bones[bone_spine_01];
        bone.parentIndex = bone_pelvis;

        bone.referenceFrame = b3Transform(b3Vec3(0.000000, 1.017288, -0.024882), b3Quat(b3Vec3(1.000000, 0.000000, 0.000000), 0.000000));
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = scene.addBody(bodyDef, ink);

        shapeDef.filter.groupIndex = -groupIndex;

        shapeDef.baseMaterial.friction = 0.5;
        shapeDef.baseMaterial.rollingResistance = 0.1;
        let capsule = b3Capsule(b3Vec3(0.029876, -0.146581, 0.006260), b3Vec3(-0.029876, -0.146574, 0.006260), 0.145663);
        b3CreateCapsuleShape(bone.bodyId, shapeDef, capsule);

        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform(b3Vec3(0.000000, -0.021069, 0.001014), b3Quat(b3Vec3(-0.642737, 0.000000, 0.000000), -0.766087));
        bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(-0.707107, 0.000000, 0.000000), -0.707107));
        bone.swingLimit = 35.0 * B3_DEG_TO_RAD;
        bone.twistLimit = b3Vec2(-17.5 * B3_DEG_TO_RAD, 17.5 * B3_DEG_TO_RAD);
    }

    {
        let bone = bones[bone_spine_03];
        bone.parentIndex = bone_spine_01;

        bone.referenceFrame = b3Transform(b3Vec3(0.000000, 1.267766, -0.022320), b3Quat(b3Vec3(1.000000, 0.000000, 0.000000), 0.000000));
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = scene.addBody(bodyDef, ink);

        shapeDef.filter.groupIndex = 0;

        shapeDef.baseMaterial.friction = 0.5;
        shapeDef.baseMaterial.rollingResistance = 0.1;
        let capsule = b3Capsule(b3Vec3(0.063996, -0.117434, -0.040199), b3Vec3(-0.063996, -0.117432, -0.040199), 0.165004);
        b3CreateCapsuleShape(bone.bodyId, shapeDef, capsule);

        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform(b3Vec3(0.000000, -0.250478, -0.002562), b3Quat(b3Vec3(-0.642766, 0.000000, 0.000000), -0.766063));
        bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(-0.707107, 0.000000, 0.000000), -0.707107));
        bone.swingLimit = 35.0 * B3_DEG_TO_RAD;
        bone.twistLimit = b3Vec2(-17.5 * B3_DEG_TO_RAD, 17.5 * B3_DEG_TO_RAD);
    }

    {
        let bone = bones[bone_neck];
        bone.parentIndex = bone_spine_03;

        bone.referenceFrame = b3Transform(b3Vec3(0.000000, 1.498783, 0.024462), b3Quat(b3Vec3(0.987879, 0.000000, 0.000000), 0.155228));
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = scene.addBody(bodyDef, ink);

        shapeDef.filter.groupIndex = 0;

        shapeDef.baseMaterial.friction = 0.2;
        shapeDef.baseMaterial.rollingResistance = 0.05;
        let head = b3Capsule(b3Vec3(0.000000, -0.212340, 0.000000), b3Vec3(0.000000, -0.087340, 0.000000), 0.110000);
        b3CreateCapsuleShape(bone.bodyId, shapeDef, head);

        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform(b3Vec3(0.000000, -0.231017, -0.046781), b3Quat(b3Vec3(-0.516157, -0.000002, 0.000003), -0.856494));
        bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(-0.707108, -0.000002, 0.000002), -0.707106));
        bone.swingLimit = 30.0 * B3_DEG_TO_RAD;
        bone.twistLimit = b3Vec2(-17.5 * B3_DEG_TO_RAD, 17.5 * B3_DEG_TO_RAD);
    }

    {
        let bone = bones[bone_thigh_l];
        bone.parentIndex = bone_pelvis;

        bone.referenceFrame = b3Transform(b3Vec3(0.092175, 0.971562, -0.011177), b3Quat(b3Vec3(0.471412, 0.529510, -0.499888), -0.497495));
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = scene.addBody(bodyDef, ink);

        shapeDef.filter.groupIndex = -groupIndex;

        shapeDef.baseMaterial.friction = 0.6;
        shapeDef.baseMaterial.rollingResistance = 0.1;
        let capsule = b3Capsule(b3Vec3(-0.047269, 0.000001, 0.000000), b3Vec3(-0.379769, 0.000005, 0.000000), 0.091539);
        b3CreateCapsuleShape(bone.bodyId, shapeDef, capsule);

        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform(b3Vec3(0.092175, 0.024656, -0.012691), b3Quat(b3Vec3(-0.460186, -0.365368, -0.637662), 0.498118));
        bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(-0.499997, -0.500003, 0.500002), 0.499998));
        bone.swingLimit = 30.0 * B3_DEG_TO_RAD;
        bone.twistLimit = b3Vec2(-10.0 * B3_DEG_TO_RAD, 10.0 * B3_DEG_TO_RAD);
    }

    {
        let bone = bones[bone_calf_l];
        bone.parentIndex = bone_thigh_l;

        bone.referenceFrame = b3Transform(b3Vec3(0.118720, 0.534541, -0.035362), b3Quat(b3Vec3(0.521188, 0.480597, -0.546380), -0.445935));
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = scene.addBody(bodyDef, ink);

        shapeDef.filter.groupIndex = 0;

        shapeDef.baseMaterial.friction = 0.1;
        shapeDef.baseMaterial.rollingResistance = 0.0;
        let shin = b3Capsule(b3Vec3(-0.445000, 0.000001, 0.000000), b3Vec3(0.005000, -0.000001, 0.000000), 0.080000);
        b3CreateCapsuleShape(bone.bodyId, shapeDef, shin);

        shapeDef.density = 0.5 * defaultDensity;
        let foot = b3Capsule(b3Vec3(-0.456371, 0.129321, 0.000013), b3Vec3(-0.359977, 0.014394, 0.000013), 0.070000);
        b3CreateCapsuleShape(bone.bodyId, shapeDef, foot);
        shapeDef.density = defaultDensity;

        bone.jointType = b3_revoluteJoint;
        bone.localFrameA = b3Transform(b3Vec3(-0.438494, -0.000175, 0.000000), b3Quat(b3Vec3(0.344866, -0.938652, 0.000002), -0.000008));
        bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(-0.008182, -0.999967, 0.000001), -0.000007));
        bone.twistLimit = b3Vec2(-30.0 * B3_DEG_TO_RAD, 30.0 * B3_DEG_TO_RAD);
    }

    {
        let bone = bones[bone_thigh_r];
        bone.parentIndex = bone_pelvis;

        bone.referenceFrame = b3Transform(b3Vec3(-0.092175, 0.971562, -0.011177), b3Quat(b3Vec3(-0.497495, 0.499888, 0.529510), -0.471412));
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = scene.addBody(bodyDef, ink);

        shapeDef.filter.groupIndex = -groupIndex;

        shapeDef.baseMaterial.friction = 0.6;
        shapeDef.baseMaterial.rollingResistance = 0.1;
        let capsule = b3Capsule(b3Vec3(0.047269, -0.000001, 0.000000), b3Vec3(0.379769, -0.000005, 0.000000), 0.092587);
        b3CreateCapsuleShape(bone.bodyId, shapeDef, capsule);

        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform(b3Vec3(-0.092175, 0.024656, -0.012691), b3Quat(b3Vec3(0.380666, 0.491846, 0.488644), -0.611889));
        bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(-0.500002, 0.499998, -0.499998), 0.500002));
        bone.swingLimit = 30.0 * B3_DEG_TO_RAD;
        bone.twistLimit = b3Vec2(-10.0 * B3_DEG_TO_RAD, 10.0 * B3_DEG_TO_RAD);
    }

    {
        let bone = bones[bone_calf_r];
        bone.parentIndex = bone_thigh_r;

        bone.referenceFrame = b3Transform(b3Vec3(-0.118720, 0.534541, -0.035362), b3Quat(b3Vec3(-0.445935, 0.546380, 0.480597), -0.521188));
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = scene.addBody(bodyDef, ink);

        shapeDef.filter.groupIndex = 0;

        shapeDef.baseMaterial.friction = 0.1;
        shapeDef.baseMaterial.rollingResistance = 0.0;
        let shin = b3Capsule(b3Vec3(0.445000, -0.000002, 0.000003), b3Vec3(-0.005000, 0.000000, 0.000003), 0.080000);
        b3CreateCapsuleShape(bone.bodyId, shapeDef, shin);

        shapeDef.density = 0.5 * defaultDensity;
        let foot = b3Capsule(b3Vec3(0.369113, -0.006830, 0.000017), b3Vec3(0.465507, -0.121757, 0.000017), 0.070000);
        b3CreateCapsuleShape(bone.bodyId, shapeDef, foot);
        shapeDef.density = defaultDensity;

        bone.jointType = b3_revoluteJoint;
        bone.localFrameA = b3Transform(b3Vec3(0.438494, 0.000175, 0.000000), b3Quat(b3Vec3(0.000007, 0.000005, -0.344860), -0.938654));
        bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(0.000006, 0.000005, 0.008188), -0.999966));
        bone.twistLimit = b3Vec2(-30.0 * B3_DEG_TO_RAD, 30.0 * B3_DEG_TO_RAD);
    }

    {
        let bone = bones[bone_upper_arm_l];
        bone.parentIndex = bone_spine_03;

        bone.referenceFrame = b3Transform(b3Vec3(0.185817, 1.443630, 0.031306), b3Quat(b3Vec3(0.560220, -0.307179, 0.366233), -0.676512));
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = scene.addBody(bodyDef, ink);

        shapeDef.filter.groupIndex = 0;

        shapeDef.baseMaterial.friction = 0.6;
        shapeDef.baseMaterial.rollingResistance = 0.1;
        let capsule = b3Capsule(b3Vec3(0.296038, 0.004386, -0.000794), b3Vec3(-0.016086, 0.004384, -0.016127), 0.065000);
        b3CreateCapsuleShape(bone.bodyId, shapeDef, capsule);

        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform(b3Vec3(0.185817, -0.175865, -0.053625), b3Quat(b3Vec3(-0.559466, -0.213784, -0.670199), 0.438323));
        bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(-0.500953, -0.502617, -0.509318), -0.486844));
        bone.swingLimit = 45.0 * B3_DEG_TO_RAD;
        bone.twistLimit = b3Vec2(-20.0 * B3_DEG_TO_RAD, 20.0 * B3_DEG_TO_RAD);
        bone.jointFriction = 0.8;
    }

    {
        let bone = bones[bone_lower_arm_l];
        bone.parentIndex = bone_upper_arm_l;

        bone.referenceFrame = b3Transform(b3Vec3(0.347683, 1.193455, 0.029378), b3Quat(b3Vec3(0.406736, -0.493012, 0.089135), -0.763911));
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = scene.addBody(bodyDef, ink);

        shapeDef.filter.groupIndex = 0;

        shapeDef.baseMaterial.friction = 0.3;
        shapeDef.baseMaterial.rollingResistance = 0.1;
        let capsule = b3Capsule(b3Vec3(0.312329, -0.008256, -0.016454), b3Vec3(0.033153, 0.008259, -0.002741), 0.060000);
        b3CreateCapsuleShape(bone.bodyId, shapeDef, capsule);

        bone.jointType = b3_revoluteJoint;
        bone.localFrameA = b3Transform(b3Vec3(0.297979, 0.000361, 0.000000), b3Quat(b3Vec3(0.916726, 0.399352, 0.000839), -0.011456));
        bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(0.999922, -0.003857, 0.004659), -0.010908));
        bone.twistLimit = b3Vec2(-35.0 * B3_DEG_TO_RAD, 35.0 * B3_DEG_TO_RAD);
        bone.jointFriction = 0.06;
    }

    {
        let bone = bones[bone_upper_arm_r];
        bone.parentIndex = bone_spine_03;

        bone.referenceFrame = b3Transform(b3Vec3(-0.185817, 1.443630, 0.031306), b3Quat(b3Vec3(0.676512, 0.366233, 0.307179), 0.560220));
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = scene.addBody(bodyDef, ink);

        shapeDef.filter.groupIndex = 0;

        shapeDef.baseMaterial.friction = 0.6;
        shapeDef.baseMaterial.rollingResistance = 0.1;
        let capsule = b3Capsule(b3Vec3(-0.300237, -0.000001, -0.000319), b3Vec3(0.011887, 0.000001, 0.015013), 0.065000);
        b3CreateCapsuleShape(bone.bodyId, shapeDef, capsule);

        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform(b3Vec3(-0.185817, -0.175865, -0.053625), b3Quat(b3Vec3(0.213793, 0.559449, 0.438334), -0.670204));
        bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(-0.509319, 0.486843, 0.500954), -0.502617));
        bone.swingLimit = 45.0 * B3_DEG_TO_RAD;
        bone.twistLimit = b3Vec2(-20.0 * B3_DEG_TO_RAD, 20.0 * B3_DEG_TO_RAD);
        bone.jointFriction = 0.8;
    }

    {
        let bone = bones[bone_lower_arm_r];
        bone.parentIndex = bone_upper_arm_r;

        bone.referenceFrame = b3Transform(b3Vec3(-0.347683, 1.193455, 0.029378), b3Quat(b3Vec3(0.763911, 0.089135, 0.493012), 0.406736));
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = scene.addBody(bodyDef, ink);

        shapeDef.filter.groupIndex = 0;

        shapeDef.baseMaterial.friction = 0.3;
        shapeDef.baseMaterial.rollingResistance = 0.1;
        let capsule = b3Capsule(b3Vec3(-0.312649, -0.000003, 0.016471), b3Vec3(-0.032986, 0.000000, 0.002732), 0.060000);
        b3CreateCapsuleShape(bone.bodyId, shapeDef, capsule);

        bone.jointType = b3_revoluteJoint;
        bone.localFrameA = b3Transform(b3Vec3(-0.297979, -0.000361, 0.000000), b3Quat(b3Vec3(-0.405444, 0.914059, 0.010541), 0.000852));
        bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(-0.002794, 0.999936, 0.010042), 0.004364));
        bone.twistLimit = b3Vec2(-35.0 * B3_DEG_TO_RAD, 35.0 * B3_DEG_TO_RAD);
        bone.jointFriction = 0.06;
    }

    for (let i = 1; i < bone_count; ++i) {
        let bone = bones[i];
        let parent = bones[bone.parentIndex];

        let bodyIdA = parent.bodyId;
        let bodyIdB = bone.bodyId;

        bone.localFrameA.q = b3NormalizeQuat(bone.localFrameA.q);
        bone.localFrameB.q = b3NormalizeQuat(bone.localFrameB.q);

        if (bone.jointType == b3_revoluteJoint) {
            let jointDef = b3DefaultRevoluteJointDef();
            jointDef.base.bodyIdA = bodyIdA;
            jointDef.base.bodyIdB = bodyIdB;
            jointDef.base.localFrameA = bone.localFrameA;
            jointDef.base.localFrameB = bone.localFrameB;
            jointDef.enableLimit = true;
            jointDef.lowerAngle = bone.twistLimit.x;
            jointDef.upperAngle = bone.twistLimit.y;
            jointDef.enableSpring = hertz > 0.0;
            jointDef.hertz = hertz;
            jointDef.dampingRatio = dampingRatio;
            jointDef.enableMotor = true;
            jointDef.maxMotorTorque = bone.jointFriction * frictionTorque;
            bone.jointId = scene.addJoint(b3CreateRevoluteJoint(scene.worldId, jointDef));
        } else if (bone.jointType == b3_sphericalJoint) {
            let jointDef = b3DefaultSphericalJointDef();
            jointDef.base.bodyIdA = bodyIdA;
            jointDef.base.bodyIdB = bodyIdB;
            jointDef.base.localFrameA = bone.localFrameA;
            jointDef.base.localFrameB = bone.localFrameB;
            jointDef.enableConeLimit = true;
            jointDef.coneAngle = bone.swingLimit;
            jointDef.enableTwistLimit = true;
            jointDef.lowerTwistAngle = bone.twistLimit.x;
            jointDef.upperTwistAngle = bone.twistLimit.y;
            jointDef.enableSpring = hertz > 0.0;
            jointDef.hertz = hertz;
            jointDef.dampingRatio = dampingRatio;
            jointDef.enableMotor = true;
            jointDef.maxMotorTorque = bone.jointFriction * frictionTorque;
            bone.jointId = scene.addJoint(b3CreateSphericalJoint(scene.worldId, jointDef));
        }
    }

    // Disable some collisions
    let filterDef = b3DefaultFilterJointDef();
    filterDef.base.bodyIdA = bones[bone_thigh_l].bodyId;
    filterDef.base.bodyIdB = bones[bone_thigh_r].bodyId;
    b3CreateFilterJoint(scene.worldId, filterDef);
    filterDef.base.bodyIdA = bones[bone_neck].bodyId;
    filterDef.base.bodyIdB = bones[bone_upper_arm_l].bodyId;
    b3CreateFilterJoint(scene.worldId, filterDef);
    filterDef.base.bodyIdA = bones[bone_neck].bodyId;
    filterDef.base.bodyIdB = bones[bone_upper_arm_r].bodyId;
    b3CreateFilterJoint(scene.worldId, filterDef);
}

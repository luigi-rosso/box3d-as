// samples/mover.cpp CharacterMover: a kinematic capsule moved by queries, not
// a body. Collide gathers planes, b3SolvePlanes finds a delta that respects
// them, CastMover sweeps it, and dynamic bodies in the way take an impulse.

import { FLT_MAX } from "../lib/core";
import {
    B3_PI,
    b3Add,
    b3BodyType,
    b3Body_ApplyLinearImpulse,
    b3Body_GetAngularVelocity,
    b3Body_GetInverseMass,
    b3Body_GetLinearVelocity,
    b3Body_GetType,
    b3Body_GetWorldCenter,
    b3Body_GetWorldInverseRotationalInertia,
    b3Capsule,
    b3ClipVector,
    b3CollisionPlane,
    b3Cross,
    b3DefaultQueryFilter,
    b3Dot,
    b3GetLengthAndNormalize,
    b3Length,
    b3LengthSquared,
    b3MaxFloat,
    b3MulMV,
    b3MulSV,
    b3MulSub,
    b3Neg,
    b3OffsetPos,
    b3PlaneResult,
    b3Pos,
    b3Quat_identity,
    b3ShapeId,
    b3Shape_GetBody,
    b3SolvePlanes,
    b3Sub,
    b3SubPos,
    b3Transform,
    b3TransformWorldPoint,
    b3Vec2,
    b3Vec3,
    b3Vec3_axisY,
    b3WorldId,
    b3WorldTransform,
    b3World_CastMover,
    b3World_CastRayClosest,
    b3World_CollideMover,
    b3_dynamicBody,
} from "../lib/box3d";

export const MOVER_PLANE_CAPACITY: i32 = 8;
const JUMP_SPEED: f32 = 5.0;
const MAX_SPEED: f32 = 6.0;
const MIN_SPEED: f32 = 0.01;
const STOP_SPEED: f32 = 1.0;
const ACCELERATE: f32 = 30.0;
const FRICTION: f32 = 4.0;
const GRAVITY: f32 = 15.0;

export class CharacterMover {
    transform: b3WorldTransform = b3Transform();
    velocity: b3Vec3 = b3Vec3();
    capsule: b3Capsule = b3Capsule(b3Vec3(0.0, -0.5, 0.0), b3Vec3(0.0, 0.5, 0.0), 0.3);
    planes: StaticArray<b3CollisionPlane> = new StaticArray<b3CollisionPlane>(MOVER_PLANE_CAPACITY);
    planePoints: StaticArray<b3Pos> = new StaticArray<b3Pos>(MOVER_PLANE_CAPACITY);
    planeShapes: StaticArray<b3ShapeId> = new StaticArray<b3ShapeId>(MOVER_PLANE_CAPACITY);
    planeCount: i32 = 0;
    totalIterations: i32 = 0;
    pogoVelocity: f32 = 0;
    onGround: bool = false;
    sprint: bool = false;

    // The last pogo ray, for drawing.
    rayOrigin: b3Pos = b3Vec3();
    rayEnd: b3Pos = b3Vec3();
    rayHit: bool = false;

    constructor(position: b3Pos) {
        this.transform.p = position;
        this.transform.q = b3Quat_identity;
    }

    jump(): void {
        if (this.onGround) {
            this.velocity.y = JUMP_SPEED;
            this.onGround = false;
        }
    }

    // CharacterMover::SolveMove; throttle x is forward, y is right.
    solveMove(worldId: b3WorldId, timeStep: f32, forward: b3Vec3, right: b3Vec3, throttle: b3Vec2,
        clipVelocity: bool): void {
        // Friction
        let speed: f32 = b3Length(this.velocity);
        if (speed < MIN_SPEED) {
            this.velocity.x = 0.0;
            this.velocity.z = 0.0;
        } else {
            // Linear damping above stopSpeed and fixed reduction below stopSpeed
            let control: f32 = speed < STOP_SPEED ? STOP_SPEED : speed;

            // friction has units of 1/time
            let drop: f32 = control * FRICTION * timeStep;
            let newSpeed: f32 = b3MaxFloat(0.0, speed - drop);
            let ratio: f32 = newSpeed / speed;
            this.velocity.x *= ratio;
            this.velocity.z *= ratio;
        }

        let maxSpeed: f32 = this.sprint ? 1.5 * MAX_SPEED : MAX_SPEED;

        let desiredVelocity = b3Add(b3MulSV(maxSpeed * throttle.x, forward), b3MulSV(maxSpeed * throttle.y, right));
        let desiredSpeedOut: FixedArray<f32, 1>;
        desiredSpeedOut[0] = 0.0;
        let desiredDirection = b3GetLengthAndNormalize(desiredSpeedOut[0], desiredVelocity);
        let desiredSpeed: f32 = desiredSpeedOut[0];

        if (desiredSpeed > maxSpeed) {
            desiredSpeed = maxSpeed;
        }

        if (this.onGround) {
            this.velocity.y = 0.0;
        }

        // Accelerate
        let currentSpeed: f32 = b3Dot(this.velocity, desiredDirection);
        let addSpeed: f32 = desiredSpeed - currentSpeed;
        if (addSpeed > 0.0) {
            let accelSpeed: f32 = ACCELERATE * maxSpeed * timeStep;
            if (accelSpeed > addSpeed) {
                accelSpeed = addSpeed;
            }
            this.velocity = b3Add(this.velocity, b3MulSV(accelSpeed, desiredDirection));
        }

        this.velocity.y -= GRAVITY * timeStep;

        let pogoRestLength: f32 = 3.0 * this.capsule.radius;
        let rayLength: f32 = pogoRestLength + this.capsule.radius;
        let rayOrigin = b3TransformWorldPoint(this.transform, this.capsule.center1);
        let rayTranslation = b3MulSV(-rayLength, b3Vec3_axisY);
        let skipTeamFilter = b3DefaultQueryFilter();
        skipTeamFilter.categoryBits = 1;
        skipTeamFilter.maskBits = 0xfffffffd;
        let rayResult = b3World_CastRayClosest(worldId, rayOrigin, rayTranslation, skipTeamFilter);
        this.rayOrigin = rayOrigin;
        this.rayEnd = rayResult.hit ? rayResult.point : b3OffsetPos(rayOrigin, rayTranslation);

        // After gravity was applied, disable pogo when still moving up.
        // Avoids getting pulled back to the ground when jumping.
        let suppressPogo = this.velocity.y > 0.0;

        if (rayResult.hit == false || suppressPogo) {
            this.onGround = false;
            this.pogoVelocity = 0.0;
            this.rayHit = false;
        } else {
            this.onGround = true;
            this.rayHit = true;
            let pogoCurrentLength: f32 = rayResult.fraction * rayLength;

            let zeta: f32 = 0.7;
            let hertz: f32 = 4.0;
            let omega: f32 = 2.0 * B3_PI * hertz;
            let omegaH: f32 = omega * timeStep;

            this.pogoVelocity = (this.pogoVelocity - omega * omegaH * (pogoCurrentLength - pogoRestLength)) /
                (1.0 + 2.0 * zeta * omegaH + omegaH * omegaH);
        }

        let startPosition = this.transform.p;
        let target = b3Add(b3Add(this.transform.p, b3MulSV(timeStep, this.velocity)),
            b3MulSV(timeStep * this.pogoVelocity, b3Vec3_axisY));

        // Want the mover to collide with allies
        let moverFilter = b3DefaultQueryFilter();
        moverFilter.categoryBits = 1;
        moverFilter.maskBits = 0xffffffff;
        moverFilter.id = 1;

        // The cast should ignore allies
        let castFilter = b3DefaultQueryFilter();
        castFilter.categoryBits = 1;
        castFilter.maskBits = 0xfffffffd;
        castFilter.id = 1;

        let self = this;
        let planeResult = (shapeId: b3ShapeId, planeResults: StaticArray<b3PlaneResult>, planeCount: i32): bool => {
            for (let i = 0; i < planeCount && self.planeCount < MOVER_PLANE_CAPACITY; ++i) {
                let index = self.planeCount;
                self.planes[index].plane = planeResults[i].plane;
                self.planes[index].pushLimit = FLT_MAX;
                self.planes[index].push = 0.0;
                self.planes[index].clipVelocity = true;
                self.planePoints[index] = b3OffsetPos(self.transform.p, planeResults[i].point);
                self.planeShapes[index] = shapeId;
                self.planeCount += 1;
            }
            return true;
        };
        let acceptAll = (shapeId: b3ShapeId): bool => true;

        this.totalIterations = 0;
        let tolerance: f32 = 0.01;

        for (let iteration = 0; iteration < 5; ++iteration) {
            this.planeCount = 0;
            let mover = b3Capsule(this.capsule.center1, this.capsule.center2, this.capsule.radius);

            b3World_CollideMover(worldId, this.transform.p, mover, moverFilter, planeResult);

            let targetDelta = b3Sub(target, this.transform.p);
            let result = b3SolvePlanes(targetDelta, this.planes, this.planeCount);
            this.totalIterations += result.iterationCount;

            let fraction: f32 = b3World_CastMover(worldId, this.transform.p, mover, result.delta, castFilter, acceptAll);
            let delta = b3MulSV(fraction, result.delta);
            this.transform.p = b3Add(this.transform.p, delta);

            if (b3LengthSquared(delta) < tolerance * tolerance) {
                break;
            }
        }

        for (let i = 0; i < this.planeCount; ++i) {
            let bodyId = b3Shape_GetBody(this.planeShapes[i]);
            let bodyType: b3BodyType = b3Body_GetType(bodyId);
            if (bodyType != b3_dynamicBody) {
                continue;
            }

            let point = this.planePoints[i];
            let normal = b3Neg(this.planes[i].plane.normal);

            let invMassA: f32 = 0.0;
            let invMassB: f32 = b3Body_GetInverseMass(bodyId);
            let invIB = b3Body_GetWorldInverseRotationalInertia(bodyId);

            let pB = b3Body_GetWorldCenter(bodyId);
            let rB = b3SubPos(point, pB);

            let rnB = b3Cross(rB, normal);
            let kNormal: f32 = invMassA + invMassB + b3Dot(rnB, b3MulMV(invIB, rnB));
            let normalMass: f32 = kNormal > 0.0 ? <f32>1.0 / kNormal : <f32>0.0;

            let vB = b3Body_GetLinearVelocity(bodyId);
            let omegaB = b3Body_GetAngularVelocity(bodyId);
            let vrB = b3Add(vB, b3Cross(omegaB, rB));
            let vn: f32 = b3Dot(b3Sub(vrB, this.velocity), normal);
            let impulse: f32 = b3MaxFloat(-normalMass * vn, 0.0);

            let P = b3MulSV(impulse, normal);
            this.velocity = b3MulSub(this.velocity, invMassA, P);

            b3Body_ApplyLinearImpulse(bodyId, P, point, true);
        }

        if (clipVelocity) {
            // Using the velocity clipper can avoid picking up velocity from depenetration.
            this.velocity = b3ClipVector(this.velocity, this.planes, this.planeCount);
        } else if (timeStep > 0.0) {
            this.velocity = b3MulSV(<f32>1.0 / timeStep, b3Sub(this.transform.p, startPosition));
        }
    }
}

// The Box3D side of a demo scene: world lifetime, drawing bodies, contacts
// and joints, mouse joint picking, shooting and stats. Scenes extend it and
// override build() plus whichever hooks they need.

import { Color } from "rive/color";
import { Ray, v3, add, scale } from "./vec";
import { OrbitCamera } from "./camera";
import { DebugDraw, STATE_AWAKE, STATE_SLEEPING, STATE_STATIC, STATE_KINEMATIC, STATE_SELECTED, STATE_STANDING,
    INK_ACCENT, INK_GROUND, INK_LINE, INK_MUTED, bodyInk } from "./debug_draw";
import { DemoScene, SceneStats, PROFILE_PAIRS, PROFILE_COLLIDE, PROFILE_SOLVER, PROFILE_UPDATE, PROFILE_CCD,
    PROFILE_ISLANDS, PROFILE_OTHER, PROFILE_COUNT } from "./scene";
import { BodyView, ShapeCache, toVec3, toXf, toB3 } from "./b3view";
import {
    b3BodyDef,
    b3BodyId,
    b3Body_GetLocalPoint,
    b3Body_GetMassData,
    b3Body_GetContactData,
    b3Body_GetTransform,
    b3Body_GetType,
    b3Body_GetWorldCenter,
    b3Body_GetWorldPoint,
    b3Body_IsAwake,
    b3Body_IsValid,
    b3Body_SetAwake,
    b3Body_SetTargetTransform,
    b3ContactData,
    b3CreateBody,
    b3CreateHullShape,
    b3CreateMotorJoint,
    b3CreateSphereShape,
    b3CreateWorld,
    b3DefaultBodyDef,
    b3DefaultMotorJointDef,
    b3DefaultQueryFilter,
    b3DefaultShapeDef,
    b3DefaultWorldDef,
    b3DestroyBody,
    b3DestroyJoint,
    b3DestroyWorld,
    b3JointId,
    b3Joint_GetBodyA,
    b3Joint_GetBodyB,
    b3Joint_GetLocalFrameA,
    b3Joint_GetLocalFrameB,
    b3Joint_GetType,
    b3Joint_IsValid,
    b3Length,
    b3MakeBoxHull,
    b3Profile,
    b3Quat_identity,
    b3Shape_GetBody,
    b3Sphere,
    b3Transform,
    b3TransformWorldPoint,
    b3Vec3,
    b3WorldDef,
    b3WorldId,
    b3World_CastRayClosest,
    b3World_GetAwakeBodyCount,
    b3World_GetCounters,
    b3World_GetGravity,
    b3World_GetProfile,
    b3World_IsValid,
    b3World_Step,
    b3_dynamicBody,
    b3_filterJoint,
    b3_kinematicBody,
    b3_nullBodyId,
    b3_nullJointId,
    b3_staticBody,
} from "../lib/box3d";

const PICK_LENGTH: f32 = 400;
const CONTACT_CAPACITY: i32 = 64;
const PROFILE_BLEND: f32 = 0.08;

export class PhysicsScene implements DemoScene {
    worldId: b3WorldId = b3WorldId();
    views: Array<BodyView> = new Array<BodyView>();
    joints: Array<b3JointId> = new Array<b3JointId>();
    cache: ShapeCache = new ShapeCache();
    stepCount: i32 = 0;
    subSteps: i32 = 4;
    seed: u32 = 1;

    // Options a scene sets in build().
    maxShots: i32 = 24;
    shotRadius: f32 = 0.25;
    shotSpeed: f32 = 30;
    shotDensity: f32 = 4;
    shotBullet: bool = true;
    contactScale: f32 = 0.3;
    // Uneven ground: shadows land at groundHeight() under each body.
    uneven: bool = false;
    // Hundreds of round bodies: no rings or shadows, which would cost more to draw than the bodies.
    crowd: bool = false;
    groundY: f32 = 0;

    shots: Array<b3BodyId> = new Array<b3BodyId>();

    private mouseBody: b3BodyId = b3_nullBodyId;
    private mouseJoint: b3JointId = b3_nullJointId;
    private mousePoint: b3Vec3 = b3Vec3();
    private mouseFraction: f32 = 0;
    private grabbed: b3BodyId = b3_nullBodyId;
    private grabLocal: b3Vec3 = b3Vec3();

    private contactData: StaticArray<b3ContactData> = new StaticArray<b3ContactData>(CONTACT_CAPACITY);
    private profile: StaticArray<f32> = new StaticArray<f32>(PROFILE_COUNT);
    private profileStep: f32 = 0;

    // ---- hooks ------------------------------------------------------------

    name(): string {
        return "";
    }

    hint(): string {
        return "";
    }

    home(camera: OrbitCamera): void {
        camera.animateTo(v3(0, 2, 0), 20, 0.6, 0.4);
    }

    updateCamera(camera: OrbitCamera): void {}

    worldDef(): b3WorldDef {
        let def = b3DefaultWorldDef();
        def.workerCount = 1;
        return def;
    }

    build(): void {}

    preStep(dt: f32): void {}

    postStep(dt: f32): void {}

    drawExtra(dd: DebugDraw): void {}

    // Height of the ground under x, z, for shadows when uneven is set.
    groundHeight(x: f32, z: f32): f32 {
        return this.groundY;
    }

    keyDown(key: i32): bool {
        return false;
    }

    keyUp(key: i32): bool {
        return false;
    }

    leave(): void {}

    // ---- world ------------------------------------------------------------

    reset(): void {
        if (b3World_IsValid(this.worldId)) {
            b3DestroyWorld(this.worldId);
        }
        this.views = new Array<BodyView>();
        this.joints = new Array<b3JointId>();
        this.shots = new Array<b3BodyId>();
        this.cache.clear();
        this.mouseBody = b3_nullBodyId;
        this.mouseJoint = b3_nullJointId;
        this.grabbed = b3_nullBodyId;
        this.stepCount = 0;
        this.seed = 1;
        this.profileStep = 0;
        for (let i = 0; i < PROFILE_COUNT; i++) {
            this.profile[i] = 0;
        }
        this.worldId = b3CreateWorld(this.worldDef());
        this.build();
    }

    // Numerical Recipes LCG, uniform in [lo, hi].
    random(lo: f32, hi: f32): f32 {
        this.seed = this.seed * 1664525 + 1013904223;
        let unit: f32 = <f32>(this.seed >> 8) / 16777216.0;
        return lo + (hi - lo) * unit;
    }

    addBody(def: b3BodyDef, ink: Color): b3BodyId {
        let id = b3CreateBody(this.worldId, def);
        this.views.push(new BodyView(id, ink));
        return id;
    }

    // A static body whose shapes stand up, so they sort with what moves past them.
    addStanding(def: b3BodyDef, ink: Color): b3BodyId {
        let id = this.addBody(def, ink);
        this.views[this.views.length - 1].standing = true;
        return id;
    }

    nextInk(): Color {
        return bodyInk(this.views.length);
    }

    removeBody(id: b3BodyId): void {
        for (let i = 0; i < this.views.length; i++) {
            if (this.views[i].id == id) {
                this.views[i] = this.views[this.views.length - 1];
                this.views.pop();
                break;
            }
        }
        if (b3Body_IsValid(id)) {
            b3DestroyBody(id);
        }
    }

    addJoint(id: b3JointId): b3JointId {
        this.joints.push(id);
        return id;
    }

    step(dt: f32): void {
        if (this.mouseJoint.index1 != 0 && !b3Joint_IsValid(this.mouseJoint)) {
            // The grabbed body was destroyed under the joint.
            this.release();
        }
        if (this.mouseBody.index1 != 0) {
            b3Body_SetTargetTransform(this.mouseBody, b3Transform(this.mousePoint, b3Quat_identity), dt, true);
        }
        this.preStep(dt);
        b3World_Step(this.worldId, dt, this.subSteps);
        this.postStep(dt);
        this.stepCount++;
        this.blendProfile(b3World_GetProfile(this.worldId));
    }

    private blendProfile(p: b3Profile): void {
        let parts = this.profile;
        let k: f32 = this.stepCount <= 1 ? 1.0 : PROFILE_BLEND;
        let solver = p.constraints;
        let update = p.transforms + p.refit;
        let islands = p.sleepIslands + p.splitIslands;
        let other = Mathf.max(p.step - p.pairs - p.collide - solver - update - p.bullets - islands, 0);
        parts[PROFILE_PAIRS] += (p.pairs - parts[PROFILE_PAIRS]) * k;
        parts[PROFILE_COLLIDE] += (p.collide - parts[PROFILE_COLLIDE]) * k;
        parts[PROFILE_SOLVER] += (solver - parts[PROFILE_SOLVER]) * k;
        parts[PROFILE_UPDATE] += (update - parts[PROFILE_UPDATE]) * k;
        parts[PROFILE_CCD] += (p.bullets - parts[PROFILE_CCD]) * k;
        parts[PROFILE_ISLANDS] += (islands - parts[PROFILE_ISLANDS]) * k;
        parts[PROFILE_OTHER] += (other - parts[PROFILE_OTHER]) * k;
        this.profileStep += (p.step - this.profileStep) * k;
    }

    stats(out: SceneStats): void {
        let c = b3World_GetCounters(this.worldId);
        out.bodies = c.bodyCount;
        out.shapes = c.shapeCount;
        out.contacts = c.contactCount;
        out.joints = c.jointCount;
        out.islands = c.islandCount;
        out.awake = b3World_GetAwakeBodyCount(this.worldId);
        out.profileStep = this.profileStep;
        for (let i = 0; i < PROFILE_COUNT; i++) {
            out.profile[i] = this.profile[i];
        }
    }

    // ---- drawing ----------------------------------------------------------

    draw(dd: DebugDraw): void {
        dd.shadows = !this.uneven && !this.crowd;
        dd.rings = !this.crowd;
        dd.shadowY = this.groundY;
        dd.gridY = this.groundY;
        for (let i = 0; i < this.views.length; i++) {
            let view = this.views[i];
            if (!view.built) {
                view.build(this.cache);
            }
            let id = view.id;
            let xf = toXf(b3Body_GetTransform(id));
            let type = b3Body_GetType(id);
            let state = STATE_AWAKE;
            if (type == b3_staticBody) {
                state = view.standing ? STATE_STATIC | STATE_STANDING : STATE_STATIC;
            } else if (type == b3_kinematicBody) {
                state = STATE_KINEMATIC;
            } else if (!b3Body_IsAwake(id)) {
                state = STATE_SLEEPING;
            }
            if (id == this.grabbed) {
                state |= STATE_SELECTED;
            }
            view.draw(dd, xf, state);
            if (this.uneven && type != b3_staticBody && view.reach > 0) {
                dd.shadow(xf.p, view.reach * 0.85, this.groundHeight(xf.p.x, xf.p.z));
            }
        }
        if (dd.showJoints) {
            this.drawJoints(dd);
        }
        if (dd.showContacts) {
            this.drawContacts(dd);
        }
        if (this.mouseJoint.index1 != 0 && b3Body_IsValid(this.grabbed)) {
            let anchor = toVec3(b3Body_GetWorldPoint(this.grabbed, this.grabLocal));
            let target = toVec3(this.mousePoint);
            dd.segment(anchor, target, INK_ACCENT);
            dd.point(anchor, 4, INK_ACCENT);
            dd.point(target, 6, INK_ACCENT);
        }
        this.drawExtra(dd);
    }

    private drawJoints(dd: DebugDraw): void {
        for (let i = 0; i < this.joints.length; i++) {
            let id = this.joints[i];
            if (!b3Joint_IsValid(id) || b3Joint_GetType(id) == b3_filterJoint) {
                continue;
            }
            let bodyA = b3Joint_GetBodyA(id);
            let bodyB = b3Joint_GetBodyB(id);
            let xfA = b3Body_GetTransform(bodyA);
            let xfB = b3Body_GetTransform(bodyB);
            let pA = toVec3(b3TransformWorldPoint(xfA, b3Joint_GetLocalFrameA(id).p));
            let pB = toVec3(b3TransformWorldPoint(xfB, b3Joint_GetLocalFrameB(id).p));
            dd.segment(toVec3(xfA.p), pA, INK_MUTED);
            dd.segment(toVec3(xfB.p), pB, INK_MUTED);
            dd.segment(pA, pB, INK_LINE);
            dd.point(pA, 3.5, INK_LINE);
        }
    }

    // Each contact once: from body A, or from body B when A is static and so never visited.
    private drawContacts(dd: DebugDraw): void {
        let data = this.contactData;
        let s = this.contactScale;
        for (let i = 0; i < this.views.length; i++) {
            let id = this.views[i].id;
            if (b3Body_GetType(id) == b3_staticBody) {
                continue;
            }
            let count = b3Body_GetContactData(id, data, CONTACT_CAPACITY);
            for (let c = 0; c < count; c++) {
                let cd = data[c];
                let bodyA = b3Shape_GetBody(cd.shapeIdA);
                if (bodyA != id && b3Body_GetType(bodyA) != b3_staticBody) {
                    continue;
                }
                let manifolds = cd.manifolds;
                if (manifolds == null) {
                    continue;
                }
                let centerA = toVec3(b3Body_GetWorldCenter(bodyA));
                for (let m = 0; m < cd.manifoldCount; m++) {
                    let manifold = manifolds[m];
                    let n = toVec3(manifold.normal);
                    for (let k = 0; k < manifold.pointCount; k++) {
                        let mp = manifold.points[k];
                        let p = add(centerA, toVec3(mp.anchorA));
                        let touching = mp.separation <= 0 || mp.totalNormalImpulse > 0;
                        dd.point(p, touching ? 4 : 3, touching ? INK_LINE : INK_MUTED);
                        if (touching) {
                            dd.arrow(p, add(p, scale(n, s)), INK_MUTED);
                        }
                    }
                }
            }
        }
    }

    // ---- input ------------------------------------------------------------

    pointerDown(ray: Ray): bool {
        let origin = toB3(ray.origin);
        let translation = toB3(scale(ray.dir, PICK_LENGTH));
        let result = b3World_CastRayClosest(this.worldId, origin, translation, b3DefaultQueryFilter());
        if (!result.hit) {
            return false;
        }
        let bodyId = b3Shape_GetBody(result.shapeId);
        if (b3Body_GetType(bodyId) != b3_dynamicBody) {
            return false;
        }
        // samples/sample.cpp Sample::MouseDown: a kinematic mouse body and a motor joint.
        this.mousePoint = result.point;
        let bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_kinematicBody;
        bodyDef.position = this.mousePoint;
        bodyDef.enableSleep = false;
        this.mouseBody = b3CreateBody(this.worldId, bodyDef);

        let jointDef = b3DefaultMotorJointDef();
        jointDef.base.bodyIdA = this.mouseBody;
        jointDef.base.bodyIdB = bodyId;
        this.grabLocal = b3Body_GetLocalPoint(bodyId, result.point);
        jointDef.base.localFrameB.p = this.grabLocal;
        jointDef.linearHertz = 7.5;
        jointDef.linearDampingRatio = 1.0;

        let massData = b3Body_GetMassData(bodyId);
        let g = b3Length(b3World_GetGravity(this.worldId));
        let mg: f32 = massData.mass * g;
        jointDef.maxSpringForce = 100.0 * mg;
        if (massData.mass > 0.0) {
            // This acts like angular friction
            let trace = massData.inertia.cx.x + massData.inertia.cy.y + massData.inertia.cz.z;
            let lever = Mathf.sqrt(trace / (3.0 * massData.mass));
            jointDef.maxVelocityTorque = 0.5 * lever * mg;
        }
        this.mouseJoint = b3CreateMotorJoint(this.worldId, jointDef);
        b3Body_SetAwake(bodyId, true);
        this.mouseFraction = result.fraction;
        this.grabbed = bodyId;
        return true;
    }

    pointerMove(ray: Ray): void {
        if (this.mouseJoint.index1 != 0) {
            this.mousePoint = toB3(add(ray.origin, scale(ray.dir, this.mouseFraction * PICK_LENGTH)));
        }
    }

    pointerUp(ray: Ray): void {
        this.release();
    }

    private release(): void {
        if (b3Joint_IsValid(this.mouseJoint)) {
            b3DestroyJoint(this.mouseJoint, true);
        }
        if (b3Body_IsValid(this.mouseBody)) {
            b3DestroyBody(this.mouseBody);
        }
        this.mouseJoint = b3_nullJointId;
        this.mouseBody = b3_nullBodyId;
        this.grabbed = b3_nullBodyId;
        this.mouseFraction = 0;
    }

    shoot(ray: Ray): void {
        let bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = toB3(add(ray.origin, scale(ray.dir, 2)));
        bodyDef.linearVelocity = toB3(scale(ray.dir, this.shotSpeed));
        bodyDef.isBullet = this.shotBullet;
        let id = this.addBody(bodyDef, INK_LINE);
        let shapeDef = b3DefaultShapeDef();
        shapeDef.density *= this.shotDensity;
        b3CreateSphereShape(id, shapeDef, b3Sphere(b3Vec3(0, 0, 0), this.shotRadius));
        this.shots.push(id);
        if (this.shots.length > this.maxShots) {
            this.removeBody(takeFirst(this.shots));
        }
    }
}

// Array.shift, which rasc lacks for value class arrays.
export function takeFirst(list: Array<b3BodyId>): b3BodyId {
    let first = list[0];
    for (let i = 1; i < list.length; i++) {
        list[i - 1] = list[i];
    }
    list.pop();
    return first;
}

// A static ground box with its top face at y = 0.
export function addGroundBox(scene: PhysicsScene, extent: f32): b3BodyId {
    let bodyDef = b3DefaultBodyDef();
    bodyDef.position = b3Vec3(0, -1, 0);
    let groundId = scene.addBody(bodyDef, INK_GROUND);
    b3CreateHullShape(groundId, b3DefaultShapeDef(), b3MakeBoxHull(extent, 1, extent).base);
    return groundId;
}

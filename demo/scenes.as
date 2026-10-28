// The Box3D scenes and makeScenes(), the registry the shell reads. Most come
// from the Box3D samples by way of verify/scenes/scenes.as, sized to stay
// interactive.

import { Vec3, v3, add, sub, scale, normalize } from "./vec";
import { OrbitCamera } from "./camera";
import { DebugDraw, DebugHeightField, STATE_KINEMATIC, INK_ACCENT, INK_BODY, INK_CLAY, INK_GROUND, INK_LINE, INK_MUTED,
    INK_SAND, INK_SLATE, INK_STONE, bodyInk } from "./debug_draw";
import { DemoScene } from "./scene";
import { PhysicsScene, addGroundBox, takeFirst } from "./physics";
import { createHuman } from "./human";
import { CharacterMover } from "./mover";
import { toVec3, toB3, toXf, fieldHeight } from "./b3view";
import { g_threads } from "./threads";
import { b3EnableTrace, b3GetTraceBegin, b3GetTraceEnd, b3TraceBegin, b3TraceEnd } from "../lib/trace";
import {
    B3_DEG_TO_RAD,
    B3_PI,
    b3BodyId,
    b3Body_GetPosition,
    b3Body_SetAngularVelocity,
    b3Body_SetAwake,
    b3Body_SetLinearVelocity,
    b3Body_SetTransform,
    b3Capsule,
    b3CompoundCapsuleDef,
    b3CompoundDef,
    b3CompoundHullDef,
    b3CompoundSphereDef,
    b3CreateBakedCompoundShape,
    b3CreateCapsuleShape,
    b3CreateCompound,
    b3CreateCylinder,
    b3CreateHeightFieldShape,
    b3CreateHullShape,
    b3CreateMeshShape,
    b3CreateRock,
    b3CreateSphereShape,
    b3CreateSphericalJoint,
    b3CreateWave,
    b3CreateWaveMesh,
    b3DefaultBodyDef,
    b3DefaultShapeDef,
    b3DefaultSphericalJointDef,
    b3DefaultSurfaceMaterial,
    b3HullData,
    b3MakeBoxHull,
    b3MakeQuatFromAxisAngle,
    b3MakeTransformedBoxHull,
    b3Normalize,
    b3Quat,
    b3Quat_identity,
    b3Sphere,
    b3Transform,
    b3Transform_identity,
    b3Vec2,
    b3Vec3,
    b3Vec3_axisY,
    b3Vec3_axisZ,
    b3Vec3_one,
    b3Vec3_zero,
    b3_dynamicBody,
    b3_kinematicBody,
    b3_nullBodyId,
    b3WorldDef,
} from "../lib/box3d";

// Terrain a step lighter than a ground box so its relief reads.
const INK_TERRAIN: u32 = 0xFF4A4640;

const KEY_SPACE: i32 = 32;
const KEY_A: i32 = 65;
const KEY_D: i32 = 68;
const KEY_S: i32 = 83;
const KEY_W: i32 = 87;
const KEY_RIGHT: i32 = 262;
const KEY_LEFT: i32 = 263;
const KEY_DOWN: i32 = 264;
const KEY_UP: i32 = 265;
const KEY_LEFT_SHIFT: i32 = 340;
const KEY_RIGHT_SHIFT: i32 = 344;

function randomAxisAngle(scene: PhysicsScene): b3Quat {
    let axis = b3Vec3(scene.random(-1, 1), scene.random(-1, 1), scene.random(-1, 1));
    return b3MakeQuatFromAxisAngle(b3Normalize(axis), scene.random(-B3_PI, B3_PI));
}

// ---- pyramid ---------------------------------------------------------------

// shared/benchmarks.c CreateLargePyramid at a fifth of the benchmark's base.
class PyramidScene extends PhysicsScene {
    name(): string {
        return "Pyramid";
    }

    hint(): string {
        return "210 boxes, 20 at the base. Drag a box out of the side, or Tab to shoot.";
    }

    home(camera: OrbitCamera): void {
        camera.animateTo(v3(0, 8, 0), 40, 0.55, 0.3);
    }

    build(): void {
        addGroundBox(this, 30);
        let baseCount = 20;
        let h: f32 = 0.5;
        let box = b3MakeBoxHull(h, h, h);
        let bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        let shapeDef = b3DefaultShapeDef();
        for (let i = 0; i < baseCount; ++i) {
            let y: f32 = (2.0 * <f32>i + 1.0) * h;
            for (let j = i; j < baseCount; ++j) {
                let x: f32 = (<f32>i + 1.0) * h + 2.0 * <f32>(j - i) * h - h * <f32>baseCount;
                bodyDef.position = b3Vec3(x, y, 0.0);
                let bodyId = this.addBody(bodyDef, bodyInk(i));
                b3CreateHullShape(bodyId, shapeDef, box.base);
            }
        }
    }
}

// ---- card house ------------------------------------------------------------

// samples/sample_stacking.cpp CardHouse.
class CardHouseScene extends PhysicsScene {
    name(): string {
        return "Card house";
    }

    hint(): string {
        return "Playing cards 2 mm thick leaning on friction alone. Pull one, or shoot.";
    }

    home(camera: OrbitCamera): void {
        camera.animateTo(v3(0.8, 0.8, 0), 4.6, 0.32, 0.22);
    }

    build(): void {
        this.shotRadius = 0.06;
        this.shotSpeed = 8;
        this.contactScale = 0.06;
        addGroundBox(this, 4);

        let shapeDef = b3DefaultShapeDef();
        shapeDef.baseMaterial.rollingResistance = 0.05;
        shapeDef.baseMaterial.friction = 0.7;

        let cardHeight: f32 = 0.2;
        let cardThickness: f32 = 0.001;
        let cardDepth: f32 = 0.1;

        let angle0: f32 = 25.0 * B3_PI / 180.0;
        let angle1: f32 = -25.0 * B3_PI / 180.0;
        let angle2: f32 = 0.5 * B3_PI;

        let cardBox = b3MakeBoxHull(cardThickness, cardHeight, cardDepth);
        let bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;

        let Nb = 5;
        let z0: f32 = 0.0;
        let y: f32 = cardHeight - 0.02;
        while (Nb > 0) {
            let z: f32 = z0;
            for (let i = 0; i < Nb; i++) {
                if (i != Nb - 1) {
                    bodyDef.position = b3Vec3(z + 0.25, y + cardHeight - 0.015, 0.0);
                    bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, angle2);
                    let bodyId = this.addBody(bodyDef, INK_SAND);
                    b3CreateHullShape(bodyId, shapeDef, cardBox.base);
                }

                bodyDef.position = b3Vec3(z, y, 0.0);
                bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, angle1);
                let bodyId = this.addBody(bodyDef, INK_BODY);
                b3CreateHullShape(bodyId, shapeDef, cardBox.base);

                z += 0.175;

                bodyDef.position = b3Vec3(z, y, 0.0);
                bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, angle0);
                bodyId = this.addBody(bodyDef, INK_CLAY);
                b3CreateHullShape(bodyId, shapeDef, cardBox.base);

                z += 0.175;
            }
            y += cardHeight * 2.0 - 0.03;
            z0 += 0.175;
            Nb -= 1;
        }
    }
}

// ---- ragdolls and a ball and chain -----------------------------------------

// samples/sample_joint.cpp BallAndChain swinging through a pile of shared/human.c ragdolls.
class RagdollScene extends PhysicsScene {
    name(): string {
        return "Ragdolls and chain";
    }

    hint(): string {
        return "Eight spherical joints swing a wrecking ball through six ragdolls. J hides joints.";
    }

    home(camera: OrbitCamera): void {
        camera.animateTo(v3(0, 3.5, 0), 17, 0.42, 0.2);
    }

    build(): void {
        addGroundBox(this, 20);

        let linkCount = 8;
        let linkRadius: f32 = 0.125;
        let linkExtent: f32 = 0.5;
        let sphereRadius: f32 = 1.0;
        // The ball's lowest swing just clears the floor.
        let anchorY: f32 = 2.0 * linkExtent * <f32>linkCount + sphereRadius + 1.45;

        let bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Vec3(0, anchorY, 0);
        let anchorId = this.addBody(bodyDef, INK_GROUND);
        let shapeDef = b3DefaultShapeDef();
        b3CreateHullShape(anchorId, shapeDef, b3MakeBoxHull(0.2, 0.2, 0.2).base);

        let capsule = b3Capsule(b3Vec3(-linkExtent, 0.0, 0.0), b3Vec3(linkExtent, 0.0, 0.0), linkRadius);
        bodyDef.type = b3_dynamicBody;
        let parent = anchorId;
        let jointDef = b3DefaultSphericalJointDef();
        jointDef.base.localFrameA = b3Transform_identity;
        jointDef.base.localFrameB = b3Transform(b3Vec3(-linkExtent, 0.0, 0.0), b3Quat_identity);
        jointDef.enableMotor = true;
        jointDef.maxMotorTorque = 10.0;

        for (let i = 0; i < linkCount; ++i) {
            bodyDef.position = b3Vec3((1.0 + 2.0 * <f32>i) * linkExtent, anchorY, 0.0);
            let childId = this.addBody(bodyDef, INK_SLATE);
            b3CreateCapsuleShape(childId, shapeDef, capsule);
            jointDef.base.bodyIdA = parent;
            jointDef.base.bodyIdB = childId;
            this.addJoint(b3CreateSphericalJoint(this.worldId, jointDef));
            jointDef.base.localFrameA = b3Transform(b3Vec3(linkExtent, 0.0, 0.0), b3Quat_identity);
            parent = childId;
        }

        bodyDef.position = b3Vec3(2.0 * <f32>linkCount * linkExtent + sphereRadius, anchorY, 0.0);
        let ballId = this.addBody(bodyDef, INK_STONE);
        b3CreateSphereShape(ballId, shapeDef, b3Sphere(b3Vec3(0.0, 0.0, 0.0), sphereRadius));
        jointDef.base.bodyIdA = parent;
        jointDef.base.bodyIdB = ballId;
        jointDef.base.localFrameB = b3Transform(b3Vec3(-sphereRadius, 0.0, 0.0), b3Quat_identity);
        this.addJoint(b3CreateSphericalJoint(this.worldId, jointDef));

        // Two rows of three, the back row standing on the front row's heads.
        for (let i = 0; i < 6; ++i) {
            let row = i / 3;
            let column = i % 3;
            let position = b3Vec3(-0.3 + 0.6 * <f32>row, 0.02 + 1.9 * <f32>row, -0.9 + 0.9 * <f32>column);
            createHuman(this, position, 10.0, 0.5, 0.7, i + 1, bodyInk(i + 1));
        }
    }
}

// ---- rain on a wave mesh ---------------------------------------------------

const RAIN_GRID: i32 = 40;
const RAIN_AMPLITUDE: f32 = 0.5;
const RAIN_ROW_HZ: f32 = 0.1;
const RAIN_COLUMN_HZ: f32 = 0.2;
const RAIN_MAX: i32 = 260;
const RAIN_INTERVAL: i32 = 3;

// samples/sample_continuous.cpp MeshDrop with bodies large enough to see, falling one every few steps.
class RainScene extends PhysicsScene {
    falling: Array<b3BodyId> = new Array<b3BodyId>();
    spawned: i32 = 0;

    name(): string {
        return "Rain on mesh";
    }

    hint(): string {
        return "Capsules and boxes fall on a 3200 triangle wave mesh, the oldest recycled past 260.";
    }

    home(camera: OrbitCamera): void {
        camera.animateTo(v3(0, 0, 0), 40, 0.5, 0.62);
    }

    // The wave b3CreateWaveMesh lays out, between its vertices.
    groundHeight(x: f32, z: f32): f32 {
        let half: f32 = 0.5 * <f32>RAIN_GRID;
        let ix = x + half;
        let iz = z + half;
        return RAIN_AMPLITUDE * Mathf.sin(2.0 * B3_PI * RAIN_COLUMN_HZ * ix) * Mathf.sin(2.0 * B3_PI * RAIN_ROW_HZ * iz);
    }

    build(): void {
        this.uneven = true;
        this.falling = new Array<b3BodyId>();
        this.spawned = 0;
        this.seed = 12345;

        let bodyDef = b3DefaultBodyDef();
        let groundId = this.addBody(bodyDef, INK_TERRAIN);

        let cellWidth: f32 = 1.0;
        let mesh = b3CreateWaveMesh(RAIN_GRID, RAIN_GRID, cellWidth, RAIN_AMPLITUDE, RAIN_ROW_HZ, RAIN_COLUMN_HZ)!;
        let shapeDef = b3DefaultShapeDef();
        b3CreateMeshShape(groundId, shapeDef, mesh, b3Vec3_one);

        let extent: f32 = 0.5 * <f32>RAIN_GRID * cellWidth;
        let halfHeight: f32 = 0.6;
        let wallId = this.addStanding(b3DefaultBodyDef(), INK_GROUND);
        for (let side = 0; side < 4; side++) {
            let sign: f32 = side % 2 == 0 ? -1.0 : 1.0;
            let alongX = side < 2;
            let transform = b3Transform(alongX ? b3Vec3(0.0, halfHeight, sign * extent) : b3Vec3(sign * extent, halfHeight, 0.0),
                b3Quat_identity);
            let wall = alongX ? b3MakeTransformedBoxHull(extent, halfHeight, 0.1, transform)
                : b3MakeTransformedBoxHull(0.1, halfHeight, extent, transform);
            b3CreateHullShape(wallId, shapeDef, wall.base);
        }
    }

    preStep(dt: f32): void {
        if (this.stepCount % RAIN_INTERVAL != 0) {
            return;
        }
        if (this.falling.length >= RAIN_MAX) {
            this.removeBody(takeFirst(this.falling));
        }
        let bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = b3Vec3(this.random(-16, 16), this.random(10, 13), this.random(-16, 16));
        bodyDef.rotation = randomAxisAngle(this);
        bodyDef.angularVelocity = b3Vec3(this.random(-4, 4), this.random(-4, 4), this.random(-4, 4));
        let id = this.addBody(bodyDef, bodyInk(this.spawned));
        let shapeDef = b3DefaultShapeDef();
        if (this.spawned % 5 < 3) {
            shapeDef.baseMaterial.rollingResistance = 0.4;
            b3CreateCapsuleShape(id, shapeDef, b3Capsule(b3Vec3(0, -0.28, 0), b3Vec3(0, 0.28, 0), 0.18));
        } else {
            shapeDef.baseMaterial.rollingResistance = 0.1;
            b3CreateHullShape(id, shapeDef, b3MakeBoxHull(0.22, 0.22, 0.38).base);
        }
        this.falling.push(id);
        this.spawned++;
    }
}

// ---- height field ----------------------------------------------------------

const FIELD_COUNT: i32 = 41;
const FIELD_HEIGHT: f32 = 2.2;
const FIELD_MAX: i32 = 110;

// A wave height field from b3CreateWave with spheres, wheels, capsules and rocks rolling into its
// valleys, past a static compound of posts, blocks and a dome.
class HeightFieldScene extends PhysicsScene {
    field: DebugHeightField? = null;
    rock: b3HullData? = null;
    wheel: b3HullData? = null;
    rolling: Array<b3BodyId> = new Array<b3BodyId>();
    origin: f32 = 0;

    name(): string {
        return "Height field";
    }

    hint(): string {
        return "A 41 by 41 height field. Bodies roll into the valleys past a static compound.";
    }

    home(camera: OrbitCamera): void {
        camera.animateTo(v3(0, 0, 0), 42, 0.7, 0.58);
    }

    groundHeight(x: f32, z: f32): f32 {
        let field = this.field;
        if (field == null) {
            return 0;
        }
        return fieldHeight(field, x - this.origin, z - this.origin);
    }

    build(): void {
        this.uneven = true;
        this.rolling = new Array<b3BodyId>();
        this.seed = 52977;

        let fieldScale = b3Vec3(1.0, FIELD_HEIGHT, 1.0);
        let hf = b3CreateWave(FIELD_COUNT, FIELD_COUNT, fieldScale, 0.05, 0.05, false);
        this.origin = -0.5 * fieldScale.x * <f32>(FIELD_COUNT - 1);
        let bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Vec3(this.origin, 0, this.origin);
        let groundId = this.addBody(bodyDef, INK_TERRAIN);
        b3CreateHeightFieldShape(groundId, b3DefaultShapeDef(), hf);
        this.field = null;

        this.buildCompound();

        this.rock = b3CreateRock(0.5);
        this.wheel = b3CreateCylinder(0.5, 0.55, -0.25, 14);
        for (let i = 0; i < 48; i++) {
            this.spawn();
        }
    }

    // Posts, two blocks and a dome on the flat saddle at the center, one baked compound.
    private buildCompound(): void {
        let material = b3DefaultSurfaceMaterial();
        let def = new b3CompoundDef();

        let postCount = 6;
        let capsules = new StaticArray<b3CompoundCapsuleDef>(postCount);
        for (let i = 0; i < postCount; i++) {
            let a: f32 = 2.0 * B3_PI * <f32>i / <f32>postCount;
            let x: f32 = 3.2 * Mathf.cos(a);
            let z: f32 = 3.2 * Mathf.sin(a);
            let post = b3CompoundCapsuleDef();
            post.capsule = b3Capsule(b3Vec3(x, -0.4, z), b3Vec3(x, 0.9, z), 0.28);
            post.material = material;
            capsules[i] = post;
        }
        def.capsules = capsules;
        def.capsuleCount = postCount;

        let block = b3MakeBoxHull(0.9, 0.35, 0.5).base;
        let hulls = new StaticArray<b3CompoundHullDef>(2);
        hulls[0] = new b3CompoundHullDef(block, b3Transform(b3Vec3(-1.4, 0.0, 1.6),
            b3MakeQuatFromAxisAngle(b3Vec3_axisY, 0.5)), material);
        hulls[1] = new b3CompoundHullDef(block, b3Transform(b3Vec3(1.5, 0.0, -1.5),
            b3MakeQuatFromAxisAngle(b3Vec3_axisY, -0.4)), material);
        def.hulls = hulls;
        def.hullCount = 2;

        let spheres = new StaticArray<b3CompoundSphereDef>(1);
        let dome = b3CompoundSphereDef();
        dome.sphere = b3Sphere(b3Vec3(0.0, -0.3, 0.0), 1.0);
        dome.material = material;
        spheres[0] = dome;
        def.spheres = spheres;
        def.sphereCount = 1;

        let compound = b3CreateCompound(def);
        if (compound == null) {
            return;
        }
        let bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Vec3(0.0, 0.0, 0.0);
        let id = this.addStanding(bodyDef, INK_STONE);
        b3CreateBakedCompoundShape(id, b3DefaultShapeDef(), compound!);
    }

    // Over a peak, where sin x sin z is one: x and z both from 5, 25 or both from 15, 35.
    private spawnPoint(): b3Vec3 {
        let base: f32 = this.random(0, 1) < 0.5 ? 5.0 : 15.0;
        let px: f32 = base;
        let pz: f32 = base;
        if (this.random(0, 1) < 0.5) {
            px += 20.0;
        }
        if (this.random(0, 1) < 0.5) {
            pz += 20.0;
        }
        return b3Vec3(this.origin + px + this.random(-2.5, 2.5), FIELD_HEIGHT + this.random(1.0, 4.0),
            this.origin + pz + this.random(-2.5, 2.5));
    }

    private spawn(): void {
        let bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = this.spawnPoint();
        bodyDef.rotation = randomAxisAngle(this);
        let index = this.rolling.length;
        let id = this.addBody(bodyDef, bodyInk(index));
        let shapeDef = b3DefaultShapeDef();
        let kind = index % 4;
        if (kind == 0) {
            shapeDef.baseMaterial.rollingResistance = 0.02;
            b3CreateSphereShape(id, shapeDef, b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.5));
        } else if (kind == 1) {
            shapeDef.baseMaterial.rollingResistance = 0.02;
            b3CreateHullShape(id, shapeDef, this.wheel!);
        } else if (kind == 2) {
            shapeDef.baseMaterial.rollingResistance = 0.05;
            b3CreateCapsuleShape(id, shapeDef, b3Capsule(b3Vec3(-0.4, 0.0, 0.0), b3Vec3(0.4, 0.0, 0.0), 0.35));
        } else {
            shapeDef.baseMaterial.rollingResistance = 0.2;
            b3CreateHullShape(id, shapeDef, this.rock!);
        }
        this.rolling.push(id);
    }

    preStep(dt: f32): void {
        if (this.stepCount % 40 == 0 && this.rolling.length < FIELD_MAX) {
            this.spawn();
        }
        // Whatever leaves over an edge comes back over a peak.
        for (let i = 0; i < this.rolling.length; i++) {
            let id = this.rolling[i];
            if (b3Body_GetPosition(id).y < -8.0) {
                b3Body_SetTransform(id, this.spawnPoint(), b3Quat_identity);
                b3Body_SetLinearVelocity(id, b3Vec3_zero);
                b3Body_SetAngularVelocity(id, b3Vec3_zero);
                b3Body_SetAwake(id, true);
            }
        }
    }

    draw(dd: DebugDraw): void {
        if (this.field == null && this.views.length > 0) {
            let ground = this.views[0];
            if (!ground.built) {
                ground.build(this.cache);
            }
            if (ground.shapes.length > 0) {
                this.field = ground.shapes[0].field;
            }
        }
        dd.showGrid = false;
        super.draw(dd);
    }
}

// ---- bullets ---------------------------------------------------------------

const VOLLEY_INTERVAL: i32 = 150;

// samples/sample_continuous.cpp BulletVersusStack and ThinWall side by side.
class BulletScene extends PhysicsScene {
    volley: Array<b3BodyId> = new Array<b3BodyId>();
    stackBullet: b3BodyId = b3_nullBodyId;

    name(): string {
        return "Bullets";
    }

    hint(): string {
        return "A 500 m/s bullet into a stack, and volleys at 180 m/s into a 20 cm wall. Click or B fires.";
    }

    home(camera: OrbitCamera): void {
        camera.animateTo(v3(3, 3, -4), 31, 0.95, 0.3);
    }

    build(): void {
        this.shotSpeed = 200;
        this.shotDensity = 10;
        this.maxShots = 12;
        this.volley = new Array<b3BodyId>();
        this.stackBullet = b3_nullBodyId;

        addGroundBox(this, 30);
        let shapeDef = b3DefaultShapeDef();

        let backing = this.addStanding(b3DefaultBodyDef(), INK_SLATE);
        b3CreateHullShape(backing, shapeDef,
            b3MakeTransformedBoxHull(0.1, 5.0, 2.5, b3Transform(b3Vec3(-1.0, 5.0, 0.0), b3Quat_identity)).base);

        let box = b3MakeBoxHull(0.5, 0.5, 0.5);
        let bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        for (let row = 0; row < 10; ++row) {
            bodyDef.position = b3Vec3(0.0, 0.5 + 1.1 * <f32>row, 0.0);
            let bodyId = this.addBody(bodyDef, bodyInk(row));
            b3CreateHullShape(bodyId, shapeDef, box.base);
        }

        let wall = this.addStanding(b3DefaultBodyDef(), INK_SLATE);
        b3CreateHullShape(wall, shapeDef,
            b3MakeTransformedBoxHull(0.1, 4.0, 4.5, b3Transform(b3Vec3(-4.0, 4.0, -9.0), b3Quat_identity)).base);
    }

    // BulletVersusStack::Launch
    private launchAtStack(): void {
        if (this.stackBullet.index1 != 0) {
            this.removeBody(this.stackBullet);
        }
        let bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.isBullet = true;
        bodyDef.position = b3Vec3(20.5, 5.5, 0.0);
        bodyDef.linearVelocity = b3Vec3(-500.0, 0.0, 0.0);
        this.stackBullet = this.addBody(bodyDef, INK_LINE);
        let shapeDef = b3DefaultShapeDef();
        shapeDef.density *= 10.0;
        b3CreateSphereShape(this.stackBullet, shapeDef, b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.25));
    }

    // ThinWall: a sphere, a capsule and a box, none of them bullets, against a static wall.
    private launchVolley(): void {
        for (let i = 0; i < this.volley.length; i++) {
            this.removeBody(this.volley[i]);
        }
        this.volley = new Array<b3BodyId>();
        let bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        let shapeDef = b3DefaultShapeDef();
        shapeDef.baseMaterial.rollingResistance = 0.1;
        for (let k = 0; k < 3; k++) {
            bodyDef.position = b3Vec3(18.0, 4.0, -9.0 + 2.5 * <f32>(k - 1));
            bodyDef.linearVelocity = b3Vec3(-180.0, 0.0, 0.0);
            bodyDef.angularVelocity = b3Vec3(0.0, 5.0 * <f32>(k - 1), 20.0);
            let id = this.addBody(bodyDef, INK_ACCENT);
            if (k == 0) {
                b3CreateSphereShape(id, shapeDef, b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.1));
            } else if (k == 1) {
                b3CreateCapsuleShape(id, shapeDef, b3Capsule(b3Vec3(0.0, 0.0, -0.3), b3Vec3(0.0, 0.0, 0.3), 0.1));
            } else {
                b3CreateHullShape(id, shapeDef, b3MakeBoxHull(0.1, 0.1, 0.4).base);
            }
            this.volley.push(id);
        }
    }

    preStep(dt: f32): void {
        if (this.stepCount == 30) {
            this.launchAtStack();
        }
        if (this.stepCount % VOLLEY_INTERVAL == 10) {
            this.launchVolley();
        }
    }
}

// ---- character -------------------------------------------------------------

const STAIRS_X: f32 = -3.0;
const MOVER_START_X: f32 = -2.6;
const MOVER_START_Z: f32 = 1.5;

// samples/mover.cpp driven by keys across a small level: a step, a ramp to a ledge, walls, a
// staircase of tread boxes and crates to push.
class CharacterScene extends PhysicsScene {
    mover: CharacterMover = new CharacterMover(b3Vec3(MOVER_START_X, 1.4, MOVER_START_Z));
    forward: Vec3 = Vec3(0, 0, -1);
    right: Vec3 = Vec3(1, 0, 0);
    followed: Vec3 = Vec3(0, 0, 0);
    following: bool = false;
    keyW: bool = false;
    keyA: bool = false;
    keyS: bool = false;
    keyD: bool = false;
    shift: bool = false;
    jumpQueued: bool = false;

    name(): string {
        return "Character";
    }

    hint(): string {
        return "WASD or arrows walk the capsule, Space jumps, Shift runs. Climb the stairs, push the crates.";
    }

    home(camera: OrbitCamera): void {
        let p = this.mover.transform.p;
        camera.animateTo(v3(p.x + 2.0, p.y - 0.2, p.z - 1.0), 15, 0.2, 0.45);
        this.following = false;
    }

    build(): void {
        this.mover = new CharacterMover(b3Vec3(MOVER_START_X, 1.4, MOVER_START_Z));
        this.jumpQueued = false;
        addGroundBox(this, 16);

        let shapeDef = b3DefaultShapeDef();
        let level = this.addStanding(b3DefaultBodyDef(), INK_STONE);
        let piece = (hx: f32, hy: f32, hz: f32, x: f32, y: f32, z: f32, q: b3Quat): void => {
            b3CreateHullShape(level, shapeDef, b3MakeTransformedBoxHull(hx, hy, hz, b3Transform(b3Vec3(x, y, z), q)).base);
        };

        // A 30 cm step.
        piece(1.2, 0.15, 1.2, 3.0, 0.15, -3.0, b3Quat_identity);
        // An 18 degree ramp up to a ledge.
        let ramp: f32 = 18.0 * B3_DEG_TO_RAD;
        let rampHalf: f32 = 2.5;
        let rise: f32 = 2.0 * rampHalf * Mathf.sin(ramp);
        piece(rampHalf, 0.1, 1.2, 7.4, 0.5 * rise - 0.1 * Mathf.cos(ramp), -3.0, b3MakeQuatFromAxisAngle(b3Vec3_axisZ, ramp));
        let ledgeX: f32 = 7.4 + rampHalf * Mathf.cos(ramp) + 1.5;
        piece(1.5, 0.5 * rise, 1.2, ledgeX, 0.5 * rise, -3.0, b3Quat_identity);
        // A long wall behind the ramp and the stairs, in 2 m blocks so the painter's sort holds
        // up against what stands in front of it, and a short one to walk into.
        for (let i = 0; i < 9; i++) {
            piece(1.0, 1.25, 0.2, -5.0 + 2.0 * <f32>i, 1.25, -6.6, b3Quat_identity);
        }
        piece(1.6, 1.0, 0.2, 2.0, 1.0, 4.5, b3Quat_identity);
        // Eight 20 cm treads rising away from the start, up to a landing against the wall.
        let treadCount = 8;
        let tread: f32 = 0.2;
        let run: f32 = 0.45;
        for (let i = 0; i < treadCount; i++) {
            let h: f32 = tread * <f32>(i + 1);
            piece(1.2, 0.5 * h, 0.5 * run, STAIRS_X, 0.5 * h, -0.5 - run * (<f32>i + 0.5), b3Quat_identity);
        }
        let top: f32 = tread * <f32>treadCount;
        piece(1.2, 0.5 * top, 0.9, STAIRS_X, 0.5 * top, -0.5 - run * <f32>treadCount - 0.9, b3Quat_identity);

        // Crates.
        let bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        let crate = b3MakeBoxHull(0.4, 0.4, 0.4);
        let spots: StaticArray<f32> = [3.0, 0.4, 2.0, 4.1, 0.4, 2.6, 3.5, 1.2, 2.3, 5.4, 0.4, 1.2];
        for (let i = 0; i < spots.length / 3; i++) {
            bodyDef.position = b3Vec3(spots[3 * i], spots[3 * i + 1], spots[3 * i + 2]);
            let id = this.addBody(bodyDef, i % 2 == 0 ? INK_SAND : INK_CLAY);
            b3CreateHullShape(id, shapeDef, crate.base);
        }
        let big = this.addBody(bodyDef, INK_BODY);
        b3Body_SetTransform(big, b3Vec3(6.5, 0.6, 3.4), b3MakeQuatFromAxisAngle(b3Vec3_axisY, 0.4));
        b3CreateHullShape(big, shapeDef, b3MakeBoxHull(0.6, 0.6, 0.6).base);
    }

    updateCamera(camera: OrbitCamera): void {
        this.forward = normalize(v3(camera.forward.x, 0, camera.forward.z));
        this.right = camera.right;
        let p = toVec3(this.mover.transform.p);
        if (this.following) {
            camera.shiftTarget(sub(p, this.followed));
        }
        this.followed = p;
        this.following = true;
    }

    preStep(dt: f32): void {
        let throttle = b3Vec2(0.0, 0.0);
        if (this.keyW) {
            throttle.x += 1.0;
        }
        if (this.keyS) {
            throttle.x -= 1.0;
        }
        if (this.keyD) {
            throttle.y += 1.0;
        }
        if (this.keyA) {
            throttle.y -= 1.0;
        }
        let mover = this.mover;
        if (this.jumpQueued) {
            mover.jump();
            this.jumpQueued = false;
        }
        mover.sprint = mover.onGround && this.shift;
        mover.solveMove(this.worldId, dt, toB3(this.forward), toB3(this.right), throttle, true);
    }

    drawExtra(dd: DebugDraw): void {
        let mover = this.mover;
        let xf = toXf(mover.transform);
        let c = mover.capsule;
        dd.capsule(xf, toVec3(c.center1), toVec3(c.center2), c.radius, INK_LINE, STATE_KINEMATIC);
        let p = xf.p;
        dd.arrow(p, add(p, scale(toVec3(mover.velocity), 0.35)), INK_ACCENT);
        dd.segment(toVec3(mover.rayOrigin), toVec3(mover.rayEnd), INK_MUTED);
        if (mover.rayHit) {
            dd.point(toVec3(mover.rayEnd), 4, INK_MUTED);
        }
        for (let i = 0; i < mover.planeCount; i++) {
            let plane = mover.planes[i].plane;
            let n = toVec3(plane.normal);
            let p1 = add(p, scale(n, plane.offset - c.radius));
            dd.point(p1, 4, INK_ACCENT);
            dd.segment(p1, add(p1, scale(n, 0.25)), INK_ACCENT);
        }
    }

    private setKey(key: i32, down: bool): bool {
        if (key == KEY_W || key == KEY_UP) {
            this.keyW = down;
        } else if (key == KEY_S || key == KEY_DOWN) {
            this.keyS = down;
        } else if (key == KEY_A || key == KEY_LEFT) {
            this.keyA = down;
        } else if (key == KEY_D || key == KEY_RIGHT) {
            this.keyD = down;
        } else if (key == KEY_LEFT_SHIFT || key == KEY_RIGHT_SHIFT) {
            this.shift = down;
        } else if (key == KEY_SPACE) {
            if (down) {
                this.jumpQueued = true;
            }
        } else {
            return false;
        }
        return true;
    }

    keyDown(key: i32): bool {
        return this.setKey(key, true);
    }

    keyUp(key: i32): bool {
        return this.setKey(key, false);
    }
}

// ---- threads ---------------------------------------------------------------

const PIT_EXTENT: f32 = 9.0;
const PIT_WALL: f32 = 1.1;
const PIT_SIDE: i32 = 16;
const PIT_LAYERS: i32 = 8;
const PIT_SPIN: f32 = 0.45;

// A walled pit of balls and capsules stirred by a kinematic paddle, with sleep off, so every step has about the same
// work to share out; the panel picks what it is shared over and traces each step.
class ThreadsScene extends PhysicsScene {
    balls: Array<b3BodyId> = new Array<b3BodyId>();

    name(): string {
        return "Threads";
    }

    hint(): string {
        return "2048 balls and capsules stirred by a paddle. Lanes copy each step out to jobs; parallel calls share the world's memory.";
    }

    home(camera: OrbitCamera): void {
        camera.animateTo(v3(0, 0, 0), 30, 0.6, 0.8);
    }

    worldDef(): b3WorldDef {
        let def = super.worldDef();
        def.enableSleep = false;
        return def;
    }

    reset(): void {
        // The lanes hold the world being destroyed
        g_threads.stop(this.worldId);
        super.reset();
        g_threads.detect();
        g_threads.open = true;
        g_threads.clearHistory();
        b3EnableTrace(true);
    }

    leave(): void {
        g_threads.stop(this.worldId);
        g_threads.open = false;
        b3EnableTrace(false);
    }

    build(): void {
        this.crowd = true;
        this.balls = new Array<b3BodyId>();
        addGroundBox(this, PIT_EXTENT + 4);
        let shapeDef = b3DefaultShapeDef();
        // Walls in 2 m blocks, so each face sorts close to the balls beside it
        let wallId = this.addStanding(b3DefaultBodyDef(), INK_GROUND);
        let blocks = <i32>(PIT_EXTENT);
        for (let side = 0; side < 4; side++) {
            let sign: f32 = side % 2 == 0 ? -1.0 : 1.0;
            let alongX = side < 2;
            for (let k = 0; k < blocks; k++) {
                let along: f32 = -PIT_EXTENT + 1.0 + 2.0 * <f32>k;
                let at = alongX ? b3Vec3(along, PIT_WALL, sign * PIT_EXTENT) : b3Vec3(sign * PIT_EXTENT, PIT_WALL, along);
                let transform = b3Transform(at, b3Quat_identity);
                let wall = alongX ? b3MakeTransformedBoxHull(1.0, PIT_WALL, 0.2, transform)
                    : b3MakeTransformedBoxHull(0.2, PIT_WALL, 1.0, transform);
                b3CreateHullShape(wallId, shapeDef, wall.base);
            }
        }

        let paddleDef = b3DefaultBodyDef();
        paddleDef.type = b3_kinematicBody;
        paddleDef.position = b3Vec3(0, 0.6, 0);
        paddleDef.angularVelocity = b3Vec3(0, PIT_SPIN, 0);
        let paddleId = this.addBody(paddleDef, INK_ACCENT);
        b3CreateHullShape(paddleId, shapeDef, b3MakeBoxHull(PIT_EXTENT - 1.4, 0.6, 0.15).base);

        let bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        let pitch: f32 = (2.0 * PIT_EXTENT - 1.4) / <f32>PIT_SIDE;
        let start: f32 = -0.5 * pitch * <f32>(PIT_SIDE - 1);
        let n = 0;
        for (let layer = 0; layer < PIT_LAYERS; layer++) {
            for (let i = 0; i < PIT_SIDE; i++) {
                for (let j = 0; j < PIT_SIDE; j++) {
                    bodyDef.position = b3Vec3(start + pitch * <f32>i + this.random(-0.1, 0.1), 1.6 + 0.9 * <f32>layer,
                        start + pitch * <f32>j + this.random(-0.1, 0.1));
                    bodyDef.rotation = randomAxisAngle(this);
                    let id = this.addBody(bodyDef, bodyInk(n));
                    this.balls.push(id);
                    if (n % 3 == 0) {
                        shapeDef.baseMaterial.rollingResistance = 0.2;
                        b3CreateCapsuleShape(id, shapeDef, b3Capsule(b3Vec3(0, -0.2, 0), b3Vec3(0, 0.2, 0), 0.22));
                    } else {
                        shapeDef.baseMaterial.rollingResistance = 0.1;
                        b3CreateSphereShape(id, shapeDef, b3Sphere(b3Vec3_zero, 0.3));
                    }
                    n++;
                }
            }
        }
    }

    preStep(dt: f32): void {
        if (this.stepCount % 60 == 0) {
            this.dropEscaped();
        }
        g_threads.apply(this.worldId);
        b3TraceBegin();
    }

    // The paddle flings a few over the wall; they drop back in.
    dropEscaped(): void {
        let inside: f32 = PIT_EXTENT - 0.5;
        for (let i = 0; i < this.balls.length; i++) {
            let id = this.balls[i];
            let p = b3Body_GetPosition(id);
            if (Mathf.abs(p.x) < inside && Mathf.abs(p.z) < inside && p.y > -1) {
                continue;
            }
            b3Body_SetTransform(id, b3Vec3(this.random(-4, 4), 5, this.random(-4, 4)), b3Quat_identity);
            b3Body_SetLinearVelocity(id, b3Vec3_zero);
            b3Body_SetAngularVelocity(id, b3Vec3_zero);
        }
    }

    postStep(dt: f32): void {
        b3TraceEnd();
        g_threads.record((b3GetTraceEnd() - b3GetTraceBegin()) * 1000);
    }
}

export function makeScenes(): Array<DemoScene> {
    let list = new Array<DemoScene>();
    list.push(new PyramidScene());
    list.push(new CardHouseScene());
    list.push(new RagdollScene());
    list.push(new RainScene());
    list.push(new HeightFieldScene());
    list.push(new BulletScene());
    list.push(new CharacterScene());
    list.push(new ThreadsScene());
    return list;
}

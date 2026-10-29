// Box3D demo shell: runs a DemoScene drawn through DebugDraw behind the panels
// of demo/shell.rml, and keeps the Demo view model in step with keys and clicks.

import { Layout, Context } from "rive/host";
import { Renderer } from "rive/renderer";
import { Paint, Gradient, GradientStop } from "rive/paint";
import { Path } from "rive/path";
import { Vector } from "rive/vector";
import { KeyboardEvent, PointerEvent, ScrollEvent } from "rive/events";
import { Font } from "rive/text";
import { clock } from "rive/os";
import { formatFixed, formatInt } from "rive/format";
import { OrbitCamera } from "./demo/camera";
import { DebugDraw, INK_BACKGROUND } from "./demo/debug_draw";
import { GpuDraw } from "./demo/gpu_draw";
import { DemoScene, SceneStats } from "./demo/scene";
import { makeScenes } from "./demo/scenes";
import { Shell } from "./demo/shell";
import { g_threads } from "./demo/threads";

const STEP: f32 = 1.0 / 60.0;
const MAX_STEPS_PER_FRAME: i32 = 3;
const CLICK_SLOP: f32 = 4;

const KEY_TAB: i32 = 258;
const KEY_SPACE: i32 = 32;
const KEY_1: i32 = 49;
const KEY_9: i32 = 57;
const KEY_B: i32 = 66;
const KEY_C: i32 = 67;
const KEY_F: i32 = 70;
const KEY_G: i32 = 71;
const KEY_H: i32 = 72;
const KEY_J: i32 = 74;
const KEY_O: i32 = 79;
const KEY_P: i32 = 80;
const KEY_R: i32 = 82;
const KEY_LEFT_SHIFT: i32 = 340;
const KEY_RIGHT_SHIFT: i32 = 344;

const TOOL_PICK: i32 = 0;
const TOOL_SHOOT: i32 = 1;

const DRAG_NONE: i32 = 0;
const DRAG_ORBIT: i32 = 1;
const DRAG_PAN: i32 = 2;
const DRAG_SCENE: i32 = 3;
const DRAG_PINCH: i32 = 4;

export class Main extends Layout {
    context: Context? = null;
    shell: Shell? = null;
    width: f32 = 1280;
    height: f32 = 800;

    scenes: Array<DemoScene> = makeScenes();
    current: i32 = 0;
    camera: OrbitCamera = new OrbitCamera();
    dd: DebugDraw = new DebugDraw();
    stats: SceneStats = new SceneStats();

    paused: bool = false;
    stepOnce: bool = false;
    tool: i32 = TOOL_PICK;
    gridOn: bool = true;
    contactsOn: bool = false;
    jointsOn: bool = true;
    accumulator: f32 = 0;
    stepMs: f64 = 0;
    drawMs: f64 = 0;
    statsAge: f32 = 1;
    logSum: f64 = 0;
    logMax: f64 = 0;
    logCount: i32 = 0;
    logSeconds: i32 = 0;
    drawSum: f64 = 0;
    drawMax: f64 = 0;
    drawCount: i32 = 0;

    // Pointer state.
    drag: i32 = DRAG_NONE;
    pointerId: f64 = -1;
    secondId: f64 = -1;
    pressX: f32 = 0;
    pressY: f32 = 0;
    lastX: f32 = 0;
    lastY: f32 = 0;
    secondX: f32 = 0;
    secondY: f32 = 0;
    moved: bool = false;
    shift: bool = false;
    orbitVx: f32 = 0;
    orbitVy: f32 = 0;

    path: Path = new Path();
    background: Paint = new Paint();

    override init(context: Context): bool {
        this.context = context;
        g_threads.probe();
        let font = context.font("InterVariable");
        if (font != null) {
            this.dd.setFont(light(font));
        } else {
            context.log("box3d demo: InterVariable font missing, labels off");
        }
        // VECTOR_DRAW keeps the 2D painter.
        if (!isDefined(VECTOR_DRAW)) {
            this.dd.gpu = GpuDraw.create(context);
        }
        let start = 0;
        if (isDefined(START_SCENE)) {
            start = START_SCENE - 1;
        }
        this.openScene(start, true);
        return true;
    }

    // The view model can arrive after init.
    private bindShell(): void {
        if (this.shell != null) {
            return;
        }
        let vm = this.context!.viewModel();
        if (vm == null) {
            return;
        }
        let names = new Array<string>();
        for (let i = 0; i < this.scenes.length; i++) {
            names.push(this.scenes[i].name());
        }
        let shell = new Shell(vm, names);
        shell.stepOnce.addListenerWith(this, Main.onStepOnce);
        shell.reset.addListenerWith(this, Main.onReset);
        this.shell = shell;
        this.syncShell();
    }

    private syncShell(): void {
        let shell = this.shell;
        if (shell == null) {
            return;
        }
        shell.scene.value = <f64>this.current;
        shell.sceneTitle.value = this.scenes[this.current].name();
        shell.description.value = this.scenes[this.current].hint();
        shell.fit(this.width, this.height);
        shell.paused.value = this.paused;
        shell.tool.value = <f64>this.tool;
    }

    static onStepOnce(self: Main): void {
        self.setPaused(true);
        self.stepOnce = true;
    }

    static onReset(self: Main): void {
        self.scenes[self.current].reset();
        self.statsAge = 1;
    }

    private setPaused(paused: bool): void {
        this.paused = paused;
        this.syncShell();
    }

    private setTool(tool: i32): void {
        this.tool = tool;
        this.syncShell();
    }

    private openScene(index: i32, snap: bool): void {
        if (index < 0 || index >= this.scenes.length) {
            return;
        }
        if (index != this.current) {
            this.scenes[this.current].leave();
        }
        this.current = index;
        this.drag = DRAG_NONE;
        let scene = this.scenes[index];
        scene.reset();
        scene.home(this.camera);
        if (snap) {
            this.camera.snap();
        }
        this.syncShell();
        this.statsAge = 1;
    }

    // Clicks on the panels land in the view model; follow them.
    private followShell(shell: Shell): void {
        let scene = <i32>shell.scene.value;
        if (scene != this.current) {
            this.openScene(scene, false);
        }
        this.paused = shell.paused.value;
        this.tool = <i32>shell.tool.value;
    }

    override resize(width: f64, height: f64, scale: f64): void {
        this.width = <f32>width;
        this.height = <f32>height;
        this.camera.setViewport(this.width, this.height);
        this.dd.setScale(<f32>scale);
        this.syncShell();
        let stops = new Array<GradientStop>();
        stops.push(GradientStop(0, 0xFF1D1C19));
        stops.push(GradientStop(1, INK_BACKGROUND));
        let g = this.background.gradient;
        if (g != null) {
            g.release();
        }
        this.background.gradient = Gradient.radial(Vector.xy(this.width * 0.55, this.height * 0.42),
            Mathf.max(this.width, this.height) * 0.75, stops);
    }

    // ---- simulation -------------------------------------------------------

    override advance(seconds: f64): bool {
        this.bindShell();
        let shell = this.shell;
        if (shell != null) {
            this.followShell(shell);
            shell.syncThreads(g_threads);
            // The bound position is in artboard points, the size in the overlay's unscaled units.
            let c = shell.clear;
            let k = shell.uiScale.value;
            this.camera.frame(<f32>c[0].value, <f32>c[1].value, <f32>(c[2].value * k), <f32>(c[3].value * k));
        }
        let dt = <f32>Math.min(seconds, 0.1);
        let scene = this.scenes[this.current];
        if (!this.paused) {
            this.accumulator += dt;
            let n = 0;
            while (this.accumulator + 1e-5 >= STEP && n < MAX_STEPS_PER_FRAME) {
                this.timedStep(scene);
                this.accumulator -= STEP;
                n++;
            }
            if (n == MAX_STEPS_PER_FRAME) {
                this.accumulator = 0;
            }
        } else if (this.stepOnce) {
            this.timedStep(scene);
        }
        this.stepOnce = false;
        this.camera.advance(dt);
        scene.updateCamera(this.camera);
        this.statsAge += dt;
        if (shell != null && this.statsAge >= 0.25) {
            this.statsAge = 0;
            scene.stats(this.stats);
            shell.showStats(this.stepMs, this.drawMs, this.stats, this.dd.primCount);
        }
        return true;
    }

    private timedStep(scene: DemoScene): void {
        let t0 = clock();
        scene.step(STEP);
        let ms = (clock() - t0) * 1000;
        this.stepMs = this.stepMs == 0 ? ms : this.stepMs * 0.9 + ms * 0.1;
        if (isDefined(STEP_LOG)) {
            this.logStep(ms);
        }
    }

    // Mean and max step over each second, for timing scenes headless.
    private logStep(ms: f64): void {
        this.logSum += ms;
        this.logMax = Math.max(this.logMax, ms);
        this.logCount++;
        if (this.logCount == 60) {
            this.logSeconds++;
            this.context!.log("step " + this.scenes[this.current].name() + " s" + formatInt(<f64>this.logSeconds) +
                " mean " + formatFixed(this.logSum / 60, 3) + " max " + formatFixed(this.logMax, 3));
            this.logSum = 0;
            this.logMax = 0;
            this.logCount = 0;
        }
    }

    // ---- drawing ----------------------------------------------------------

    override draw(renderer: Renderer): void {
        let t0 = clock();
        let path = this.path;
        path.reset();
        path.moveTo(Vector.xy(0, 0));
        path.lineTo(Vector.xy(this.width, 0));
        path.lineTo(Vector.xy(this.width, this.height));
        path.lineTo(Vector.xy(0, this.height));
        path.close();
        renderer.drawPath(path, this.background);

        let dd = this.dd;
        dd.begin(this.camera);
        dd.showContacts = this.contactsOn;
        dd.showJoints = this.jointsOn;
        this.scenes[this.current].draw(dd);
        if (!this.gridOn) {
            dd.showGrid = false;
        }
        dd.end(renderer);
        let ms = (clock() - t0) * 1000;
        this.drawMs = this.drawMs == 0 ? ms : this.drawMs * 0.9 + ms * 0.1;
        if (isDefined(DRAW_LOG)) {
            this.logDraw(ms);
        }
    }

    // Mean and max draw over each 120 frames, for timing the renderers headless with --bench.
    private logDraw(ms: f64): void {
        this.drawSum += ms;
        this.drawMax = Math.max(this.drawMax, ms);
        this.drawCount++;
        if (this.drawCount == 120) {
            this.context!.log("draw " + this.scenes[this.current].name() + " mean " + formatFixed(this.drawSum / 120, 3) +
                " max " + formatFixed(this.drawMax, 3) + " prims " + formatInt(<f64>this.dd.primCount));
            this.drawSum = 0;
            this.drawMax = 0;
            this.drawCount = 0;
        }
    }

    // ---- input ------------------------------------------------------------

    override pointerDown(event: PointerEvent): void {
        let shell = this.shell;
        if (shell != null && shell.takeUiPress()) {
            return;
        }
        let x = event.position.x;
        let y = event.position.y;
        event.hit();
        if (this.drag != DRAG_NONE && event.id != this.pointerId) {
            // A second finger turns any drag into a pinch.
            if (this.drag == DRAG_SCENE) {
                this.scenes[this.current].pointerUp(this.camera.ray(this.lastX, this.lastY));
            }
            this.secondId = event.id;
            this.secondX = x;
            this.secondY = y;
            this.drag = DRAG_PINCH;
            return;
        }
        this.pointerId = event.id;
        this.pressX = x;
        this.pressY = y;
        this.lastX = x;
        this.lastY = y;
        this.moved = false;
        this.orbitVx = 0;
        this.orbitVy = 0;
        this.camera.yawVelocity = 0;
        this.camera.pitchVelocity = 0;
        if (this.shift) {
            this.drag = DRAG_PAN;
        } else if (this.tool == TOOL_PICK && this.scenes[this.current].pointerDown(this.camera.ray(x, y))) {
            this.drag = DRAG_SCENE;
        } else {
            this.drag = DRAG_ORBIT;
        }
    }

    override pointerMove(event: PointerEvent): void {
        let x = event.position.x;
        let y = event.position.y;
        if (this.drag == DRAG_NONE) {
            return;
        }
        if (this.drag == DRAG_PINCH) {
            let before = Mathf.sqrt((this.lastX - this.secondX) * (this.lastX - this.secondX) +
                (this.lastY - this.secondY) * (this.lastY - this.secondY));
            if (event.id == this.secondId) {
                this.secondX = x;
                this.secondY = y;
            } else {
                this.lastX = x;
                this.lastY = y;
            }
            let after = Mathf.sqrt((this.lastX - this.secondX) * (this.lastX - this.secondX) +
                (this.lastY - this.secondY) * (this.lastY - this.secondY));
            if (before > 1 && after > 1) {
                this.camera.zoom(before / after);
            }
            return;
        }
        if (event.id != this.pointerId) {
            return;
        }
        let dx = x - this.lastX;
        let dy = y - this.lastY;
        this.lastX = x;
        this.lastY = y;
        if (Mathf.abs(x - this.pressX) > CLICK_SLOP || Mathf.abs(y - this.pressY) > CLICK_SLOP) {
            this.moved = true;
        }
        if (this.drag == DRAG_ORBIT) {
            this.camera.orbit(dx, dy);
            this.orbitVx = this.orbitVx * 0.5 + dx * 0.5;
            this.orbitVy = this.orbitVy * 0.5 + dy * 0.5;
        } else if (this.drag == DRAG_PAN) {
            this.camera.pan(dx, dy);
        } else if (this.drag == DRAG_SCENE) {
            this.scenes[this.current].pointerMove(this.camera.ray(x, y));
        }
    }

    override pointerUp(event: PointerEvent): void {
        let x = event.position.x;
        let y = event.position.y;
        if (this.drag == DRAG_PINCH) {
            if (event.id == this.secondId || event.id == this.pointerId) {
                this.drag = DRAG_NONE;
            }
            return;
        }
        if (this.drag == DRAG_NONE || event.id != this.pointerId) {
            return;
        }
        if (this.drag == DRAG_SCENE) {
            this.scenes[this.current].pointerUp(this.camera.ray(x, y));
        } else if (this.drag == DRAG_ORBIT) {
            if (!this.moved && this.tool == TOOL_SHOOT) {
                this.scenes[this.current].shoot(this.camera.ray(x, y));
            } else if (this.moved) {
                // Fling: the last moves per frame become radians per second.
                this.camera.yawVelocity = -this.orbitVx * 0.008 * 60;
                this.camera.pitchVelocity = this.orbitVy * 0.008 * 60;
            }
        }
        this.drag = DRAG_NONE;
    }

    override pointerExit(event: PointerEvent): void {
        if (this.drag != DRAG_NONE && event.id == this.pointerId) {
            this.pointerUp(event);
        }
    }

    override pointerScroll(event: ScrollEvent): void {
        event.hit();
        let dy = event.delta.y;
        let dx = event.delta.x;
        if (event.precise) {
            this.camera.zoom(Mathf.exp(-dy * 0.004));
            this.camera.orbit(dx * 0.6, 0);
        } else {
            this.camera.zoom(Mathf.exp(-dy * 0.0025));
        }
    }

    override keyboardEvent(event: KeyboardEvent): bool {
        let key = <i32>event.key;
        let scene = this.scenes[this.current];
        if (key == KEY_LEFT_SHIFT || key == KEY_RIGHT_SHIFT) {
            this.shift = event.phase != "up";
            if (event.phase == "up") {
                scene.keyUp(key);
            } else if (event.phase == "down") {
                scene.keyDown(key);
            }
            return true;
        }
        if (event.phase == "up") {
            return scene.keyUp(key);
        }
        if (event.phase == "repeat") {
            return true;
        }
        if (key == KEY_P) {
            this.setPaused(!this.paused);
        } else if (key == KEY_O) {
            Main.onStepOnce(this);
        } else if (key == KEY_R) {
            Main.onReset(this);
        } else if (key == KEY_TAB) {
            this.setTool(this.tool == TOOL_PICK ? TOOL_SHOOT : TOOL_PICK);
        } else if (key >= KEY_1 && key <= KEY_9) {
            this.openScene(key - KEY_1, false);
        } else if (key == KEY_F) {
            scene.home(this.camera);
        } else if (key == KEY_G) {
            this.gridOn = !this.gridOn;
        } else if (key == KEY_H) {
            let gpu = this.dd.gpu;
            if (gpu != null) {
                this.context!.log("shadows " + gpu.cycleShadows());
            }
        } else if (key == KEY_C) {
            this.contactsOn = !this.contactsOn;
        } else if (key == KEY_J) {
            this.jointsOn = !this.jointsOn;
        } else if (scene.keyDown(key)) {
            return true;
        } else if (key == KEY_SPACE || key == KEY_B) {
            scene.shoot(this.camera.ray(this.camera.cx, this.camera.cy));
        } else {
            return false;
        }
        return true;
    }
}

// The light weight, with tabular figures, for debug draw labels.
function light(font: Font): Font {
    let axes = new Map<string, f32>();
    axes.set("wght", 300);
    let features = new Map<string, u32>();
    features.set("tnum", 1);
    return font.withOptions(axes, features);
}

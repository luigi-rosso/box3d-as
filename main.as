// Box3D demo shell: scene picker, transport, stats overlay and camera input
// around a DemoScene drawn through DebugDraw.

import { Layout, Context } from "rive/host";
import { Renderer } from "rive/renderer";
import { Paint, PaintStyle, StrokeJoin, StrokeCap, Gradient, GradientStop } from "rive/paint";
import { Path } from "rive/path";
import { Vector } from "rive/vector";
import { KeyboardEvent, PointerEvent, ScrollEvent } from "rive/events";
import { Font, TextSizing } from "rive/text";
import { clock } from "rive/os";
import { formatFixed, formatInt } from "rive/format";
import { OrbitCamera } from "./demo/camera";
import { DebugDraw, INK_BACKGROUND } from "./demo/debug_draw";
import { DemoScene, SceneStats, PROFILE_COUNT } from "./demo/scene";
import { makeScenes } from "./demo/scenes";
import { UiText, Rect, addRoundRect, fontAt, textStyle, UI_INK, UI_DIM, UI_FAINT, UI_HAIRLINE, UI_HAIRLINE_HOT,
    UI_INVERTED_TEXT, UI_ACCENT } from "./demo/ui";

const STEP: f32 = 1.0 / 60.0;
const MAX_STEPS_PER_FRAME: i32 = 3;
const MARGIN: f32 = 28;
const ROW_HEIGHT: f32 = 30;
const LIST_WIDTH: f32 = 196;
const BUTTON: f32 = 34;
const TOOL_WIDTH: f32 = 64;
const CLICK_SLOP: f32 = 4;

const KEY_TAB: i32 = 258;
const KEY_SPACE: i32 = 32;
const KEY_1: i32 = 49;
const KEY_9: i32 = 57;
const KEY_B: i32 = 66;
const KEY_C: i32 = 67;
const KEY_F: i32 = 70;
const KEY_G: i32 = 71;
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

const BTN_PAUSE: i32 = 0;
const BTN_STEP: i32 = 1;
const BTN_RESET: i32 = 2;
const BTN_PICK: i32 = 3;
const BTN_SHOOT: i32 = 4;
const BUTTON_COUNT: i32 = 5;

const STATS_WIDTH: f32 = 168;
const PROFILE_LABELS: StaticArray<string> = ["PAIRS", "COLLIDE", "SOLVER", "UPDATE", "CCD", "ISLANDS", "OTHER"];
// One tone per profile part, light to dark, for the bar and the swatches.
const PROFILE_TONES: StaticArray<u32> = [0xFFEDE8E1, 0xFFC9C2B8, 0xFFA59E95, 0xFF877F75, 0xFF6E6860, 0xFF58534D,
    0xFF45413C];

function formatMs(ms: f64): string {
    return formatFixed(ms, ms < 0.1 ? 3 : 2) + " ms";
}

export class Main extends Layout {
    context: Context? = null;
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
    hoverRow: i32 = -1;
    hoverButton: i32 = -1;

    // UI.
    rows: Array<Rect> = new Array<Rect>();
    buttons: Array<Rect> = new Array<Rect>();
    path: Path = new Path();
    fill: Paint = new Paint();
    stroke: Paint = new Paint();
    icon: Paint = new Paint();
    background: Paint = new Paint();
    scrim: Paint = new Paint();
    scrimRight: Paint = new Paint();
    fontReady: bool = false;
    brand: UiText? = null;
    subtitle: UiText? = null;
    hint: UiText? = null;
    help: UiText? = null;
    pausedTag: UiText? = null;
    rowIndex: Array<UiText> = new Array<UiText>();
    rowName: Array<UiText> = new Array<UiText>();
    statLabel: Array<UiText> = new Array<UiText>();
    statValue: Array<UiText> = new Array<UiText>();
    toolLabel: Array<UiText> = new Array<UiText>();
    profileTitle: UiText? = null;
    profileTotal: UiText? = null;
    profileLabel: Array<UiText> = new Array<UiText>();
    profileValue: Array<UiText> = new Array<UiText>();

    override init(context: Context): bool {
        this.context = context;
        this.stroke.style = PaintStyle.stroke;
        this.stroke.thickness = 1;
        this.icon.style = PaintStyle.stroke;
        this.icon.thickness = 1.25;
        this.icon.join = StrokeJoin.round;
        this.icon.cap = StrokeCap.round;
        let font = context.font("InterVariable");
        if (font != null) {
            this.setupText(font);
        } else {
            context.log("box3d demo: InterVariable font missing, drawing without text");
        }
        for (let i = 0; i < BUTTON_COUNT; i++) {
            this.buttons.push(new Rect());
        }
        for (let i = 0; i < this.scenes.length; i++) {
            this.rows.push(new Rect());
        }
        let start = 0;
        if (isDefined(START_SCENE)) {
            start = START_SCENE - 1;
        }
        this.openScene(start, true);
        return true;
    }

    private setupText(font: Font): void {
        let light = fontAt(font, 300);
        let regular = fontAt(font, 420);
        this.dd.setFont(light);
        this.brand = new UiText(textStyle(light, 15, 4.5)).set("BOX3D");
        this.subtitle = new UiText(textStyle(regular, 9.5, 1.8)).set("ANIMASCRIPT PORT  /  DEBUG DRAW");
        let hintStyle = textStyle(light, 11.5, 0.1);
        hintStyle.lineHeight = 17;
        this.hint = new UiText(hintStyle);
        this.help = new UiText(textStyle(light, 11, 0.2)).set(
            "Drag orbit   Shift drag pan   Scroll zoom   P pause   O step   R reset   Tab tool   C contacts   J joints   1-7 scene");
        this.pausedTag = new UiText(textStyle(regular, 9.5, 2)).set("PAUSED");
        let indexStyle = textStyle(regular, 9.5, 1);
        let nameStyle = textStyle(light, 13, 0.2);
        for (let i = 0; i < this.scenes.length; i++) {
            this.rowIndex.push(new UiText(indexStyle).set(formatInt(<f64>(i + 1), 2, true)));
            this.rowName.push(new UiText(nameStyle).set(this.scenes[i].name()));
        }
        let labelStyle = textStyle(regular, 9.5, 1.6);
        let valueStyle = textStyle(light, 13, 0.2);
        let labels = ["STEP", "DRAW", "BODIES", "AWAKE", "CONTACTS", "JOINTS", "ISLANDS", "FACES"];
        for (let i = 0; i < labels.length; i++) {
            this.statLabel.push(new UiText(labelStyle).set(labels[i]));
            this.statValue.push(new UiText(valueStyle).set("0"));
        }
        let smallLabel = textStyle(regular, 8.5, 1.4);
        let smallValue = textStyle(light, 11, 0.2);
        this.profileTitle = new UiText(labelStyle).set("B3 PROFILE");
        this.profileTotal = new UiText(valueStyle).set("0");
        for (let i = 0; i < PROFILE_COUNT; i++) {
            this.profileLabel.push(new UiText(smallLabel).set(PROFILE_LABELS[i]));
            this.profileValue.push(new UiText(smallValue).set("0"));
        }
        let toolStyle = textStyle(light, 12, 0.3);
        this.toolLabel.push(new UiText(toolStyle).set("Pick"));
        this.toolLabel.push(new UiText(toolStyle).set("Shoot"));
        this.fontReady = true;
    }

    private openScene(index: i32, snap: bool): void {
        if (index < 0 || index >= this.scenes.length) {
            return;
        }
        this.current = index;
        this.drag = DRAG_NONE;
        let scene = this.scenes[index];
        scene.reset();
        scene.home(this.camera);
        if (snap) {
            this.camera.snap();
        }
        let hint = this.hint;
        if (hint != null) {
            hint.text.sizing = TextSizing.autoHeight;
            hint.text.maxWidth = LIST_WIDTH + 24;
            hint.set(scene.hint());
        }
        this.statsAge = 1;
    }

    override resize(width: f64, height: f64, scale: f64): void {
        this.width = <f32>width;
        this.height = <f32>height;
        this.camera.setViewport(this.width, this.height);
        let stops = new Array<GradientStop>();
        stops.push(GradientStop(0, 0xFF1D1C19));
        stops.push(GradientStop(1, INK_BACKGROUND));
        let g = this.background.gradient;
        if (g != null) {
            g.release();
        }
        this.background.gradient = Gradient.radial(Vector.xy(this.width * 0.55, this.height * 0.42),
            Mathf.max(this.width, this.height) * 0.75, stops);
        // Keeps the scene list legible over geometry.
        let scrimStops = new Array<GradientStop>();
        scrimStops.push(GradientStop(0, 0xB8151412));
        scrimStops.push(GradientStop(1, 0x00151412));
        let sg = this.scrim.gradient;
        if (sg != null) {
            sg.release();
        }
        this.scrim.gradient = Gradient.linear(Vector.xy(0, 0), Vector.xy(MARGIN + LIST_WIDTH + 140, 0), scrimStops);
        let rg = this.scrimRight.gradient;
        if (rg != null) {
            rg.release();
        }
        this.scrimRight.gradient = Gradient.linear(Vector.xy(this.width, 0),
            Vector.xy(this.width - MARGIN - STATS_WIDTH - 120, 0), scrimStops);
        this.layoutUi();
    }

    private layoutUi(): void {
        let y = MARGIN + 62;
        for (let i = 0; i < this.rows.length; i++) {
            this.rows[i].setTo(MARGIN - 10, y + <f32>i * ROW_HEIGHT, LIST_WIDTH, ROW_HEIGHT - 4);
        }
        let by = this.height - MARGIN - BUTTON;
        let x = MARGIN;
        for (let i = 0; i < 3; i++) {
            this.buttons[i].setTo(x, by, BUTTON, BUTTON);
            x += BUTTON + 8;
        }
        x += 14;
        this.buttons[BTN_PICK].setTo(x, by, TOOL_WIDTH, BUTTON);
        this.buttons[BTN_SHOOT].setTo(x + TOOL_WIDTH, by, TOOL_WIDTH, BUTTON);
    }

    // ---- simulation -------------------------------------------------------

    override advance(seconds: f64): bool {
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

    private refreshStats(scene: DemoScene): void {
        scene.stats(this.stats);
        if (!this.fontReady) {
            return;
        }
        let s = this.stats;
        this.statValue[0].set(formatMs(this.stepMs));
        this.statValue[1].set(formatMs(this.drawMs));
        this.statValue[2].set(formatInt(<f64>s.bodies));
        this.statValue[3].set(formatInt(<f64>s.awake));
        this.statValue[4].set(formatInt(<f64>s.contacts));
        this.statValue[5].set(formatInt(<f64>s.joints));
        this.statValue[6].set(formatInt(<f64>s.islands));
        this.statValue[7].set(formatInt(<f64>this.dd.primCount));
        this.profileTotal!.set(formatMs(<f64>s.profileStep));
        for (let i = 0; i < PROFILE_COUNT; i++) {
            this.profileValue[i].set(formatFixed(<f64>s.profile[i], 2));
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

        if (this.statsAge >= 0.25) {
            this.statsAge = 0;
            this.refreshStats(this.scenes[this.current]);
        }
        path.reset();
        path.moveTo(Vector.xy(0, 0));
        path.lineTo(Vector.xy(MARGIN + LIST_WIDTH + 140, 0));
        path.lineTo(Vector.xy(MARGIN + LIST_WIDTH + 140, this.height));
        path.lineTo(Vector.xy(0, this.height));
        path.close();
        renderer.drawPath(path, this.scrim);
        let edge = this.width - MARGIN - STATS_WIDTH - 120;
        path.reset();
        path.moveTo(Vector.xy(edge, 0));
        path.lineTo(Vector.xy(this.width, 0));
        path.lineTo(Vector.xy(this.width, this.height));
        path.lineTo(Vector.xy(edge, this.height));
        path.close();
        renderer.drawPath(path, this.scrimRight);
        if (this.fontReady) {
            this.drawPanel(renderer);
            this.drawStats(renderer);
        }
        this.drawTransport(renderer);
    }

    private drawPanel(renderer: Renderer): void {
        let fill = this.fill;
        fill.color = UI_INK;
        this.brand!.draw(renderer, fill, MARGIN, MARGIN, 0);
        fill.color = UI_DIM;
        this.subtitle!.draw(renderer, fill, MARGIN, MARGIN + 24, 0);

        let path = this.path;
        for (let i = 0; i < this.rows.length; i++) {
            let r = this.rows[i];
            let active = i == this.current;
            if (active || i == this.hoverRow) {
                path.reset();
                addRoundRect(path, r.x, r.y, r.w, r.h, 4);
                if (active) {
                    fill.color = UI_INK;
                    renderer.drawPath(path, fill);
                } else {
                    this.stroke.color = UI_HAIRLINE_HOT;
                    renderer.drawPath(path, this.stroke);
                }
            }
            let ty = r.y + (r.h - 16) * 0.5;
            fill.color = active ? UI_INVERTED_TEXT : UI_FAINT;
            this.rowIndex[i].draw(renderer, fill, r.x + 10, ty + 2, 0);
            fill.color = active ? UI_INVERTED_TEXT : UI_DIM;
            this.rowName[i].draw(renderer, fill, r.x + 36, ty, 0);
        }
        let last = this.rows[this.rows.length - 1];
        let hy = last.y + last.h + 18;
        this.stroke.color = UI_HAIRLINE;
        path.reset();
        path.moveTo(Vector.xy(MARGIN, hy));
        path.lineTo(Vector.xy(MARGIN + 24, hy));
        renderer.drawPath(path, this.stroke);
        fill.color = UI_DIM;
        this.hint!.draw(renderer, fill, MARGIN, hy + 14, 0);

        // Hidden when it would run into the transport.
        let help = this.help!;
        let tools = this.buttons[BTN_SHOOT];
        if (this.width - MARGIN - help.width > tools.x + tools.w + 140) {
            fill.color = UI_FAINT;
            help.draw(renderer, fill, this.width - MARGIN, this.height - MARGIN - 14, 1);
        }
    }

    private drawStats(renderer: Renderer): void {
        let fill = this.fill;
        let right = this.width - MARGIN;
        let left = right - STATS_WIDTH;
        let y = MARGIN + 2;
        for (let i = 0; i < this.statLabel.length; i++) {
            fill.color = UI_FAINT;
            this.statLabel[i].draw(renderer, fill, left, y + 3, 0);
            fill.color = i < 2 ? UI_INK : UI_DIM;
            this.statValue[i].draw(renderer, fill, right, y, 1);
            y += 24;
            if (i == 1) {
                this.hairline(renderer, left, right, y - 4);
                y += 6;
            }
        }
        this.hairline(renderer, left, right, y - 4);
        y += 10;
        this.drawProfile(renderer, left, right, y);
    }

    // b3Profile as a stacked bar over the step, then one row per part.
    private drawProfile(renderer: Renderer, left: f32, right: f32, y: f32): void {
        let fill = this.fill;
        let s = this.stats;
        fill.color = UI_FAINT;
        this.profileTitle!.draw(renderer, fill, left, y + 3, 0);
        fill.color = UI_DIM;
        this.profileTotal!.draw(renderer, fill, right, y, 1);
        y += 24;
        let total: f32 = 0;
        for (let i = 0; i < PROFILE_COUNT; i++) {
            total += s.profile[i];
        }
        let path = this.path;
        let x = left;
        for (let i = 0; i < PROFILE_COUNT && total > 0; i++) {
            let w = STATS_WIDTH * s.profile[i] / total;
            if (w > 0.25) {
                path.reset();
                path.moveTo(Vector.xy(x, y));
                path.lineTo(Vector.xy(x + w, y));
                path.lineTo(Vector.xy(x + w, y + 3));
                path.lineTo(Vector.xy(x, y + 3));
                path.close();
                fill.color = PROFILE_TONES[i];
                renderer.drawPath(path, fill);
            }
            x += w;
        }
        y += 12;
        for (let i = 0; i < PROFILE_COUNT; i++) {
            path.reset();
            addRoundRect(path, left, y + 4, 6, 6, 1.5);
            fill.color = PROFILE_TONES[i];
            renderer.drawPath(path, fill);
            fill.color = UI_FAINT;
            this.profileLabel[i].draw(renderer, fill, left + 14, y + 3, 0);
            fill.color = UI_DIM;
            this.profileValue[i].draw(renderer, fill, right, y, 1);
            y += 18;
        }
    }

    private hairline(renderer: Renderer, left: f32, right: f32, y: f32): void {
        let path = this.path;
        path.reset();
        path.moveTo(Vector.xy(left, y));
        path.lineTo(Vector.xy(right, y));
        this.stroke.color = UI_HAIRLINE;
        renderer.drawPath(path, this.stroke);
    }

    private drawTransport(renderer: Renderer): void {
        let path = this.path;
        for (let i = 0; i < BUTTON_COUNT; i++) {
            let r = this.buttons[i];
            let active = (i == BTN_PAUSE && this.paused) || (i == BTN_PICK && this.tool == TOOL_PICK) ||
                (i == BTN_SHOOT && this.tool == TOOL_SHOOT);
            let hot = i == this.hoverButton;
            path.reset();
            addRoundRect(path, r.x, r.y, r.w, r.h, 4);
            if (active) {
                this.fill.color = UI_INK;
                renderer.drawPath(path, this.fill);
            } else if (i < BTN_PICK) {
                this.stroke.color = hot ? UI_HAIRLINE_HOT : UI_HAIRLINE;
                renderer.drawPath(path, this.stroke);
            }
            let ink = active ? UI_INVERTED_TEXT : (hot ? UI_INK : UI_DIM);
            if (i >= BTN_PICK) {
                if (this.fontReady) {
                    this.fill.color = ink;
                    let t = this.toolLabel[i - BTN_PICK];
                    t.draw(renderer, this.fill, r.x + r.w * 0.5, r.y + (r.h - 15) * 0.5, 0.5);
                }
            } else {
                this.drawIcon(renderer, i, r.x + r.w * 0.5, r.y + r.h * 0.5, ink);
            }
        }
        // The tool switch is one outlined group.
        let pick = this.buttons[BTN_PICK];
        path.reset();
        addRoundRect(path, pick.x, pick.y, 2 * TOOL_WIDTH, BUTTON, 4);
        this.stroke.color = UI_HAIRLINE;
        renderer.drawPath(path, this.stroke);
        if (this.paused && this.fontReady) {
            let r = this.buttons[BTN_SHOOT];
            this.fill.color = UI_ACCENT;
            this.pausedTag!.draw(renderer, this.fill, r.x + r.w + 18, r.y + 12, 0);
        }
    }

    private drawIcon(renderer: Renderer, which: i32, cx: f32, cy: f32, ink: u32): void {
        let path = this.path;
        path.reset();
        if (which == BTN_PAUSE) {
            if (this.paused) {
                path.moveTo(Vector.xy(cx - 3.5, cy - 5.5));
                path.lineTo(Vector.xy(cx + 5.5, cy));
                path.lineTo(Vector.xy(cx - 3.5, cy + 5.5));
                path.close();
            } else {
                path.moveTo(Vector.xy(cx - 3, cy - 5.5));
                path.lineTo(Vector.xy(cx - 3, cy + 5.5));
                path.moveTo(Vector.xy(cx + 3, cy - 5.5));
                path.lineTo(Vector.xy(cx + 3, cy + 5.5));
            }
        } else if (which == BTN_STEP) {
            path.moveTo(Vector.xy(cx - 5, cy - 5));
            path.lineTo(Vector.xy(cx + 2, cy));
            path.lineTo(Vector.xy(cx - 5, cy + 5));
            path.close();
            path.moveTo(Vector.xy(cx + 5, cy - 5.5));
            path.lineTo(Vector.xy(cx + 5, cy + 5.5));
        } else {
            // From the top round the left and bottom, up to an open corner head at the upper right.
            let r: f32 = 6.0;
            let a0: f32 = -1.5708;
            let sweep: f32 = 5.63;
            let steps = 24;
            for (let k = 0; k <= steps; k++) {
                let a: f32 = a0 - sweep * <f32>k / <f32>steps;
                let p = Vector.xy(cx + Mathf.cos(a) * r, cy + Mathf.sin(a) * r);
                if (k == 0) {
                    path.moveTo(p);
                } else {
                    path.lineTo(p);
                }
            }
            let a1 = a0 - sweep;
            let tipX = cx + Mathf.cos(a1) * r;
            let tipY = cy + Mathf.sin(a1) * r;
            // The arms run back along the arc 45 degrees off it, turned clockwise so the inner one clears the curve.
            let bx = -Mathf.sin(a1);
            let by = Mathf.cos(a1);
            let arm: f32 = 0.45 * r;
            let inner: f32 = 0.785 + 0.35;
            let outer: f32 = -0.785 + 0.35;
            path.moveTo(Vector.xy(tipX + (bx * Mathf.cos(inner) - by * Mathf.sin(inner)) * arm,
                tipY + (bx * Mathf.sin(inner) + by * Mathf.cos(inner)) * arm));
            path.lineTo(Vector.xy(tipX, tipY));
            path.lineTo(Vector.xy(tipX + (bx * Mathf.cos(outer) - by * Mathf.sin(outer)) * arm,
                tipY + (bx * Mathf.sin(outer) + by * Mathf.cos(outer)) * arm));
        }
        this.icon.color = ink;
        renderer.drawPath(path, this.icon);
    }

    // ---- input ------------------------------------------------------------

    private hitButton(x: f32, y: f32): i32 {
        for (let i = 0; i < BUTTON_COUNT; i++) {
            if (this.buttons[i].contains(x, y)) {
                return i;
            }
        }
        return -1;
    }

    private hitRow(x: f32, y: f32): i32 {
        for (let i = 0; i < this.rows.length; i++) {
            if (this.rows[i].contains(x, y)) {
                return i;
            }
        }
        return -1;
    }

    private press(button: i32): void {
        if (button == BTN_PAUSE) {
            this.paused = !this.paused;
        } else if (button == BTN_STEP) {
            this.paused = true;
            this.stepOnce = true;
        } else if (button == BTN_RESET) {
            this.scenes[this.current].reset();
            this.statsAge = 1;
        } else if (button == BTN_PICK) {
            this.tool = TOOL_PICK;
        } else if (button == BTN_SHOOT) {
            this.tool = TOOL_SHOOT;
        }
    }

    override pointerDown(event: PointerEvent): void {
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
        let button = this.hitButton(x, y);
        if (button >= 0) {
            this.press(button);
            return;
        }
        let row = this.hitRow(x, y);
        if (row >= 0) {
            if (row != this.current) {
                this.openScene(row, false);
            }
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
            this.hoverButton = this.hitButton(x, y);
            this.hoverRow = this.hitRow(x, y);
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
        this.hoverButton = -1;
        this.hoverRow = -1;
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
            this.press(BTN_PAUSE);
        } else if (key == KEY_O) {
            this.press(BTN_STEP);
        } else if (key == KEY_R) {
            this.press(BTN_RESET);
        } else if (key == KEY_TAB) {
            this.tool = this.tool == TOOL_PICK ? TOOL_SHOOT : TOOL_PICK;
        } else if (key >= KEY_1 && key <= KEY_9) {
            this.openScene(key - KEY_1, false);
        } else if (key == KEY_F) {
            scene.home(this.camera);
        } else if (key == KEY_G) {
            this.gridOn = !this.gridOn;
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

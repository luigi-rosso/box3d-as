// The Threads panel's graphs: the latest step's spans on each thread, and the step time over the last few seconds.
// The scene records both; this script sits in the panel's layout box and only draws.

import { Layout, Context } from "rive/host";
import { Renderer } from "rive/renderer";
import { Paint, PaintStyle } from "rive/paint";
import { Path } from "rive/path";
import { Vector } from "rive/vector";
import { Mat2D } from "rive/mat2d";
import { Color } from "rive/color";
import { Font, Text, TextRunStyle } from "rive/text";
import { formatFixed } from "rive/format";
import { BACKEND_LANES, BACKEND_NAMES, BACKEND_PARALLEL, BACKEND_SERIAL, HISTORY, g_threads } from "./threads";
import {
    b3GetTraceBegin,
    b3GetTraceEnd,
    b3GetTraceRow,
    b3_traceBullets,
    b3_traceCollide,
    b3_traceCopy,
    b3_traceFinalize,
    b3_tracePairs,
    b3_traceSensors,
    b3_traceTree,
} from "../lib/trace";

const INK: Color = 0xFFEDE8E1;
const DIM: Color = 0xFF8C867E;
const FAINT: Color = 0xFF5A5650;
const HAIRLINE: Color = 0xFF3B3833;
const TRACK: Color = 0xFF1F1E1B;
const ACCENT: Color = 0xFFE0875A;

// The profile panel's tones, so a span reads as the part of the profile it is
const GROUP_PAIRS: i32 = 0;
const GROUP_COLLIDE: i32 = 1;
const GROUP_SOLVER: i32 = 2;
const GROUP_FINALIZE: i32 = 3;
const GROUP_TREE: i32 = 4;
const GROUP_OTHER: i32 = 5;
const GROUP_COPY: i32 = 6;
const GROUP_COUNT: i32 = 7;
const GROUP_TONES: StaticArray<u32> = [0xFFEDE8E1, 0xFFC9C2B8, 0xFFA59E95, 0xFF877F75, 0xFF6E6860, 0xFF45413C, 0xFFE0875A];
const GROUP_NAMES: StaticArray<string> = ["PAIRS", "COLLIDE", "SOLVER", "FINALIZE", "TREE", "OTHER", "COPY"];

const AXIS_STEPS: StaticArray<f64> = [0.05, 0.1, 0.2, 0.25, 0.5, 1, 2, 2.5, 5, 10, 20, 50];
const SCALE_STEPS: StaticArray<f64> = [0.25, 0.5, 1, 2, 3, 4, 5, 6, 8, 10, 15, 20, 30, 40, 50, 100, 200];

const LABEL_WIDTH: f32 = 46;
const MAX_ROWS: i32 = 9;

function groupOf(kind: i32): i32 {
    if (kind == b3_tracePairs) {
        return GROUP_PAIRS;
    }
    if (kind == b3_traceCollide) {
        return GROUP_COLLIDE;
    }
    if (kind == b3_traceTree) {
        return GROUP_TREE;
    }
    if (kind == b3_traceFinalize) {
        return GROUP_FINALIZE;
    }
    if (kind == b3_traceCopy) {
        return GROUP_COPY;
    }
    if (kind == b3_traceBullets || kind == b3_traceSensors) {
        return GROUP_OTHER;
    }
    return GROUP_SOLVER;
}

// The smallest step of steps at least value
function niceAtLeast(steps: StaticArray<f64>, value: f64): f64 {
    for (let i = 0; i < steps.length; i++) {
        if (steps[i] >= value) {
            return steps[i];
        }
    }
    return steps[steps.length - 1];
}

function rect(path: Path, x: f32, y: f32, w: f32, h: f32): void {
    path.moveTo(Vector.xy(x, y));
    path.lineTo(Vector.xy(x + w, y));
    path.lineTo(Vector.xy(x + w, y + h));
    path.lineTo(Vector.xy(x, y + h));
    path.close();
}

// Text drawn at a spot, laid out again only when its string changes
class Label {
    text: Text = new Text();
    value: string = "";
    width: f32 = 0;
    top: f32 = 0;
}

class Labels {
    pool: Array<Label> = new Array<Label>();
    used: i32 = 0;
    paint: Paint = new Paint();

    // Draws value with its left edge at x, or its right edge when right is set, and its glyphs' top at y; returns its
    // width.
    draw(renderer: Renderer, style: TextRunStyle, value: string, x: f32, y: f32, color: Color, right: bool): f32 {
        if (this.used == this.pool.length) {
            this.pool.push(new Label());
        }
        let l = this.pool[this.used++];
        if (l.value != value) {
            l.value = value;
            l.text.clear();
            l.text.append(value, style);
            let b = l.text.bounds();
            l.width = b.v1.x - b.v0.x;
            l.top = b.v0.y;
        }
        renderer.save();
        renderer.transform(Mat2D.withTranslation(right ? x - l.width : x, y - l.top));
        this.paint.color = color;
        l.text.draw(renderer, this.paint);
        renderer.restore();
        return l.width;
    }
}

export class ThreadsGraph extends Layout {
    width: f32 = 0;
    height: f32 = 0;
    // The timeline's span in ms, easing toward the latest steps
    axisMs: f64 = 0;
    labels: Labels = new Labels();
    caption: TextRunStyle? = null;
    small: TextRunStyle? = null;
    value: TextRunStyle? = null;
    fills: Array<Path> = new Array<Path>();
    paints: Array<Paint> = new Array<Paint>();
    present: StaticArray<bool> = new StaticArray<bool>(GROUP_COUNT);
    path: Path = new Path();
    marks: Path = new Path();
    paint: Paint = new Paint();
    line: Paint = new Paint();

    override init(context: Context): bool {
        for (let g = 0; g < GROUP_COUNT; g++) {
            this.fills.push(new Path());
            let p = new Paint();
            p.color = GROUP_TONES[g];
            this.paints.push(p);
        }
        this.line.style = PaintStyle.stroke;
        let font = context.font("InterVariable");
        if (font != null) {
            this.caption = style(font, 420, 8.5, 1.4);
            this.small = style(font, 300, 10, 0.2);
            this.value = style(font, 300, 15, 0.2);
        }
        return true;
    }

    override resize(width: f64, height: f64, scale: f64): void {
        this.width = <f32>width;
        this.height = <f32>height;
    }

    override draw(renderer: Renderer): void {
        if (!g_threads.open || this.caption == null || this.width < 40) {
            return;
        }
        this.labels.used = 0;
        let w = this.width;
        let h = this.height;
        if (w >= 520) {
            let historyWidth = Mathf.min(Mathf.max(w * 0.3, 220), 300);
            this.timeline(renderer, 0, 0, w - historyWidth - 28, h);
            this.history(renderer, w - historyWidth, 0, historyWidth, h);
        } else {
            // Stacked, the history keeps its figures and the timeline takes the rest
            let historyHeight: f32 = 36;
            this.timeline(renderer, 0, 0, w, h - historyHeight - 12);
            this.history(renderer, 0, h - historyHeight, w, historyHeight);
        }
    }

    private rowName(row: i32): string {
        if (row == 0) {
            return "main";
        }
        if (g_threads.backend == BACKEND_LANES) {
            return row == g_threads.threads ? "tree job" : "lane " + row.toString();
        }
        return "part " + row.toString();
    }

    // One row per thread, each span placed by when it ran in the latest step
    private timeline(renderer: Renderer, x: f32, y: f32, w: f32, h: f32): void {
        let caption = this.caption!;
        let small = this.small!;
        let labels = this.labels;
        let legend = w >= 480;
        labels.draw(renderer, caption, legend ? "LATEST STEP, BY THREAD" : "LATEST STEP", x, y, FAINT, false);

        let begin = b3GetTraceBegin();
        let stepMs = (b3GetTraceEnd() - begin) * 1000;
        this.axisMs = Math.max(this.axisMs * 0.97, stepMs * 1.08);
        let axisMs = Math.max(this.axisMs, 0.05);

        let rows = g_threads.backend == BACKEND_SERIAL ? 1 : g_threads.threads;
        if (g_threads.backend == BACKEND_LANES && b3GetTraceRow(rows).count > 0) {
            rows += 1;
        }
        rows = min(rows, MAX_ROWS);
        let top = y + 18;
        let bottom = y + h - 14;
        let pitch = Mathf.min(16, (bottom - top) / <f32>rows);
        let gap: f32 = pitch >= 8 ? 2 : 1;
        let trackX = x + LABEL_WIDTH;
        let trackW = w - LABEL_WIDTH;
        let pxPerMs = <f64>trackW / axisMs;

        let path = this.path;
        path.reset();
        for (let r = 0; r < rows; r++) {
            rect(path, trackX, top + pitch * <f32>r, trackW, pitch - gap);
        }
        this.paint.color = TRACK;
        renderer.drawPath(path, this.paint);

        for (let g = 0; g < GROUP_COUNT; g++) {
            this.fills[g].reset();
            this.present[g] = false;
        }
        for (let r = 0; r < rows; r++) {
            let row = b3GetTraceRow(r);
            let rowY = top + pitch * <f32>r;
            for (let k = 0; k < row.count; k++) {
                let x0 = <f32>((row.begins[k] - begin) * 1000 * pxPerMs);
                let x1 = <f32>((row.ends[k] - begin) * 1000 * pxPerMs);
                x0 = Mathf.max(x0, 0);
                x1 = Mathf.min(x1, trackW);
                // A sliver between neighbors keeps back to back stages apart
                let inset = Mathf.min(0.5, (x1 - x0) * 0.25);
                let spanW = Mathf.max(x1 - x0 - 2 * inset, 0.6);
                let group = groupOf(row.kinds[k]);
                rect(this.fills[group], trackX + x0 + inset, rowY, spanW, pitch - gap);
                this.present[group] = true;
            }
            if (pitch >= 7) {
                labels.draw(renderer, small, this.rowName(r), x, rowY + (pitch - gap) * 0.5 - 4, DIM, false);
            }
        }
        for (let g = 0; g < GROUP_COUNT; g++) {
            renderer.drawPath(this.fills[g], this.paints[g]);
        }

        // The step's end, and the axis under the rows
        let endX = trackX + <f32>(stepMs * pxPerMs);
        let rowsBottom = top + pitch * <f32>rows - gap;
        path.reset();
        rect(path, Mathf.min(endX, trackX + trackW - 1), top - 3, 1, rowsBottom - top + 6);
        this.paint.color = DIM;
        renderer.drawPath(path, this.paint);
        let tick = niceAtLeast(AXIS_STEPS, axisMs / 5);
        let ticks = <i32>Math.floor(axisMs / tick);
        path.reset();
        for (let i = 0; i <= ticks; i++) {
            let tx = trackX + <f32>(<f64>i * tick * pxPerMs);
            rect(path, tx, rowsBottom + 3, 1, 3);
            let text = formatFixed(<f64>i * tick, tick < 0.1 ? 2 : tick < 1 ? 1 : 0) + (i == ticks ? " ms" : "");
            labels.draw(renderer, caption, text, tx + 3, rowsBottom + 4, FAINT, false);
        }
        this.paint.color = HAIRLINE;
        renderer.drawPath(path, this.paint);

        // The legend, right aligned on the caption's line, for the groups this step shows
        let right = x + w;
        for (let g = GROUP_COUNT - 1; g >= 0; g--) {
            if (!legend || !this.present[g]) {
                continue;
            }
            right -= labels.draw(renderer, caption, GROUP_NAMES[g], right, y, DIM, true) + 6;
            path.reset();
            rect(path, right - 6, y, 6, 6);
            renderer.drawPath(path, this.paints[g]);
            right -= 18;
        }
    }

    // Milliseconds per step over the last HISTORY steps, the serial time beside it once measured
    private history(renderer: Renderer, x: f32, y: f32, w: f32, h: f32): void {
        let t = g_threads;
        let caption = this.caption!;
        let labels = this.labels;
        let threads = t.backend == BACKEND_SERIAL ? "" : ", " + t.threads.toString() + (t.threads == 1 ? " THREAD" : " THREADS");
        labels.draw(renderer, caption, "STEP TIME", x, y, FAINT, false);
        labels.draw(renderer, caption, BACKEND_NAMES[t.backend].toUpperCase() + threads, x + w, y, DIM, true);
        labels.draw(renderer, this.value!, formatFixed(t.stepMs, 2) + " ms", x, y + 16, INK, false);
        let speed = "";
        if (t.backend == BACKEND_SERIAL) {
            speed = "the serial baseline";
        } else if (t.serialMs > 0 && t.stepMs > 0) {
            speed = formatFixed(t.serialMs / t.stepMs, 1) + "x serial (" + formatFixed(t.serialMs, 2) + " ms)";
        } else {
            speed = "Serial not yet measured";
        }
        labels.draw(renderer, this.small!, speed, x + w, y + 18, DIM, true);

        let top = y + 42;
        let bottom = y + h - 14;
        if (bottom - top < 12 || t.historyCount == 0) {
            return;
        }
        let most = 0.0;
        for (let i = 0; i < t.historyCount; i++) {
            most = Math.max(most, t.history[i]);
        }
        most = Math.max(most, t.serialMs);
        let scale = niceAtLeast(SCALE_STEPS, most * 1.05);
        let plotH = bottom - top;
        let path = this.path;

        path.reset();
        rect(path, x, top, w, 1);
        rect(path, x, bottom, w, 1);
        this.paint.color = HAIRLINE;
        renderer.drawPath(path, this.paint);
        labels.draw(renderer, caption, formatFixed(scale, scale < 1 ? 2 : 0) + " MS", x, top + 4, FAINT, false);
        labels.draw(renderer, caption, "5 S AGO", x, bottom + 4, FAINT, false);
        labels.draw(renderer, caption, "NOW", x + w, bottom + 4, FAINT, true);

        if (t.serialMs > 0 && t.backend != BACKEND_SERIAL) {
            let sy = bottom - <f32>(t.serialMs / scale) * plotH;
            path.reset();
            for (let dx: f32 = 0; dx < w; dx += 6) {
                rect(path, x + dx, sy, 3, 1);
            }
            this.paint.color = DIM;
            renderer.drawPath(path, this.paint);
            labels.draw(renderer, caption, "SERIAL", x + w, sy - 9, DIM, true);
        }

        // Oldest at the left; a mark where the backend or thread count changed
        let first = t.historyCount < HISTORY ? 0 : t.historyHead;
        let dx = w / <f32>(HISTORY - 1);
        let start = x + w - dx * <f32>(t.historyCount - 1);
        path.reset();
        let marks = this.marks;
        marks.reset();
        for (let i = 0; i < t.historyCount; i++) {
            let at = (first + i) % HISTORY;
            let px = start + dx * <f32>i;
            let py = bottom - <f32>Math.min(t.history[at] / scale, 1.0) * plotH;
            if (i == 0) {
                path.moveTo(Vector.xy(px, py));
            } else {
                path.lineTo(Vector.xy(px, py));
                let before = (at + HISTORY - 1) % HISTORY;
                if (t.historyBackend[at] != t.historyBackend[before] || t.historyThreads[at] != t.historyThreads[before]) {
                    rect(marks, px, top, 1, plotH);
                }
            }
        }
        this.paint.color = FAINT;
        renderer.drawPath(marks, this.paint);
        this.line.color = t.backend == BACKEND_PARALLEL ? INK : t.backend == BACKEND_LANES ? ACCENT : DIM;
        this.line.thickness = 1.25;
        renderer.drawPath(path, this.line);
    }
}

function style(font: Font, weight: f32, size: f32, track: f32): TextRunStyle {
    let axes = new Map<string, f32>();
    axes.set("wght", weight);
    let features = new Map<string, u32>();
    features.set("tnum", 1);
    let s = new TextRunStyle(font.withOptions(axes, features));
    s.size = size;
    s.letterSpacing = track;
    return s;
}

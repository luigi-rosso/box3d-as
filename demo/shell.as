// The Demo view model that demo/shell.rml draws the UI from. The panels write
// scene, paused and tool and fire stepOnce and reset; the script writes the
// rest. demo/gen/shell.py declares the same names.

import { PropertyBoolean, PropertyNumber, PropertyString, PropertyTrigger, ViewModel } from "rive/data";
import { formatFixed } from "rive/format";
import { SceneStats, PROFILE_COUNT } from "./scene";
import { BACKEND_LANES, BACKEND_PARALLEL, THREAD_CHOICES, ThreadsState } from "./threads";

const COUNT_NAMES: StaticArray<string> = ["bodies", "awake", "contacts", "joints", "islands", "faces"];
const PROFILE_NAMES: StaticArray<string> = ["profilePairs", "profileCollide", "profileSolver", "profileUpdate",
    "profileCcd", "profileIslands", "profileOther"];

export const CLASS_WIDE: i32 = 0;
export const CLASS_MEDIUM: i32 = 1;
export const CLASS_COMPACT: i32 = 2;

// Wide keeps every panel; compact is for phones and small windows; medium sits between.
export function sizeClass(width: f32, height: f32): i32 {
    if (width < 720 || width < height) {
        return CLASS_COMPACT;
    }
    if (width >= 1100 && width >= 1.2 * height) {
        return CLASS_WIDE;
    }
    return CLASS_MEDIUM;
}

// The panels grow and shrink with the window's smaller side, 800 points being 1.
export function uiScaleFor(width: f32, height: f32): f32 {
    return Mathf.min(Mathf.max(Mathf.min(width, height) / 800, 0.85), 1.3);
}

export function formatMs(ms: f64): string {
    return formatFixed(ms, ms < 0.1 ? 3 : 2) + " ms";
}

export class Shell {
    scene: PropertyNumber;
    paused: PropertyBoolean;
    tool: PropertyNumber;
    stepOnce: PropertyTrigger;
    reset: PropertyTrigger;
    // Raised by a press on a control, so the 3D view leaves that press alone.
    uiPress: PropertyBoolean;
    sceneTitle: PropertyString;
    description: PropertyString;
    stepMs: PropertyString;
    drawMs: PropertyString;
    profileMs: PropertyString;
    layoutClass: PropertyNumber;
    uiScale: PropertyNumber;
    uiWidth: PropertyNumber;
    uiHeight: PropertyNumber;
    // Bounds of the area the panels leave clear, written back by the layout.
    clear: Array<PropertyNumber> = new Array<PropertyNumber>();
    counts: Array<PropertyNumber> = new Array<PropertyNumber>();
    profile: Array<PropertyNumber> = new Array<PropertyNumber>();
    // The Threads scene's panel, and the backend and threads last written to it
    threadsOpen: PropertyBoolean;
    backend: PropertyNumber;
    threads: PropertyNumber;
    threadsNote: PropertyString;
    okLanes: PropertyBoolean;
    okParallel: PropertyBoolean;
    okThreads: Array<PropertyBoolean> = new Array<PropertyBoolean>();
    shownBackend: i32 = -1;
    shownThreads: i32 = -1;

    constructor(vm: ViewModel, sceneNames: Array<string>) {
        this.scene = vm.getNumber("scene")!;
        this.paused = vm.getBoolean("paused")!;
        this.tool = vm.getNumber("tool")!;
        this.stepOnce = vm.getTrigger("stepOnce")!;
        this.reset = vm.getTrigger("reset")!;
        this.uiPress = vm.getBoolean("_uiPress")!;
        this.sceneTitle = vm.getString("sceneTitle")!;
        this.description = vm.getString("description")!;
        this.layoutClass = vm.getNumber("layoutClass")!;
        this.uiScale = vm.getNumber("uiScale")!;
        this.uiWidth = vm.getNumber("uiWidth")!;
        this.uiHeight = vm.getNumber("uiHeight")!;
        this.clear.push(vm.getNumber("clearX")!);
        this.clear.push(vm.getNumber("clearY")!);
        this.clear.push(vm.getNumber("clearWidth")!);
        this.clear.push(vm.getNumber("clearHeight")!);
        this.stepMs = vm.getString("stepMs")!;
        this.drawMs = vm.getString("drawMs")!;
        this.profileMs = vm.getString("profileMs")!;
        for (let i = 0; i < COUNT_NAMES.length; i++) {
            this.counts.push(vm.getNumber(COUNT_NAMES[i])!);
        }
        for (let i = 0; i < PROFILE_COUNT; i++) {
            this.profile.push(vm.getNumber(PROFILE_NAMES[i])!);
        }
        for (let i = 0; i < sceneNames.length; i++) {
            vm.getString("sceneName" + i.toString())!.value = sceneNames[i];
        }
        this.threadsOpen = vm.getBoolean("threadsOpen")!;
        this.backend = vm.getNumber("backend")!;
        this.threads = vm.getNumber("threads")!;
        this.threadsNote = vm.getString("threadsNote")!;
        this.okLanes = vm.getBoolean("_okLanes")!;
        this.okParallel = vm.getBoolean("_okParallel")!;
        for (let i = 1; i < THREAD_CHOICES.length; i++) {
            this.okThreads.push(vm.getBoolean("_okThreads" + THREAD_CHOICES[i].toString())!);
        }
    }

    // Takes the panel's clicks as requests, then shows what the scene steps on.
    syncThreads(t: ThreadsState): void {
        this.threadsOpen.value = t.open;
        if (!t.open) {
            return;
        }
        let backend = <i32>this.backend.value;
        if (backend != this.shownBackend && this.shownBackend >= 0) {
            t.wantBackend = backend;
        }
        let threads = <i32>this.threads.value;
        if (threads != this.shownThreads && this.shownThreads >= 0) {
            t.wantThreads = threads;
        }
        backend = t.targetBackend();
        threads = t.threadsFor(backend, t.wantThreads);
        this.shownBackend = backend;
        this.shownThreads = threads;
        this.backend.value = <f64>backend;
        this.threads.value = <f64>threads;
        this.okLanes.value = t.available(BACKEND_LANES);
        this.okParallel.value = t.available(BACKEND_PARALLEL);
        for (let i = 1; i < THREAD_CHOICES.length; i++) {
            this.okThreads[i - 1].value = THREAD_CHOICES[i] <= t.maxThreads(backend);
        }
        this.threadsNote.value = t.note(backend);
    }

    // Picks the size class and scale for a window size.
    fit(width: f32, height: f32): void {
        let scale = uiScaleFor(width, height);
        this.layoutClass.value = <f64>sizeClass(width, height);
        this.uiScale.value = <f64>scale;
        this.uiWidth.value = <f64>(width / scale);
        this.uiHeight.value = <f64>(height / scale);
    }

    // True once for a press that a control already took.
    takeUiPress(): bool {
        if (!this.uiPress.value) {
            return false;
        }
        this.uiPress.value = false;
        return true;
    }

    showStats(stepMs: f64, drawMs: f64, s: SceneStats, faces: i32): void {
        this.stepMs.value = formatMs(stepMs);
        this.drawMs.value = formatMs(drawMs);
        this.counts[0].value = <f64>s.bodies;
        this.counts[1].value = <f64>s.awake;
        this.counts[2].value = <f64>s.contacts;
        this.counts[3].value = <f64>s.joints;
        this.counts[4].value = <f64>s.islands;
        this.counts[5].value = <f64>faces;
        this.profileMs.value = formatMs(<f64>s.profileStep);
        for (let i = 0; i < PROFILE_COUNT; i++) {
            this.profile[i].value = <f64>s.profile[i];
        }
    }
}

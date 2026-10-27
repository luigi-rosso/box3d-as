// The Demo view model that demo/shell.rml draws the UI from. The panels write
// scene, paused and tool and fire stepOnce and reset; the script writes the
// rest. demo/gen/shell.py declares the same names.

import { PropertyBoolean, PropertyNumber, PropertyString, PropertyTrigger, ViewModel } from "rive/data";
import { formatFixed } from "rive/format";
import { SceneStats, PROFILE_COUNT } from "./scene";

const COUNT_NAMES: StaticArray<string> = ["bodies", "awake", "contacts", "joints", "islands", "faces"];
const PROFILE_NAMES: StaticArray<string> = ["profilePairs", "profileCollide", "profileSolver", "profileUpdate",
    "profileCcd", "profileIslands", "profileOther"];

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
    description: PropertyString;
    stepMs: PropertyString;
    drawMs: PropertyString;
    profileMs: PropertyString;
    counts: Array<PropertyNumber> = new Array<PropertyNumber>();
    profile: Array<PropertyNumber> = new Array<PropertyNumber>();

    constructor(vm: ViewModel, sceneNames: Array<string>) {
        this.scene = vm.getNumber("scene")!;
        this.paused = vm.getBoolean("paused")!;
        this.tool = vm.getNumber("tool")!;
        this.stepOnce = vm.getTrigger("stepOnce")!;
        this.reset = vm.getTrigger("reset")!;
        this.uiPress = vm.getBoolean("_uiPress")!;
        this.description = vm.getString("description")!;
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

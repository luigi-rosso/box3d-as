// Small UI pieces for the shell: cached text runs and rounded rectangles.

import { Renderer } from "rive/renderer";
import { Paint } from "rive/paint";
import { Path } from "rive/path";
import { Vector } from "rive/vector";
import { Mat2D } from "rive/mat2d";
import { Color } from "rive/color";
import { Font, Text, TextRunStyle } from "rive/text";

export const UI_INK: Color = 0xFFEDE8E1;
export const UI_DIM: Color = 0xFF8C867E;
export const UI_FAINT: Color = 0xFF5A5650;
export const UI_HAIRLINE: Color = 0xFF3B3833;
export const UI_HAIRLINE_HOT: Color = 0xFF6B665F;
export const UI_PANEL: Color = 0xE6181715;
export const UI_INVERTED_TEXT: Color = 0xFF151412;
export const UI_ACCENT: Color = 0xFFE0875A;

// Font at a weight with tabular figures.
export function fontAt(font: Font, weight: f32): Font {
    let axes = new Map<string, f32>();
    axes.set("wght", weight);
    let features = new Map<string, u32>();
    features.set("tnum", 1);
    return font.withOptions(axes, features);
}

export function textStyle(font: Font, size: f32, spacing: f32): TextRunStyle {
    let style = new TextRunStyle(font);
    style.size = size;
    style.letterSpacing = spacing;
    return style;
}

// A text run that reshapes only when its string changes.
export class UiText {
    text: Text = new Text();
    value: string = "";
    width: f32 = 0;
    height: f32 = 0;
    style: TextRunStyle;
    private shaped: bool = false;

    constructor(style: TextRunStyle) {
        this.style = style;
    }

    set(value: string): UiText {
        if (this.shaped && value == this.value) {
            return this;
        }
        this.shaped = true;
        this.value = value;
        this.text.clear();
        this.text.append(value, this.style);
        let b = this.text.bounds();
        this.width = b.v1.x - b.v0.x;
        this.height = b.v1.y - b.v0.y;
        return this;
    }

    // Draws with the top left at x, y; align 0 left, 1 right, 0.5 center.
    draw(renderer: Renderer, paint: Paint, x: f32, y: f32, align: f32): void {
        renderer.save();
        renderer.transform(Mat2D.withTranslation(Mathf.round(x - this.width * align), Mathf.round(y)));
        this.text.draw(renderer, paint);
        renderer.restore();
    }
}

export class Rect {
    x: f32 = 0;
    y: f32 = 0;
    w: f32 = 0;
    h: f32 = 0;

    setTo(x: f32, y: f32, w: f32, h: f32): void {
        this.x = x;
        this.y = y;
        this.w = w;
        this.h = h;
    }

    contains(px: f32, py: f32): bool {
        return px >= this.x && px <= this.x + this.w && py >= this.y && py <= this.y + this.h;
    }
}

export function addRoundRect(path: Path, x: f32, y: f32, w: f32, h: f32, r: f32): void {
    let k: f32 = r * 0.4477;
    path.moveTo(Vector.xy(x + r, y));
    path.lineTo(Vector.xy(x + w - r, y));
    path.cubicTo(Vector.xy(x + w - k, y), Vector.xy(x + w, y + k), Vector.xy(x + w, y + r));
    path.lineTo(Vector.xy(x + w, y + h - r));
    path.cubicTo(Vector.xy(x + w, y + h - k), Vector.xy(x + w - k, y + h), Vector.xy(x + w - r, y + h));
    path.lineTo(Vector.xy(x + r, y + h));
    path.cubicTo(Vector.xy(x + k, y + h), Vector.xy(x, y + h - k), Vector.xy(x, y + h - r));
    path.lineTo(Vector.xy(x, y + r));
    path.cubicTo(Vector.xy(x, y + k), Vector.xy(x + k, y), Vector.xy(x + r, y));
    path.close();
}

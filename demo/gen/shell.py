"""Generates demo/shell.rml, the demo's UI shell around the scripted 3D view.

Run with `python3 demo/gen/shell.py` from the repo root.
"""

import math
import os
import struct
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))

from rml import E, VM, Machine, Node, bind, cond, document, set_vm, toggle_vm

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

SCENES = 7
PROFILE = ["pairs", "collide", "solver", "update", "ccd", "islands", "other"]
TONES = ["FFEDE8E1", "FFC9C2B8", "FFA59E95", "FF877F75", "FF6E6860", "FF58534D", "FF45413C"]
STATS = ["bodies", "awake", "contacts", "joints", "islands", "faces"]
# Size classes the script writes to layoutClass; see demo/shell.as.
WIDE, MEDIUM, COMPACT = 0, 1, 2
CLASSES = [WIDE, MEDIUM, COMPACT]
# No break space inside each hint, so a wrapped line keeps a key with its action.
KEY_HELP = "   ".join(h.replace(" ", "\u00a0") for h in (
    "Drag orbit", "Shift drag pan", "Scroll zoom", "P pause", "O step", "R reset", "Tab tool", "C contacts", "J joints",
    "1-7 scene"))
TOUCH_HELP = "   ".join(h.replace(" ", "\u00a0") for h in ("Drag to orbit", "Pinch to zoom", "Tap a body to pick"))

INK = "FFEDE8E1"
DIM = "FF8C867E"
FAINT = "FF5A5650"
HAIRLINE = "FF3B3833"
HAIRLINE_HOT = "FF6B665F"
INVERTED = "FF151412"
ACCENT = "FFE0875A"
CLEAR = "00EDE8E1"
CLEAR_HOT = "006B665F"
BACKGROUND = "FF151412"

LIGHT = 300
REGULAR = 420
HOVER_MS = 120

FONT = E("FontAsset", file="demo/fonts/InterVariable.ttf", name="InterVariable")
NEGATE = E("DataConverterBooleanNegate", name="Not")
WHOLE = E("DataConverterToString", decimals=0, round=True, name="Whole")
HUNDREDTHS = E("DataConverterToString", decimals=2, round=True, name="Hundredths")
# Yoga hands out only part of the free space when fill weights sum below 1.
WEIGHT = E("DataConverterOperationValue", operationType=2, operationValue=1e6, name="Weight")

# (object, property) -> value per size class, gathered while building and
# played by the Size layer; the authored values are the wide ones.
SIZE_KEYS = {c: {} for c in CLASSES}


def tag(name):
    return struct.unpack(">I", name.encode())[0]


def per_class(obj, prop, values):
    for c, v in zip(CLASSES, values):
        SIZE_KEYS[c][(obj, prop)] = v


def shown_in(node, *classes):
    """Takes a box out of the layout, and out of hit testing, in other classes."""
    style = node.meta["style"]
    per_class(style, "displayValue", [("uint", 0 if c in classes else 1) for c in CLASSES])
    if WIDE not in classes:
        style.attrs["displayValue"] = "none"
    return node


# ------------------------------------------------------------------ layout


def _points(style, prop, value):
    style.attrs[prop] = float(value)
    style.attrs[prop + "UnitsValue"] = "points"


def _sides(style, prop, value):
    """value: (left, top, right, bottom)."""
    for side, v in zip(("Left", "Top", "Right", "Bottom"), value):
        if v:
            _points(style, prop + side, v)


def paint(kind, color, **attrs):
    c = E("SolidColor", colorValue=color, name="Color")
    p = E(kind, c, name=kind, **attrs)
    p.meta["color"] = c
    return p


def box(name, *kids, w=None, h=None, ws=None, hs=None, dir="column", align=None, gap=None, pad=None, margin=None,
        bg=None, line=None, radius=None, clip=None, absolute=None):
    """A LayoutComponent and its style; a side with no size hugs."""
    ws = ws or ("fixed" if w is not None else "hug")
    hs = hs or ("fixed" if h is not None else "hug")
    style = E("LayoutComponentStyle", name=f"{name} Style", layoutWidthScaleType=ws, layoutHeightScaleType=hs,
              flexDirectionValue=dir, layoutAlignmentType=align)
    if "hug" in (ws, hs) and absolute is None:
        style.attrs["intrinsicallySizedValue"] = True
    if gap is not None:
        _points(style, "gapHorizontal", gap)
        _points(style, "gapVertical", gap)
    if pad is not None:
        _sides(style, "padding", pad)
    if margin is not None:
        _sides(style, "margin", margin)
    if radius is not None:
        for corner in ("TL", "TR", "BL", "BR"):
            style.attrs["cornerRadius" + corner] = float(radius)
    if absolute is not None:
        style.attrs["positionTypeValue"] = "absolute"
        for side, v in absolute.items():
            _points(style, "position" + side.capitalize(), v)
        if "top" in absolute and "bottom" in absolute:
            style.attrs["layoutHeightScaleType"] = "fixed"
            style.attrs["heightUnitsValue"] = "auto"
        if "left" in absolute and "right" in absolute:
            style.attrs["layoutWidthScaleType"] = "fixed"
            style.attrs["widthUnitsValue"] = "auto"
    style.attrs["id"] = style.attrs.pop("id")
    node = E("LayoutComponent", style, width=w, height=h, clip=clip, styleId=style, name=name)
    node.meta["style"] = style
    if bg is not None:
        node.meta["bg"] = bg if isinstance(bg, Node) else paint("Fill", bg)
        node.add(node.meta["bg"])
    if line is not None:
        node.meta["line"] = paint("Stroke", line, thickness=1.0)
        node.add(node.meta["line"])
    return node.add(*kids)


def text_style(size, color, weight, track=0.0, line_height=None):
    """Inter at a weight with tabular figures."""
    style = E("TextStylePaint", paint("Fill", color),
              E("TextStyleAxis", tag=tag("wght"), axisValue=float(weight), name="Weight"),
              E("TextStyleFeature", tag=tag("tnum"), featureValue=1, name="tnum"),
              fontSize=float(size), letterSpacing=float(track) if track else None, lineHeight=line_height,
              fontAssetId=FONT, familyName="Inter", styleName="Light" if weight == LIGHT else "Regular", name="Style")
    style.meta["color"] = style.kids[0].meta["color"]
    return style


def text(name, styles, runs, width=None, fill=False, align=None):
    """runs: [(style, text, bind path or None, converter or None)]. Sized to its
    glyphs unless given a width (or fill) to wrap in."""
    wraps = width is not None or fill
    flow = E("LayoutParticipant", name="Flow", width=width,
             layoutWidthScaleType="fixed" if width is not None else ("fill" if fill else "hug"),
             layoutHeightScaleType="hug")
    t = E("Text", flow, *styles, sizingValue="autoHeight" if wraps else "autoWidth",
          wrapValue="wrap" if wraps else "noWrap", alignValue=align, name=name)
    for style, literal, path, converter in runs:
        run = E("TextValueRun", styleId=style, text=literal, name="Run")
        if path:
            run.add(bind(path, "text", converter))
        t.add(run)
    return t


def label(literal, size, color, weight, track=0.0, path=None, converter=None, width=None, fill=False, align=None,
          line_height=None, name=None):
    style = text_style(size, color, weight, track, line_height)
    t = text(name or literal, [style], [(style, literal, path, converter)], width, fill, align)
    t.meta["color"] = style.meta["color"]
    return t


def rule(w=None, margin=None):
    return box("Rule", w=w, ws=None if w else "fill", h=1, bg=HAIRLINE, margin=margin)


def scrim(name, side, size):
    """Darkens the 3D view behind a panel, fading toward the middle."""
    stops = [E("GradientStop", colorValue="B8151412", position=0.0), E("GradientStop", colorValue="00151412", position=1.0)]
    across = side in ("left", "right")
    start, end = (0.0, size) if side in ("left", "top") else (size, 0.0)
    ends = dict(startX=start, startY=0.0, endX=end, endY=0.0) if across else dict(startX=0.0, startY=start, endX=0.0, endY=end)
    fill = E("Fill", E("LinearGradient", *stops, **ends, name="Gradient"), name="Fill")
    pinned = {"top": 0, "bottom": 0} if across else {"left": 0, "right": 0}
    return box(name, w=size if across else None, h=None if across else size, bg=fill,
               absolute={side: 0, **pinned})


# ------------------------------------------------------------------ icons


def points_path(points, closed=False):
    return E("PointsPath", *[E("StraightVertex", x=round(x, 4), y=round(y, 4)) for x, y in points],
             isClosed=closed, name="Path")


def icon(name, *paths):
    """Stroked line art drawn about the origin."""
    shape = E("Shape", *paths, name=name)
    stroke = paint("Stroke", DIM, thickness=1.25, cap="round", join="round")
    shape.add(stroke)
    shape.meta["ink"] = stroke.meta["color"]
    return shape


def reset_paths():
    """A counterclockwise arc from the top round to the upper right, with an
    open corner head turned clockwise so its inner arm clears the arc."""
    r, a0, sweep, steps = 6.0, -1.5708, 5.63, 24
    arc = [(math.cos(a0 - sweep * k / steps) * r, math.sin(a0 - sweep * k / steps) * r) for k in range(steps + 1)]
    a1 = a0 - sweep
    tip = arc[-1]
    bx, by = -math.sin(a1), math.cos(a1)
    arm = 0.45 * r

    def arm_end(turn):
        return (tip[0] + (bx * math.cos(turn) - by * math.sin(turn)) * arm,
                tip[1] + (bx * math.sin(turn) + by * math.cos(turn)) * arm)

    return [points_path(arc), points_path([arm_end(0.785 + 0.35), tip, arm_end(-0.785 + 0.35)])]


# ------------------------------------------------------------------ data


def view_model():
    vm = VM("Demo")
    vm.num("scene")
    vm.bool("paused")
    vm.num("tool")
    vm.trig("stepOnce")
    vm.trig("reset")
    for i in range(SCENES):
        vm.str(f"sceneName{i}", f"Scene {i + 1}")
    vm.str("sceneTitle", "")
    vm.str("description", "")
    vm.str("stepMs", "0 ms")
    vm.str("drawMs", "0 ms")
    for s in STATS:
        vm.num(s)
    vm.str("profileMs", "0 ms")
    for p in PROFILE:
        vm.num("profile" + p.capitalize())
    vm.num("layoutClass")
    vm.num("uiScale", 1)
    vm.num("uiWidth", 1280)
    vm.num("uiHeight", 800)
    for edge in ("X", "Y", "Width", "Height"):
        vm.num("clear" + edge)
    vm.bool("_uiPress")
    return vm


# ------------------------------------------------------------------ controls


def equals(path, value):
    """Active while a number property holds value."""
    return [cond("Number", path, "equal", float(value))], [cond("Number", path, "notEqual", float(value))]


def is_true(path):
    return [cond("Boolean", path, "equal", True)], [cond("Boolean", path, "equal", False)]


class Controls:
    """Hover, press and active looks for every clickable control. A press also
    raises _uiPress so the 3D view ignores it."""

    def __init__(self, machine, vm):
        self.m = machine
        self.vm = vm

    def add(self, node, looks, actions, active=None):
        """looks: {"idle"|"hover"|"active": {(object, property): value}};
        active: (conditions while active, conditions while not)."""
        name = node.attrs["name"]
        hover = self.vm.bool("_hover" + name.replace(" ", ""))
        hp = self.vm.path(hover)
        self.m.listen(node, "enter", set_vm("Boolean", hp, True))
        self.m.listen(node, "exit", set_vm("Boolean", hp, False))
        self.m.listen(node, "down", *actions, set_vm("Boolean", self.vm.path("_uiPress"), True))
        on, off = is_true(hp)
        states = [("idle", looks["idle"], [(1, HOVER_MS, on)]), ("hover", looks["hover"], [(0, HOVER_MS, off)])]
        if active:
            when, unless = active
            states[0][2].insert(0, (2, HOVER_MS, when))
            states[1][2].insert(0, (2, HOVER_MS, when))
            states.append(("active", looks["active"], [(1, HOVER_MS, unless + on), (0, HOVER_MS, unless + off)]))
        self.m.layer(name, states)

    def picker(self, node, inks, path, value, idle_ink):
        """A row or chip that opens a scene: outlined on hover, inverted when open."""
        bg, line = node.meta["bg"].meta["color"], node.meta["line"].meta["color"]

        def look(fill, stroke, ink, first_ink=None):
            keys = {(bg, "colorValue"): fill, (line, "colorValue"): stroke}
            for i, c in enumerate(inks):
                keys[(c, "colorValue")] = first_ink if i == 0 and first_ink else ink
            return keys

        self.add(node, {"idle": look(CLEAR, CLEAR_HOT, idle_ink, FAINT if len(inks) > 1 else None),
                        "hover": look(CLEAR, HAIRLINE_HOT, INK, FAINT if len(inks) > 1 else None),
                        "active": look(INK, CLEAR_HOT, INVERTED)},
                 [set_vm("Number", path, float(value))], active=equals(path, value))
        return node


def scene_row(controls, vm, i):
    index = label(f"{i + 1:02d}", 9.5, FAINT, REGULAR, 1.0, width=26, name="Index")
    title = label(f"Scene {i + 1}", 13, DIM, LIGHT, 0.2, path=vm.path(f"sceneName{i}"), name="Name")
    row = box(f"Scene {i + 1}", index, title, w=196, h=26, dir="row", align="centerLeft", pad=(10, 0, 0, 0),
              bg=CLEAR, line=CLEAR_HOT, radius=4)
    return controls.picker(row, [index.meta["color"], title.meta["color"]], vm.path("scene"), i, DIM)


def scene_chip(controls, vm, i):
    index = label(f"{i + 1:02d}", 9.5, DIM, REGULAR, 1.0, name="Index")
    chip = box(f"Chip {i + 1}", index, w=34, h=26, dir="row", align="center", bg=CLEAR, line=CLEAR_HOT, radius=4)
    return controls.picker(chip, [index.meta["color"]], vm.path("scene"), i, DIM)


def icon_button(controls, name, shapes, actions, active=None, active_extra=None):
    art = E("Node", *shapes, x=17.0, y=17.0, name="Icon")
    button = box(name, art, w=34, h=34, bg=CLEAR, line=HAIRLINE, radius=4)
    bg, line = button.meta["bg"].meta["color"], button.meta["line"].meta["color"]
    inks = [s.meta["ink"] for s in shapes]

    def look(fill, stroke, ink, extra=None):
        keys = {(bg, "colorValue"): fill, (line, "colorValue"): stroke, **{(c, "colorValue"): ink for c in inks}}
        return {**keys, **(extra or {})}

    idle_extra, active_extra = active_extra or ({}, {})
    looks = {"idle": look(CLEAR, HAIRLINE, DIM, idle_extra), "hover": look(CLEAR, HAIRLINE_HOT, INK, idle_extra)}
    if active:
        looks["active"] = look(INK, INK, INVERTED, active_extra)
    controls.add(button, looks, actions, active)
    return button


def tool_button(controls, vm, name, value):
    t = label(name, 12, DIM, LIGHT, 0.3, name="Label")
    button = box(name, t, w=64, h=34, dir="row", align="center", bg=CLEAR, radius=4)
    bg, tc = button.meta["bg"].meta["color"], t.meta["color"]
    tool = vm.path("tool")
    controls.add(button,
                 {"idle": {(bg, "colorValue"): CLEAR, (tc, "colorValue"): DIM},
                  "hover": {(bg, "colorValue"): CLEAR, (tc, "colorValue"): INK},
                  "active": {(bg, "colorValue"): INK, (tc, "colorValue"): INVERTED}},
                 [set_vm("Number", tool, float(value))], active=equals(tool, value))
    return button


# ------------------------------------------------------------------ panels


def left_column(controls, vm):
    title = box("Title", label("BOX3D", 15, INK, LIGHT, 4.5, name="Brand"),
                label("ANIMASCRIPT PORT  /  DEBUG DRAW", 9.5, DIM, REGULAR, 1.8, name="Subtitle"),
                gap=6, pad=(10, 0, 0, 0))
    rows = shown_in(box("Scenes", *[scene_row(controls, vm, i) for i in range(SCENES)], gap=4, margin=(0, 26, 0, 0)),
                    WIDE, MEDIUM)
    chips = shown_in(box("Chips", box("Chip Row", *[scene_chip(controls, vm, i) for i in range(SCENES)], dir="row", gap=4),
                         label("", 13, INK, LIGHT, 0.2, path=vm.path("sceneTitle"), name="Scene Title"),
                         gap=12, pad=(10, 0, 0, 0), margin=(0, 20, 0, 0)), COMPACT)
    about = shown_in(box("About", rule(24),
                         label("", 11.5, DIM, LIGHT, 0.1, path=vm.path("description"), width=220, line_height=17,
                               name="Description"),
                         gap=13, pad=(10, 0, 0, 0), margin=(0, 18, 0, 0)), WIDE)
    return box("Left", title, rows, chips, about)


def stat_row(name, path, ink, converter=None, value_size=13, label_size=9.5, label_track=1.6, swatch=None):
    name_text = label(name.upper(), label_size, FAINT, REGULAR, label_track, name="Label")
    if swatch:
        name_text = box("Key", box("Swatch", w=6, h=6, bg=swatch, radius=1.5), name_text, dir="row", align="center", gap=8)
    value = label("0", value_size, ink, LIGHT, 0.2, path=path, converter=converter, name="Value")
    return box(name, name_text, value, ws="fill", dir="row", align="spaceBetweenEnd" if not swatch else "spaceBetweenCenter")


def right_panel(vm):
    P = vm.path
    timing = box("Timing", stat_row("Step", P("stepMs"), INK), stat_row("Draw", P("drawMs"), INK), ws="fill", gap=8)
    counts = [stat_row(s, P(s), DIM, WHOLE) for s in STATS]
    for row in counts[2:]:
        shown_in(row, WIDE)
    segments = [box(p.capitalize(), ws="fill", h=3, bg=tone) for p, tone in zip(PROFILE, TONES)]
    for p, seg in zip(PROFILE, segments):
        seg.add(bind(P("profile" + p.capitalize()), "fractionalWidth", WEIGHT))
    bar = box("Bar", *segments, ws="fill", h=3, dir="row", clip=True, margin=(0, 1, 0, 0))
    parts = box("Parts", *[stat_row(p, P("profile" + p.capitalize()), DIM, HUNDREDTHS, value_size=11, label_size=8.5,
                                    label_track=1.4, swatch=tone) for p, tone in zip(PROFILE, TONES)], ws="fill",
                gap=4.67)
    profile = shown_in(box("Profile", stat_row("B3 Profile", P("profileMs"), DIM), bar, parts, ws="fill", gap=9), WIDE)
    # Pinned to the corner in every class, since a keyed switch to absolute can land at the left after a live
    # resize; the wide class holds its column open with Right Room instead.
    return shown_in(box("Right", timing, rule(margin=(0, 4, 0, 9)), box("Counts", *counts, ws="fill", gap=8.4),
                        shown_in(rule(margin=(0, 4, 0, 13)), WIDE), profile, w=168, margin=(0, 1, 0, 0),
                        absolute={"top": 0, "right": 0}), WIDE, MEDIUM)


def stat_line(vm):
    """The compact class's stats, one line of label and value runs."""
    key = text_style(8.5, FAINT, REGULAR, 1.4)
    value = text_style(11, DIM, LIGHT, 0.2)
    runs = []
    for name, path, converter in (("STEP", "stepMs", None), ("DRAW", "drawMs", None), ("BODIES", "bodies", WHOLE),
                                  ("AWAKE", "awake", WHOLE)):
        runs += [(key, ("     " if runs else "") + name + "  ", None, None), (value, "0", vm.path(path), converter)]
    return shown_in(box("Stat Line", text("Stats", [key, value], runs), pad=(10, 0, 0, 0)), COMPACT)


def clear_area(vm):
    """What the panels leave of the view; its bounds go back to the script,
    which frames the scene in them."""
    area = box("Clear", ws="fill", hs="fill")
    for edge, key in (("X", 864), ("Y", 865), ("Width", 810), ("Height", 811)):
        ctx = bind(vm.path("clear" + edge), key)
        ctx.attrs["direction"] = True
        area.add(ctx)
    return area


def bottom_bar(controls, vm):
    P = vm.path
    paused_tag = label("PAUSED", 9.5, ACCENT, REGULAR, 2.0, name="Paused")
    paused_tag.attrs["opacity"] = 0.0
    bars = icon("Pause", points_path([(-3, -5.5), (-3, 5.5)]), points_path([(3, -5.5), (3, 5.5)]))
    play = icon("Play", points_path([(-3.5, -5.5), (5.5, 0), (-3.5, 5.5)], closed=True))
    play.attrs["opacity"] = 0.0
    pause = icon_button(controls, "Pause", [bars, play], [toggle_vm(P("paused"), NEGATE)], active=is_true(P("paused")),
                        active_extra=({(bars, "opacity"): 1, (play, "opacity"): 0, (paused_tag, "opacity"): 0},
                                      {(bars, "opacity"): 0, (play, "opacity"): 1, (paused_tag, "opacity"): 1}))
    step = icon_button(controls, "Step", [icon("Step", points_path([(-5, -5), (2, 0), (-5, 5)], closed=True),
                                               points_path([(5, -5.5), (5, 5.5)]))], [set_vm("Trigger", P("stepOnce"))])
    reset = icon_button(controls, "Reset", [icon("Reset", *reset_paths())], [set_vm("Trigger", P("reset"))])
    tools = box("Tools", tool_button(controls, vm, "Pick", 0), tool_button(controls, vm, "Shoot", 1), dir="row",
                line=HAIRLINE, radius=4, margin=(14, 0, 0, 0))
    transport = box("Transport", pause, step, reset, tools, paused_tag, dir="row", align="centerLeft", gap=8)
    keys = shown_in(box("Key Help", label(KEY_HELP, 11, FAINT, LIGHT, 0.2, fill=True, align="right", name="Help"),
                        ws="fill"), WIDE, MEDIUM)
    return box("Bottom", transport, keys, ws="fill", dir="row", align="bottomLeft", gap=140, pad=(10, 0, 0, 0))


def size_layer(machine, vm):
    """Plays the keys gathered by per_class for the class the script picked."""
    path = vm.path("layoutClass")
    states = []
    for c, name in zip(CLASSES, ("wide", "medium", "compact")):
        to = [(other, 0, [cond("Number", path, "equal", float(other))]) for other in CLASSES if other != c]
        states.append((name, SIZE_KEYS[c], to))
    machine.layer("Size", states)


def main():
    vm = view_model()
    P = vm.path
    script = E("ScriptAsset", file="main.as", name="main")
    machine = Machine("Shell")
    controls = Controls(machine, vm)

    left = left_column(controls, vm)
    # Right floats over the corner; in wide it is tall, so Right Room keeps the view clear of it.
    room = shown_in(box("Right Room", w=168, h=1), WIDE)
    top = box("Top", left, stat_line(vm), clear_area(vm), room, right_panel(vm), ws="fill", hs="fill", dir="row", gap=24)
    per_class(top.meta["style"], "flexDirectionValue", [("uint", 2), ("uint", 2), ("uint", 0)])
    touch = shown_in(box("Touch Help", label(TOUCH_HELP, 11, FAINT, LIGHT, 0.2, fill=True, name="Help"), ws="fill",
                         pad=(10, 0, 0, 0)), COMPACT)
    panels = box("Panels", top, touch, bottom_bar(controls, vm), gap=16,
                 absolute={"left": 18, "top": 28, "right": 28, "bottom": 28})
    scrims = [scrim("Scrim Left", "left", 364), scrim("Scrim Right", "right", 316), scrim("Scrim Top", "top", 240),
              scrim("Scrim Bottom", "bottom", 200)]
    for s, shown in zip(scrims, ((WIDE, MEDIUM), (WIDE, MEDIUM), (COMPACT,), (COMPACT,))):
        shown_in(s, *shown)
    # Laid out at the window size over uiScale, then scaled back up, so the
    # panels grow with the window.
    overlay = box("Overlay", panels, *scrims, w=1280, h=800, absolute={"left": 0, "top": 0})
    overlay.add(bind(P("uiWidth"), 7), bind(P("uiHeight"), 8), bind(P("uiScale"), 16), bind(P("uiScale"), 17))
    stage = box("Stage", E("ScriptedLayout", E("FocusData", name="Focus"), scriptAssetId=script, name="Demo"),
                ws="fill", hs="fill")
    size_layer(machine, vm)
    vm.instance()

    style = E("LayoutComponentStyle", name="Artboard Style")
    machine_node = machine.build()
    board = E("Artboard", style, paint("Fill", BACKGROUND), overlay, stage, machine_node, *machine.anims,
              styleId=style, viewModelId=vm, viewModelInstanceId=vm.node.attrs["defaultInstanceId"],
              defaultStateMachineId=machine_node, width=1280.0, height=800.0, x=0.0, y=0.0, name="Box3D")
    roots = [board, vm.node, FONT, script, NEGATE, WHOLE, HUNDREDTHS, WEIGHT]
    with open(os.path.join(ROOT, "demo", "shell.rml"), "w") as f:
        f.write(document(roots))


if __name__ == "__main__":
    main()

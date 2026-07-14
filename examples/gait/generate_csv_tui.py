#!/usr/bin/env python3
"""
Interactive TUI for CorgiRobot Hardware & Lean Pose CSV Generator.

Combines generate_csv_ui.py (Gait mode) and generate_lean_csv_ui.py (Lean mode)
into a single full-screen terminal interface built on prompt_toolkit.

Usage:
    uv run python examples/gait/generate_csv_tui.py
    legwheel tui

Keys:
    ↑ / ↓ / Tab / Shift+Tab   Navigate fields
    j / k                      Navigate fields (vim)
    ← / → / Space              Toggle booleans, cycle choices
    h / l                      Toggle / cycle (vim)
    gg / G                     Jump to first / last field (vim)
    Enter / i / a              Begin editing a text field
    Enter / Esc                Confirm / cancel edit
    F5  / Ctrl+G               Generate CSV
    1 / 2                      Switch Gait / Lean mode
    F1                         Show / hide key-binding help
    Ctrl+L                     Clear log
    q / Ctrl+C / Ctrl+Q        Quit
"""

import argparse
import math
import os
import re
import subprocess
import sys
import threading
from dataclasses import dataclass, field as dc_field
from typing import Any, Dict, List, Optional

from prompt_toolkit import Application
from prompt_toolkit.buffer import Buffer
from prompt_toolkit.document import Document
from prompt_toolkit.filters import Condition
from prompt_toolkit.key_binding import KeyBindings
from prompt_toolkit.layout import Layout
from prompt_toolkit.layout.containers import (
    ConditionalContainer, Float, FloatContainer, HSplit, VSplit, Window,
)
from prompt_toolkit.layout.controls import BufferControl, FormattedTextControl
from prompt_toolkit.layout.dimension import D
from prompt_toolkit.styles import Style

SCRIPT_DIR       = os.path.dirname(os.path.abspath(__file__))
BASE_DIR         = os.path.abspath(os.path.join(SCRIPT_DIR, "../.."))
GAIT_SCRIPT      = os.path.join(SCRIPT_DIR, "generate_hardware_csv.py")
LEAN_SCRIPT      = os.path.join(SCRIPT_DIR, "generate_lean_csv.py")
TRANSFORM_SCRIPT = os.path.join(SCRIPT_DIR, "generate_transform_csv.py")

GAIT_CHOICES = ["Walk", "Trot", "Pace", "Bound", "Pronk"]

HELP_TEXT = """\
 ╔══════════════ Key Bindings ═══════════════╗
 ║                                           ║
 ║  Navigation                               ║
 ║  ↑ / ↓            Prev / Next field      ║
 ║  j / k            Prev / Next  (vim)     ║
 ║  Tab / Shift+Tab  Prev / Next field      ║
 ║  gg               Jump to first field    ║
 ║  G                Jump to last field     ║
 ║                                           ║
 ║  Toggle / Cycle                           ║
 ║  ← / → / Space    Toggle / cycle fwd     ║
 ║  h / l            Toggle / cycle  (vim)  ║
 ║                                           ║
 ║  Edit text field                          ║
 ║  Enter / i / a    Enter edit mode        ║
 ║  Enter            Confirm value          ║
 ║  Esc              Cancel edit            ║
 ║                                           ║
 ║  Commands                                 ║
 ║  F5 / Ctrl+G      Generate CSV           ║
 ║  1 / 2 / 3        Gait/Lean/Transform    ║
 ║  Ctrl+L           Clear log              ║
 ║  F1 / Esc         Close this help        ║
 ║  q / Ctrl+C       Quit                   ║
 ║                                           ║
 ╚═══════════════════════════════════════════╝
"""

# ──────────────────────────────────────────────────────────
# Field model
# ──────────────────────────────────────────────────────────

@dataclass
class Field:
    label:   str
    key:     str
    type:    str            # "float" | "int" | "text" | "bool" | "choice"
    default: Any
    choices: List[str] = dc_field(default_factory=list)
    section: Optional[str] = None   # separator header above this field

    def __post_init__(self):
        self._val: Any = bool(self.default) if self.type == "bool" else str(self.default)

    def get(self) -> Any:
        return self._val

    def set(self, v: Any):
        self._val = bool(v) if self.type == "bool" else str(v)

    def display(self) -> str:
        if self.type == "bool":
            return "[x]" if self._val else "[ ]"
        if self.type == "choice":
            return f"◄ {self._val} ►"
        return str(self._val)

    def toggle(self):
        if self.type == "bool":
            self._val = not self._val

    def cycle(self, d: int = 1):
        if self.type == "choice" and self.choices:
            i = self.choices.index(self._val) if self._val in self.choices else 0
            self._val = self.choices[(i + d) % len(self.choices)]

    @property
    def is_text(self) -> bool:
        return self.type in ("float", "int", "text")


# ──────────────────────────────────────────────────────────
# Field definitions
# ──────────────────────────────────────────────────────────

def _gait_fields() -> List[Field]:
    return [
        Field("Gait Type",    "gait",        "choice", "Walk",        GAIT_CHOICES),
        Field("Vx (m/s)",     "vx",          "float",  "0.10"),
        Field("Vy (m/s)",     "vy",          "float",  "0.00"),
        Field("Wz (rad/s)",   "wz",          "float",  "0.00"),
        Field("Height (m)",   "height",      "float",  "0.25"),
        Field("Step H (m)",   "step",        "float",  "0.04"),
        Field("Period (s)",   "period",      "float",  "4.0"),
        Field("Cycles",       "cycles",      "int",    "10"),
        Field("dt (s)",       "dt",          "float",  "0.001"),
        Field("Output Dir",   "outdir",      "text",   "outputs/csv"),
        Field("Launch Enable","launch",      "bool",   False,         section="── Launch Control ──"),
        Field("Ramp Mode",    "ramp_mode",   "choice", "cycles",      ["cycles", "seconds"]),
        Field("Ramp Cycles",  "ramp_cycles", "int",    "3"),
        Field("Ramp Seconds", "ramp_secs",   "float",  "3.0"),
        Field("Ramp Floor",   "ramp_floor",  "float",  "0.10"),
    ]


def _lean_fields() -> List[Field]:
    return [
        Field("Roll (°)",      "roll",    "float", "0.0"),
        Field("Pitch (°)",     "pitch",   "float", "0.0"),
        Field("Yaw (°)",       "yaw",     "float", "0.0"),
        Field("X offset (m)",  "x",       "float", "0.0"),
        Field("Y offset (m)",  "y",       "float", "0.0"),
        Field("Height (m)",    "height",  "float", "0.30"),
        Field("Compensation",  "comp",    "float", "0.0"),
        Field("Steps / Seg",   "steps",   "int",   "500"),
        Field("Repeats",       "repeats", "int",   "1"),
        Field("Prep (s)",      "prep",    "float", "5.0"),
        Field("dt (s)",        "dt",      "float", "0.001"),
        Field("Return Neutral","ret",     "bool",  True),
        Field("Rock ±",        "rock",    "bool",  False),
        Field("Output Dir",    "outdir",  "text",  "outputs/csv"),
    ]


def _transform_fields() -> List[Field]:
    return [
        Field("Target θ (°)",  "t_theta", "float", "45.0",  section="── Target Pose (all legs) ──"),
        Field("Target β (°)",  "t_beta",  "float", "0.0"),
        Field("Target γ (°)",  "t_gamma", "float", "0.0"),
        Field("Start θ (°)",   "s_theta", "float", "17.0",  section="── Start Pose (all legs) ──"),
        Field("Start β (°)",   "s_beta",  "float", "0.0"),
        Field("Start γ (°)",   "s_gamma", "float", "0.0"),
        Field("Duration (s)",  "dur",     "float", "5.0",   section="── Options ──"),
        Field("Hold (s)",      "hold",    "float", "0.0"),
        Field("dt (s)",        "dt",      "float", "0.001"),
        Field("Output Dir",    "outdir",  "text",  "outputs/csv"),
    ]


# ──────────────────────────────────────────────────────────
# TUI Application
# ──────────────────────────────────────────────────────────

LABEL_W = 15    # label column width (characters)


class CSVGeneratorTUI:
    def __init__(self, mode: Optional[str] = None, overrides: Optional[Dict[str, str]] = None):
        self.mode        = mode if mode in ("gait", "lean", "transform") else "gait"
        self.all_fields  = {"gait": _gait_fields(), "lean": _lean_fields(), "transform": _transform_fields()}
        if overrides:
            for fs in self.all_fields.values():
                for f in fs:
                    if f.key in overrides:
                        f.set(overrides[f.key])
        self.focus_idx   = 0
        self.log_lines: List[str] = ["  Ready — press F5 to generate."]
        self.is_running  = False
        self.last_path   = ""
        self.status      = "Idle"
        self.edit_mode   = False
        self.show_help   = False
        self.edit_buffer = Buffer(name="edit", multiline=False)
        self._app        = self._build_app()

    # ── field helpers ─────────────────────────────────────

    def _fields(self) -> List[Field]:
        return self.all_fields[self.mode]

    def _focused(self) -> Optional[Field]:
        fs = self._fields()
        return fs[self.focus_idx] if 0 <= self.focus_idx < len(fs) else None

    def _fval(self, key: str, default: Any = "") -> Any:
        for f in self._fields():
            if f.key == key:
                return f.get()
        return default

    # ── renderers ─────────────────────────────────────────

    def _render_title(self):
        gs = "class:tab.on" if self.mode == "gait"      else "class:tab"
        ls = "class:tab.on" if self.mode == "lean"      else "class:tab"
        ts = "class:tab.on" if self.mode == "transform" else "class:tab"
        return [
            ("class:title", "  CorgiRobot CSV Generator   "),
            (gs, " 1·Gait "),
            ("class:title", " "),
            (ls, " 2·Lean "),
            ("class:title", " "),
            (ts, " 3·Transform "),
        ]

    def _render_fields(self):
        out = []
        for i, f in enumerate(self._fields()):
            if f.section:
                out.append(("class:sep", f"\n  {f.section}\n\n"))
            foc = (i == self.focus_idx)
            ls  = "class:lbl.f" if foc else "class:lbl"
            vs  = "class:val.f" if foc else "class:val"
            cur = " ▶ " if foc else "   "
            out += [(ls, f"{cur}{f.label.ljust(LABEL_W)}"), (vs, f" {f.display()}\n")]
        out += [
            ("class:hint", "\n  Tab/↑↓/jk  Navigate     ←→/Spc/hl  Toggle\n"),
            ("class:hint",   "  Enter/i/a  Edit text    F5          Generate\n"),
            ("class:hint",   "  gg/G       First/Last   1/2         Gait/Lean\n"),
            ("class:hint",   "  F1         Key help     q           Quit\n"),
        ]
        return out

    def _render_right(self):
        out = [("class:sep", " ── Summary ──\n\n")]
        if self.mode == "gait":
            out += self._gait_summary()
        elif self.mode == "lean":
            out += self._lean_summary()
        else:
            out += self._transform_summary()
        out.append(("class:sep", "\n ── Log ──\n"))
        for line in self.log_lines[-28:]:
            s = "class:log.ok" if line.startswith("✓") else (
                "class:log.err" if line.startswith("✗") else "class:log")
            out.append((s, f"{line}\n"))
        return out

    def _gait_summary(self):
        out = []
        try:
            gait   = self._fval("gait",   "Walk")
            vx     = float(self._fval("vx",     "0"))
            vy     = float(self._fval("vy",     "0"))
            wz     = float(self._fval("wz",     "0"))
            h      = float(self._fval("height", "0.25"))
            period = float(self._fval("period", "4"))
            cycles = int(float(self._fval("cycles", "10")))
            dt     = float(self._fval("dt",     "0.001"))
            launch = self._fval("launch", False)

            gait_s  = cycles * period
            total_s = 5.0 + gait_s

            out += [
                ("class:sum",    f" Gait    {gait}\n"),
                ("class:sum",    f" Speed   Vx={vx:+.2f}  Vy={vy:+.2f}  Wz={wz:+.2f}\n"),
                ("class:sum",    f" Height  {h:.3f} m\n"),
                ("class:sum",    f" Period  {period:.2f} s\n"),
                ("class:sum",    f" Cycles  {cycles}  →  {gait_s:.1f} s\n"),
                ("class:sum",    f" Prep    5.0 s (fixed)\n"),
            ]
            if launch:
                rmode = self._fval("ramp_mode", "cycles")
                if rmode == "seconds":
                    secs   = float(self._fval("ramp_secs", "3"))
                    n_ramp = max(1, math.ceil(secs / max(period, 1e-9)))
                else:
                    n_ramp = int(float(self._fval("ramp_cycles", "3")))
                ramp_s   = n_ramp * period
                total_s += ramp_s
                out.append(("class:sum", f" Launch  {n_ramp} cy → {ramp_s:.1f} s\n"))

            out.append(("class:sum.hi", f" Total   {total_s:.1f} s  ({int(total_s / max(dt, 1e-9)):,} frames)\n"))
        except Exception:
            out.append(("class:log.err", " (invalid params)\n"))
        return out

    def _lean_summary(self):
        out = []
        try:
            roll    = float(self._fval("roll",    "0"))
            pitch   = float(self._fval("pitch",   "0"))
            yaw     = float(self._fval("yaw",     "0"))
            x       = float(self._fval("x",       "0"))
            y       = float(self._fval("y",       "0"))
            h       = float(self._fval("height",  "0.3"))
            steps   = int(float(self._fval("steps",   "500")))
            repeats = int(float(self._fval("repeats", "1")))
            dt      = float(self._fval("dt",      "0.001"))
            prep    = float(self._fval("prep",    "5"))
            ret     = self._fval("ret", True)
            rock    = self._fval("rock", False)

            cycle_segs = 4 if rock else 2
            total_segs = repeats * cycle_segs - (0 if ret else 1)
            total_s    = prep + steps * total_segs * dt

            out += [
                ("class:sum",    f" Roll    {roll:+.1f}°\n"),
                ("class:sum",    f" Pitch   {pitch:+.1f}°\n"),
                ("class:sum",    f" Yaw     {yaw:+.1f}°\n"),
                ("class:sum",    f" X off   {x:+.4f} m\n"),
                ("class:sum",    f" Y off   {y:+.4f} m\n"),
                ("class:sum",    f" Height  {h:.3f} m\n"),
                ("class:sum",    f" Rock±   {'yes' if rock else 'no'}\n"),
                ("class:sum",    f" Steps   {steps} × {total_segs} seg ({repeats}x)\n"),
                ("class:sum",    f" Prep    {prep:.1f} s\n"),
                ("class:sum.hi", f" Total   {total_s:.1f} s  ({int(total_s / max(dt, 1e-9)):,} frames)\n"),
            ]
        except Exception:
            out.append(("class:log.err", " (invalid params)\n"))
        return out

    def _transform_summary(self):
        out = []
        try:
            t_theta = float(self._fval("t_theta", "45"))
            t_beta  = float(self._fval("t_beta",  "0"))
            t_gamma = float(self._fval("t_gamma", "0"))
            s_theta = float(self._fval("s_theta", "17"))
            s_beta  = float(self._fval("s_beta",  "0"))
            s_gamma = float(self._fval("s_gamma", "0"))
            dur     = float(self._fval("dur",     "5"))
            hold    = float(self._fval("hold",    "0"))
            dt      = float(self._fval("dt",      "0.001"))

            total_s = dur + hold
            out += [
                ("class:sum",    f" Target  θ={t_theta:.1f}° β={t_beta:.1f}° γ={t_gamma:.1f}°\n"),
                ("class:sum",    f" Start   θ={s_theta:.1f}° β={s_beta:.1f}° γ={s_gamma:.1f}°\n"),
                ("class:sum",    f" Duration {dur:.1f} s\n"),
                ("class:sum",    f" Hold    {hold:.1f} s\n"),
                ("class:sum.hi", f" Total   {total_s:.1f} s  ({int(total_s / max(dt, 1e-9)):,} frames)\n"),
            ]
        except Exception:
            out.append(("class:log.err", " (invalid params)\n"))
        return out

    def _render_status(self):
        if self.is_running:
            return [("class:st.run", f"  ⏳  Running…  {self.status}")]
        fname = os.path.basename(self.last_path) or "—"
        return [("class:st", f"  ●  {self.status}  │  {fname}  │  Ctrl+L clear log")]

    def _render_edit_label(self):
        f = self._focused()
        label = f.label if f else "?"
        return [("class:ed.lbl", f"  ✏  {label}: ")]

    def _render_help(self):
        return [("class:help", HELP_TEXT)]

    # ── layout construction ───────────────────────────────

    def _build_app(self) -> Application:
        in_edit  = Condition(lambda: self.edit_mode)
        not_edit = Condition(lambda: not self.edit_mode)

        self._fields_win = Window(
            content=FormattedTextControl(self._render_fields, focusable=True),
            width=D(preferred=46, min=32),
            wrap_lines=False,
        )

        right_win = Window(
            content=FormattedTextControl(self._render_right),
            width=D(preferred=44, min=28),
            wrap_lines=False,
        )

        # Bottom bar: status OR edit area (mutually exclusive via ConditionalContainer)
        status_bar = ConditionalContainer(
            content=Window(
                content=FormattedTextControl(self._render_status),
                height=1, style="class:st",
            ),
            filter=not_edit,
        )
        edit_bar = ConditionalContainer(
            content=VSplit([
                Window(
                    content=FormattedTextControl(self._render_edit_label),
                    width=26, height=1, style="class:ed.lbl",
                ),
                Window(
                    content=BufferControl(buffer=self.edit_buffer),
                    height=1, style="class:ed",
                ),
            ]),
            filter=in_edit,
        )

        help_filter = Condition(lambda: self.show_help)

        layout = Layout(
            FloatContainer(
                content=HSplit([
                    Window(
                        content=FormattedTextControl(self._render_title),
                        height=1, style="class:title",
                    ),
                    Window(height=1, char="─", style="class:border"),
                    VSplit([
                        self._fields_win,
                        Window(width=1, char="│", style="class:border"),
                        right_win,
                    ]),
                    Window(height=1, char="─", style="class:border"),
                    edit_bar,
                    status_bar,
                ]),
                floats=[
                    Float(
                        content=ConditionalContainer(
                            content=Window(
                                content=FormattedTextControl(self._render_help),
                                style="class:help",
                            ),
                            filter=help_filter,
                        ),
                        top=2, left=4, width=47, height=29,
                    ),
                ],
            ),
            focused_element=self._fields_win,
        )

        style = Style.from_dict({
            "title":    "bg:#002244 #cce0ff bold",
            "tab":      "bg:#002244 #446688",
            "tab.on":   "bg:#002244 #ffffff bold underline",
            "border":   "#334455",
            "lbl":      "#99aabb",
            "lbl.f":    "#ffffff bold",
            "val":      "#556677",
            "val.f":    "#00ccff bold",
            "sep":      "#446688 italic",
            "hint":     "#334455",
            "sum":      "#8899aa",
            "sum.hi":   "#00ccff bold",
            "log":      "#445566",
            "log.ok":   "#22aa55",
            "log.err":  "#cc3344",
            "ed":       "bg:#00213f #ffffff",
            "ed.lbl":   "bg:#00213f #7799bb",
            "st":       "bg:#0d0d0d #445566",
            "st.run":   "bg:#0d0d0d #ffaa00 bold",
            "help":     "bg:#001830 #99ccff",
        })

        return Application(
            layout=layout,
            key_bindings=self._build_kb(),
            style=style,
            full_screen=True,
            mouse_support=False,
        )

    # ── key bindings ──────────────────────────────────────

    def _build_kb(self) -> KeyBindings:
        kb       = KeyBindings()
        not_edit = Condition(lambda: not self.edit_mode)
        in_edit  = Condition(lambda: self.edit_mode)
        not_run  = Condition(lambda: not self.is_running)

        not_help = Condition(lambda: not self.show_help)
        in_help  = Condition(lambda: self.show_help)
        nav_mode = not_edit & not_help   # navigation: no edit, no help overlay

        # Quit — always active
        @kb.add("c-c")
        @kb.add("c-q")
        def _quit(ev): ev.app.exit()

        @kb.add("q", filter=nav_mode)
        def _q(ev): ev.app.exit()

        # Help overlay — F1 toggles; Esc closes when open
        @kb.add("f1")
        def _toggle_help(ev):
            self.show_help = not self.show_help
            ev.app.invalidate()

        @kb.add("escape", filter=in_help & not_edit)
        def _close_help(ev):
            self.show_help = False
            ev.app.invalidate()

        # Mode switch
        @kb.add("1", filter=nav_mode)
        def _m1(ev):
            self.mode = "gait"; self.focus_idx = 0; ev.app.invalidate()

        @kb.add("2", filter=nav_mode)
        def _m2(ev):
            self.mode = "lean"; self.focus_idx = 0; ev.app.invalidate()

        @kb.add("3", filter=nav_mode)
        def _m3(ev):
            self.mode = "transform"; self.focus_idx = 0; ev.app.invalidate()

        # Navigation — arrow keys + vim j/k
        @kb.add("down",  filter=nav_mode)
        @kb.add("tab",   filter=nav_mode)
        @kb.add("j",     filter=nav_mode)
        def _next(ev):
            self.focus_idx = (self.focus_idx + 1) % len(self._fields())
            ev.app.invalidate()

        @kb.add("up",    filter=nav_mode)
        @kb.add("s-tab", filter=nav_mode)
        @kb.add("k",     filter=nav_mode)
        def _prev(ev):
            self.focus_idx = (self.focus_idx - 1) % len(self._fields())
            ev.app.invalidate()

        # vim gg → first field, G → last field
        @kb.add("g", "g", filter=nav_mode)
        def _first(ev):
            self.focus_idx = 0
            ev.app.invalidate()

        @kb.add("G", filter=nav_mode)
        def _last(ev):
            self.focus_idx = len(self._fields()) - 1
            ev.app.invalidate()

        # Toggle / cycle — arrow keys + vim h/l
        @kb.add("right", filter=nav_mode)
        @kb.add("space", filter=nav_mode)
        @kb.add("l",     filter=nav_mode)
        def _fwd(ev):
            f = self._focused()
            if f:
                if f.type == "bool":
                    f.toggle()
                elif f.type == "choice":
                    f.cycle(1)
            ev.app.invalidate()

        @kb.add("left", filter=nav_mode)
        @kb.add("h",    filter=nav_mode)
        def _bwd(ev):
            f = self._focused()
            if f:
                if f.type == "bool":
                    f.toggle()
                elif f.type == "choice":
                    f.cycle(-1)
            ev.app.invalidate()

        # Enter edit mode — Enter, i, a (vim insert)
        def _start_edit(ev):
            f = self._focused()
            if not f:
                return
            if f.is_text:
                val = f.get()
                self.edit_buffer.set_document(
                    Document(text=val, cursor_position=len(val)), bypass_readonly=True
                )
                self.edit_mode = True
                ev.app.layout.focus(self.edit_buffer)
            elif f.type == "bool":
                f.toggle()
            elif f.type == "choice":
                f.cycle(1)
            ev.app.invalidate()

        @kb.add("enter", filter=nav_mode)
        @kb.add("i",     filter=nav_mode)
        @kb.add("a",     filter=nav_mode)
        def _enter_nav(ev): _start_edit(ev)

        # Confirm edit
        @kb.add("enter", filter=in_edit, eager=True)
        def _confirm(ev):
            f = self._focused()
            if f and f.is_text:
                f.set(self.edit_buffer.text)
            self.edit_mode = False
            ev.app.layout.focus(self._fields_win)
            ev.app.invalidate()

        # Cancel edit
        @kb.add("escape", filter=in_edit, eager=True)
        def _cancel(ev):
            self.edit_mode = False
            ev.app.layout.focus(self._fields_win)
            ev.app.invalidate()

        # Generate
        @kb.add("f5",  filter=nav_mode & not_run)
        @kb.add("c-g", filter=nav_mode & not_run)
        def _gen(ev):
            threading.Thread(target=self._run_gen, args=(ev.app,), daemon=True).start()

        # Clear log
        @kb.add("c-l", filter=not_edit)
        def _clear(ev):
            self.log_lines.clear()
            ev.app.invalidate()

        return kb

    # ── generation ────────────────────────────────────────

    def _log(self, line: str, app=None):
        self.log_lines.append(line)
        if len(self.log_lines) > 500:
            self.log_lines = self.log_lines[-200:]
        if app:
            app.invalidate()

    def _run_gen(self, app):
        self.is_running = True
        self.status = "Starting…"
        app.invalidate()
        try:
            cmd = self._build_cmd()
            self._log(f"$ {os.path.basename(cmd[1])}", app)
            self._log("─" * 38, app)

            proc = subprocess.Popen(
                cmd, cwd=BASE_DIR,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1,
            )
            if proc.stdout:
                for raw in proc.stdout:
                    line = raw.rstrip()
                    if line:
                        self._log(f"  {line}", app)
                        m = re.search(r"Saved to\s*[:\-]\s*(.+\.csv)", line)
                        if m:
                            self.last_path = m.group(1).strip()
                proc.stdout.close()
            rc = proc.wait()
            if rc == 0:
                fname = os.path.basename(self.last_path)
                self._log(f"✓ Saved: {fname}", app)
                self.status = f"Done — {fname}"
            else:
                self._log(f"✗ Failed (exit {rc})", app)
                self.status = f"Failed (exit {rc})"
        except Exception as e:
            self._log(f"✗ {e}", app)
            self.status = f"Error: {e}"
        finally:
            self.is_running = False
            app.invalidate()

    def _build_cmd(self) -> List[str]:
        if self.mode == "gait":
            return self._gait_cmd()
        elif self.mode == "lean":
            return self._lean_cmd()
        else:
            return self._transform_cmd()

    def _gait_cmd(self) -> List[str]:
        cmd = [
            sys.executable, GAIT_SCRIPT,
            "-g",  self._fval("gait",   "Walk"),
            "-vx", self._fval("vx",     "0.10"),
            "-vy", self._fval("vy",     "0.00"),
            "-wz", self._fval("wz",     "0.00"),
            "-z",  self._fval("height", "0.25"),
            "-s",  self._fval("step",   "0.04"),
            "-p",  self._fval("period", "4.0"),
            "-c",  self._fval("cycles", "10"),
            "-dt", self._fval("dt",     "0.001"),
            "-o",  self._fval("outdir", "outputs/csv"),
        ]
        if self._fval("launch", False):
            rmode = self._fval("ramp_mode", "cycles")
            if rmode == "seconds":
                try:
                    secs   = float(self._fval("ramp_secs", "3.0"))
                    period = float(self._fval("period", "4.0"))
                    n      = max(1, math.ceil(secs / max(period, 1e-9)))
                except (ValueError, ZeroDivisionError):
                    n = 3
                n_str = str(n)
            else:
                n_str = self._fval("ramp_cycles", "3")
            cmd += [
                "--launch",
                "--ramp-cycles", n_str,
                "--ramp-floor",  self._fval("ramp_floor", "0.10"),
            ]
        return cmd

    def _lean_cmd(self) -> List[str]:
        cmd = [
            sys.executable, LEAN_SCRIPT,
            "--roll",         self._fval("roll",    "0.0"),
            "--pitch",        self._fval("pitch",   "0.0"),
            "--yaw",          self._fval("yaw",     "0.0"),
            "--x",            self._fval("x",       "0.0"),
            "--y",            self._fval("y",       "0.0"),
            "-z",             self._fval("height",  "0.30"),
            "--compensation", self._fval("comp",    "0.0"),
            "-n",             self._fval("steps",   "500"),
            "--repeats",      self._fval("repeats", "1"),
            "-dt",            self._fval("dt",      "0.001"),
            "--prep",         self._fval("prep",    "5.0"),
            "-o",             self._fval("outdir",  "outputs/csv"),
        ]
        if not self._fval("ret", True):
            cmd.append("--no-return")
        if self._fval("rock", False):
            cmd.append("--rock")
        return cmd

    def _transform_cmd(self) -> List[str]:
        t  = self._fval("t_theta", "45.0")
        b  = self._fval("t_beta",  "0.0")
        g  = self._fval("t_gamma", "0.0")
        st = self._fval("s_theta", "17.0")
        sb = self._fval("s_beta",  "0.0")
        sg = self._fval("s_gamma", "0.0")
        outdir = self._fval("outdir", "outputs/csv")
        import time as _time
        fname = os.path.join(outdir, f"transform_pose_{_time.strftime('%Y%m%d_%H%M%S')}.csv")
        return [
            sys.executable, TRANSFORM_SCRIPT,
            "--theta",       t,  t,  t,  t,
            "--beta",        b,  b,  b,  b,
            "--gamma",       g,  g,  g,  g,
            "--start-theta", st, st, st, st,
            "--start-beta",  sb, sb, sb, sb,
            "--start-gamma", sg, sg, sg, sg,
            "--duration",    self._fval("dur",  "5.0"),
            "--hold",        self._fval("hold", "0.0"),
            "--dt",          self._fval("dt",   "0.001"),
            "-o",            fname,
        ]

    # ── entry point ───────────────────────────────────────

    def run(self):
        self._app.run()


def _make_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="legwheel tui",
        description="CorgiRobot full-screen TUI CSV generator (Gait / Lean / Transform modes)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("-m", "--mode", choices=["gait", "lean", "transform"], default=None,
                   help="Starting mode")
    # ── Gait params ────────────────────────────────
    grp_g = p.add_argument_group("Gait defaults")
    grp_g.add_argument("-g", "--gait", choices=GAIT_CHOICES, default=None,
                       metavar="TYPE", help="Gait type")
    grp_g.add_argument("-vx", dest="vx", type=float, default=None,
                       metavar="M/S",  help="Forward velocity (m/s)")
    grp_g.add_argument("-vy", dest="vy", type=float, default=None,
                       metavar="M/S",  help="Lateral velocity (m/s)")
    grp_g.add_argument("-wz", dest="wz", type=float, default=None,
                       metavar="RAD/S", help="Yaw rate (rad/s)")
    grp_g.add_argument("-z", "--height", type=float, default=None,
                       metavar="M",    help="Stand height (m)")
    grp_g.add_argument("-s", "--step", type=float, default=None,
                       metavar="M",    help="Step height (m)")
    grp_g.add_argument("-p", "--period", type=float, default=None,
                       metavar="S",    help="Gait period (s)")
    grp_g.add_argument("-c", "--cycles", type=int, default=None,
                       metavar="N",    help="Number of gait cycles")
    grp_g.add_argument("-dt", dest="dt", type=float, default=None,
                       metavar="S",    help="Time step (s)")
    # ── Lean params ────────────────────────────────
    grp_l = p.add_argument_group("Lean defaults")
    grp_l.add_argument("--roll",  type=float, default=None, metavar="DEG")
    grp_l.add_argument("--pitch", type=float, default=None, metavar="DEG")
    grp_l.add_argument("--yaw",   type=float, default=None, metavar="DEG")
    grp_l.add_argument("--comp",  type=float, default=None, metavar="M/RAD",
                       help="Height compensation (m/rad)")
    grp_l.add_argument("-n", "--steps", type=int, default=None, metavar="N",
                       help="Steps per segment")
    grp_l.add_argument("--repeats", type=int, default=None, metavar="N",
                       help="Lean repeat count")
    grp_l.add_argument("--prep", type=float, default=None, metavar="S",
                       help="Prep duration (s)")
    # ── Transform params ───────────────────────────
    grp_t = p.add_argument_group("Transform defaults")
    grp_t.add_argument("--t-theta", type=float, default=None, metavar="DEG",
                       help="Target theta for all legs (deg)")
    grp_t.add_argument("--t-beta",  type=float, default=None, metavar="DEG",
                       help="Target beta for all legs (deg)")
    grp_t.add_argument("--t-gamma", type=float, default=None, metavar="DEG",
                       help="Target gamma for all legs (deg)")
    grp_t.add_argument("--s-theta", type=float, default=None, metavar="DEG",
                       help="Start theta for all legs (deg)")
    grp_t.add_argument("--dur",  type=float, default=None, metavar="S",
                       help="Transform duration (s)")
    grp_t.add_argument("--hold", type=float, default=None, metavar="S",
                       help="Hold time after transform (s)")
    # ── Shared ─────────────────────────────────────
    p.add_argument("-o", "--outdir", type=str, default=None,
                   metavar="DIR", help="Output directory")
    return p


# Maps argparse dest name → Field key (identical here, but explicit for clarity)
_DEST_TO_KEY = {
    "gait": "gait", "vx": "vx", "vy": "vy", "wz": "wz",
    "height": "height", "step": "step", "period": "period",
    "cycles": "cycles", "dt": "dt", "outdir": "outdir",
    "roll": "roll", "pitch": "pitch", "yaw": "yaw",
    "comp": "comp", "steps": "steps", "repeats": "repeats", "prep": "prep",
    "t_theta": "t_theta", "t_beta": "t_beta", "t_gamma": "t_gamma",
    "s_theta": "s_theta", "dur": "dur", "hold": "hold",
}


def main(argv: Optional[List[str]] = None):
    args = _make_parser().parse_args(argv)
    overrides = {
        key: str(getattr(args, dest))
        for dest, key in _DEST_TO_KEY.items()
        if getattr(args, dest, None) is not None
    }
    CSVGeneratorTUI(mode=args.mode, overrides=overrides or None).run()


if __name__ == "__main__":
    main()

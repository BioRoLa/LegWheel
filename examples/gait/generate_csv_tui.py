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

import math
import os
import re
import subprocess
import sys
import threading
from dataclasses import dataclass, field as dc_field
from typing import Any, List, Optional

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

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
BASE_DIR    = os.path.abspath(os.path.join(SCRIPT_DIR, "../.."))
GAIT_SCRIPT = os.path.join(SCRIPT_DIR, "generate_hardware_csv.py")
LEAN_SCRIPT = os.path.join(SCRIPT_DIR, "generate_lean_csv.py")

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
 ║  1 / 2            Gait / Lean mode       ║
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
        Field("Roll (°)",      "roll",   "float", "0.0"),
        Field("Pitch (°)",     "pitch",  "float", "0.0"),
        Field("Yaw (°)",       "yaw",    "float", "0.0"),
        Field("Height (m)",    "height", "float", "0.30"),
        Field("Compensation",  "comp",   "float", "0.0"),
        Field("Steps / Seg",   "steps",  "int",   "500"),
        Field("Prep (s)",      "prep",   "float", "3.0"),
        Field("dt (s)",        "dt",     "float", "0.001"),
        Field("Return Neutral","ret",    "bool",  True),
        Field("Output Dir",    "outdir", "text",  "outputs/csv"),
    ]


# ──────────────────────────────────────────────────────────
# TUI Application
# ──────────────────────────────────────────────────────────

LABEL_W = 15    # label column width (characters)


class CSVGeneratorTUI:
    def __init__(self):
        self.mode        = "gait"
        self.all_fields  = {"gait": _gait_fields(), "lean": _lean_fields()}
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
        gs = "class:tab.on" if self.mode == "gait" else "class:tab"
        ls = "class:tab.on" if self.mode == "lean" else "class:tab"
        return [
            ("class:title", "  CorgiRobot CSV Generator   "),
            (gs, " 1·Gait "),
            ("class:title", "   "),
            (ls, " 2·Lean "),
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
        out += self._gait_summary() if self.mode == "gait" else self._lean_summary()
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
            roll  = float(self._fval("roll",   "0"))
            pitch = float(self._fval("pitch",  "0"))
            yaw   = float(self._fval("yaw",    "0"))
            h     = float(self._fval("height", "0.3"))
            steps = int(float(self._fval("steps", "500")))
            dt    = float(self._fval("dt",     "0.001"))
            prep  = float(self._fval("prep",   "3"))
            ret   = self._fval("ret", True)

            segs    = 2 if ret else 1
            total_s = prep + steps * segs * dt

            out += [
                ("class:sum",    f" Roll    {roll:+.1f}°\n"),
                ("class:sum",    f" Pitch   {pitch:+.1f}°\n"),
                ("class:sum",    f" Yaw     {yaw:+.1f}°\n"),
                ("class:sum",    f" Height  {h:.3f} m\n"),
                ("class:sum",    f" Steps   {steps} × {segs} seg\n"),
                ("class:sum",    f" Prep    {prep:.1f} s\n"),
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
        return self._gait_cmd() if self.mode == "gait" else self._lean_cmd()

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
            "--roll",         self._fval("roll",  "0.0"),
            "--pitch",        self._fval("pitch", "0.0"),
            "--yaw",          self._fval("yaw",   "0.0"),
            "-z",             self._fval("height","0.30"),
            "--compensation", self._fval("comp",  "0.0"),
            "-n",             self._fval("steps", "500"),
            "-dt",            self._fval("dt",    "0.001"),
            "--prep",         self._fval("prep",  "3.0"),
            "-o",             self._fval("outdir","outputs/csv"),
        ]
        if not self._fval("ret", True):
            cmd.append("--no-return")
        return cmd

    # ── entry point ───────────────────────────────────────

    def run(self):
        self._app.run()


def main():
    CSVGeneratorTUI().run()


if __name__ == "__main__":
    main()

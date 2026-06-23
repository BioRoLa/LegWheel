"""
Interactive Tkinter UI for CorgiRobot Lean Pose CSV Generator.

Wraps generate_lean_csv.py with sliders and live parameter readouts.
Launch via:  uv run python examples/gait/generate_lean_csv_ui.py
        or:  legwheel lean-ui
"""

import os
import sys
import re
import queue
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
GENERATOR_SCRIPT = os.path.join(BASE_DIR, "examples", "gait", "generate_lean_csv.py")

ROLL_LIMIT  = 10.0   # degrees
PITCH_LIMIT = 8.0
YAW_LIMIT   = 15.0
HEIGHT_MIN  = 0.18   # m — theta ~75° at this height (low crouch)
HEIGHT_MAX  = 0.32   # m — theta ~159° (just below 160° upper limit)


class SliderRow:
    """Label + Scale + Entry for one angle parameter."""

    def __init__(self, parent, label, var: tk.DoubleVar, lo, hi, row, fmt="{:.1f}°"):
        self.var = var
        self.fmt = fmt

        ttk.Label(parent, text=label, width=22, anchor="e").grid(
            row=row, column=0, sticky=tk.E, padx=(0, 6), pady=4)

        self.scale = ttk.Scale(parent, from_=lo, to=hi, variable=var,
                               orient=tk.HORIZONTAL, length=220,
                               command=self._on_scale)
        self.scale.grid(row=row, column=1, sticky=tk.EW, pady=4)

        self.entry = ttk.Entry(parent, width=8)
        self.entry.grid(row=row, column=2, padx=(8, 0), pady=4)
        self.entry.bind("<Return>", self._on_entry)
        self.entry.bind("<FocusOut>", self._on_entry)

        var.trace_add("write", lambda *_: self._refresh_entry())
        self._refresh_entry()

    def _refresh_entry(self):
        val = self.var.get()
        self.entry.delete(0, tk.END)
        self.entry.insert(0, self.fmt.format(val))

    def _on_scale(self, _=None):
        self._refresh_entry()

    def _on_entry(self, _=None):
        try:
            raw = self.entry.get().strip().rstrip("°")
            self.var.set(float(raw))
        except ValueError:
            self._refresh_entry()


class LeanCSVUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("CorgiRobot Lean Pose CSV Generator")
        self.geometry("680x740")
        self.configure(padx=20, pady=20)
        self.resizable(False, False)

        style = ttk.Style(self)
        style.theme_use("clam")

        # --- Parameter variables ---
        self.var_roll   = tk.DoubleVar(value=0.0)
        self.var_pitch  = tk.DoubleVar(value=0.0)
        self.var_yaw    = tk.DoubleVar(value=0.0)
        self.var_height = tk.DoubleVar(value=0.30)
        self.var_comp   = tk.DoubleVar(value=0.0)
        self.var_steps  = tk.IntVar(value=500)
        self.var_return = tk.BooleanVar(value=True)
        self.var_dt     = tk.DoubleVar(value=0.001)
        self.var_prep   = tk.DoubleVar(value=3.0)
        self.var_outdir = tk.StringVar(value="outputs/csv")

        self.is_running = False
        self.proc_queue: queue.Queue = queue.Queue()
        self.t_start = 0.0
        self.last_filepath = ""
        self.status_var = tk.StringVar(value="Status: Idle")

        self._build_ui()

    # ------------------------------------------------------------------
    # UI Construction
    # ------------------------------------------------------------------

    def _build_ui(self):
        # ---- Angle sliders ----
        frm_angles = ttk.LabelFrame(self, text=" Body Lean Angles ", padding=(14, 10))
        frm_angles.pack(fill=tk.X, pady=(0, 12))
        frm_angles.columnconfigure(1, weight=1)

        SliderRow(frm_angles, "Roll (°, + = left up):", self.var_roll,
                  -ROLL_LIMIT, ROLL_LIMIT, row=0)
        SliderRow(frm_angles, "Pitch (°, + = nose ↓):", self.var_pitch,
                  -PITCH_LIMIT, PITCH_LIMIT, row=1)
        SliderRow(frm_angles, "Yaw (°):", self.var_yaw,
                  -YAW_LIMIT, YAW_LIMIT, row=2)

        # ---- Other params ----
        frm_params = ttk.LabelFrame(self, text=" Motion Parameters ", padding=(14, 10))
        frm_params.pack(fill=tk.X, pady=(0, 12))
        frm_params.columnconfigure(1, weight=1)

        SliderRow(frm_params, "Stand Height (m):", self.var_height,
                  HEIGHT_MIN, HEIGHT_MAX, row=0, fmt="{:.3f}")
        SliderRow(frm_params, "Height Comp. (m/rad):", self.var_comp,
                  0.0, 0.4, row=1, fmt="{:.2f}")
        SliderRow(frm_params, "Steps / Segment:", self.var_steps,
                  50, 2000, row=2, fmt="{:.0f}")
        SliderRow(frm_params, "Prep Duration (s):", self.var_prep,
                  0.0, 10.0, row=3, fmt="{:.1f}")

        # ---- Checkboxes + extras ----
        frm_misc = ttk.Frame(self)
        frm_misc.pack(fill=tk.X, pady=(0, 12))

        ttk.Checkbutton(frm_misc, text="Return to neutral", variable=self.var_return).pack(
            side=tk.LEFT, padx=(0, 20))

        ttk.Label(frm_misc, text="dt (s):").pack(side=tk.LEFT)
        dt_entry = ttk.Entry(frm_misc, textvariable=self.var_dt, width=8)
        dt_entry.pack(side=tk.LEFT, padx=(4, 20))

        ttk.Label(frm_misc, text="Output dir:").pack(side=tk.LEFT)
        ttk.Entry(frm_misc, textvariable=self.var_outdir, width=18).pack(
            side=tk.LEFT, padx=4)

        # ---- Estimated duration label ----
        self.lbl_est = ttk.Label(self, text="", foreground="gray")
        self.lbl_est.pack(anchor=tk.W, pady=(0, 6))
        self.var_roll.trace_add("write",  lambda *_: self._update_estimate())
        self.var_pitch.trace_add("write", lambda *_: self._update_estimate())
        self.var_yaw.trace_add("write",   lambda *_: self._update_estimate())
        self.var_steps.trace_add("write", lambda *_: self._update_estimate())
        self.var_return.trace_add("write", lambda *_: self._update_estimate())
        self.var_dt.trace_add("write",    lambda *_: self._update_estimate())
        self.var_prep.trace_add("write",  lambda *_: self._update_estimate())
        self._update_estimate()

        # ---- Buttons ----
        frm_btns = ttk.Frame(self)
        frm_btns.pack(fill=tk.X, pady=(0, 8))

        self.btn_gen = ttk.Button(frm_btns, text="Generate CSV",
                                  command=self.generate_csv)
        self.btn_gen.pack(side=tk.LEFT, fill=tk.X, expand=True, ipady=6, padx=(0, 5))

        self.btn_copy = ttk.Button(frm_btns, text="Copy Path",
                                   command=self.copy_path, state=tk.DISABLED)
        self.btn_copy.pack(side=tk.LEFT, fill=tk.X, expand=True, ipady=6, padx=(5, 0))

        # ---- Progress + status ----
        self.progress = ttk.Progressbar(self, mode="indeterminate")
        self.progress.pack(fill=tk.X, pady=(0, 4))

        ttk.Label(self, textvariable=self.status_var, foreground="#1f6aa5").pack(
            anchor=tk.W, pady=(0, 6))

        # ---- Log ----
        self.log_txt = tk.Text(self, height=12, bg="#1e1e1e", fg="#d4d4d4",
                               font=("Consolas", 9))
        self.log_txt.pack(fill=tk.BOTH, expand=True)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _update_estimate(self):
        try:
            n = int(self.var_steps.get())
            segs = 2 if self.var_return.get() else 1
            dt = float(self.var_dt.get())
            prep = float(self.var_prep.get())
            total_s = (n * segs * dt) + prep
            self.lbl_est.config(
                text=f"Estimated duration: {total_s:.1f} s  ({n * segs} IK steps)")
        except Exception:
            self.lbl_est.config(text="")

    def log(self, text, clear=False):
        if clear:
            self.log_txt.delete("1.0", tk.END)
        self.log_txt.insert(tk.END, text + "\n")
        self.log_txt.see(tk.END)
        self.update_idletasks()

    # ------------------------------------------------------------------
    # Generation
    # ------------------------------------------------------------------

    def generate_csv(self):
        if self.is_running:
            self.log("Already running.")
            return

        cmd = [
            sys.executable, GENERATOR_SCRIPT,
            "--roll",  str(self.var_roll.get()),
            "--pitch", str(self.var_pitch.get()),
            "--yaw",   str(self.var_yaw.get()),
            "-z",      str(self.var_height.get()),
            "--compensation", str(self.var_comp.get()),
            "-n",      str(int(self.var_steps.get())),
            "-dt",     str(self.var_dt.get()),
            "--prep",  str(self.var_prep.get()),
            "-o",      self.var_outdir.get(),
        ]
        if not self.var_return.get():
            cmd.append("--no-return")

        self.is_running = True
        self.t_start = time.time()
        self.btn_gen.config(state=tk.DISABLED)
        self.btn_copy.config(state=tk.DISABLED)
        self.status_var.set("Status: Running (0.0 s)")
        self.progress.start(10)
        self.log(f"$ {' '.join(cmd)}", clear=True)
        self.log("-" * 56)

        threading.Thread(target=self._run, args=(cmd,), daemon=True).start()
        self.after(150, self._poll)

    def _run(self, cmd):
        lines = []
        try:
            proc = subprocess.Popen(
                cmd, cwd=BASE_DIR,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1,
            )
            if proc.stdout:
                for raw in proc.stdout:
                    line = raw.rstrip("\n")
                    if line:
                        lines.append(line)
                        self.proc_queue.put({"type": "log", "text": line})
                proc.stdout.close()
            rc = proc.wait()
            self.proc_queue.put({"type": "done", "rc": rc, "output": "\n".join(lines)})
        except Exception as e:
            self.proc_queue.put({"type": "exception", "error": str(e)})

    def _poll(self):
        while not self.proc_queue.empty():
            ev = self.proc_queue.get()
            t = ev["type"]
            if t == "log":
                self.log(ev["text"])
            elif t == "done":
                self._on_done(ev["rc"], ev["output"])
                return
            elif t == "exception":
                self._on_exception(ev["error"])
                return

        if self.is_running:
            elapsed = time.time() - self.t_start
            self.status_var.set(f"Status: Running ({elapsed:.1f} s)")
            self.after(150, self._poll)

    def _finish(self):
        self.is_running = False
        self.progress.stop()
        self.btn_gen.config(state=tk.NORMAL)

    def _on_done(self, rc, output):
        self._finish()
        if rc == 0:
            self.status_var.set("Status: Done")
            m = re.search(r"Saved to\s*:\s*(.*\.csv)", output)
            if m:
                self.last_filepath = m.group(1).strip()
                self.btn_copy.config(state=tk.NORMAL)
                self.log(f"\nFile: {self.last_filepath}")
            messagebox.showinfo("Done", "Lean CSV generated successfully.")
        else:
            self.status_var.set("Status: Failed")
            messagebox.showerror("Error", "Generation failed — see log for details.")

    def _on_exception(self, err):
        self._finish()
        self.status_var.set("Status: Exception")
        self.log(f"\nException: {err}")
        messagebox.showerror("Exception", err)

    def copy_path(self):
        if self.last_filepath:
            self.clipboard_clear()
            self.clipboard_append(self.last_filepath)
            self.log(f"Copied: {self.last_filepath}")


if __name__ == "__main__":
    app = LeanCSVUI()
    app.mainloop()

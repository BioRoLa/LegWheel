"""
Interactive UI for CorgiRobot Hardware CSV Generator.

This is a standalone Tkinter application that acts as an interactive GUI
wrapper for `generate_hardware_csv.py`.
"""

import math
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import subprocess
import os
import re
import sys
import threading
import queue
import time

# Assume this script is in LegWheel/examples/
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
GENERATOR_SCRIPT = os.path.join(BASE_DIR, "examples", "gait", "generate_hardware_csv.py")

class CSVGeneratorUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("CorgiRobot CSV Generator")
        self.geometry("650x700")
        self.configure(padx=20, pady=20)

        # Style
        style = ttk.Style(self)
        style.theme_use('clam')
        
        # --- Variables ---
        self.var_gait = tk.StringVar(value="Walk")
        self.var_vx = tk.StringVar(value="0.10")
        self.var_vy = tk.StringVar(value="0.00")
        self.var_wz = tk.StringVar(value="0.00")
        self.var_height = tk.StringVar(value="0.25")
        self.var_step = tk.StringVar(value="0.04")
        self.var_period = tk.StringVar(value="4.0")
        self.var_cycles = tk.StringVar(value="10")
        self.var_dt = tk.StringVar(value="0.001")
        self.var_outdir = tk.StringVar(value="outputs/csv")
        # Launch control
        self.var_launch = tk.BooleanVar(value=False)
        self.var_ramp_mode = tk.StringVar(value="cycles")   # "cycles" | "seconds"
        self.var_ramp_cycles = tk.StringVar(value="3")
        self.var_ramp_seconds = tk.StringVar(value="3.0")
        self.var_ramp_floor = tk.StringVar(value="0.10")

        self.is_running = False
        self.proc_queue = queue.Queue()
        self.generation_started_at = 0.0
        self.last_filename = ""
        self.status_var = tk.StringVar(value="Status: Idle")

        self.create_widgets()

    def create_widgets(self):
        # --- Top Section: Load CSV ---
        frame_top = ttk.LabelFrame(self, text=" Template (Optional) ", padding=(10, 10))
        frame_top.pack(fill=tk.X, pady=(0, 15))

        btn_load = ttk.Button(frame_top, text="Load from existing CSV ...", command=self.load_csv)
        btn_load.pack(side=tk.LEFT)
        
        self.lbl_loaded = ttk.Label(frame_top, text="No file loaded.", foreground="gray")
        self.lbl_loaded.pack(side=tk.LEFT, padx=10)

        # --- Middle Section: Parameters ---
        frame_mid = ttk.LabelFrame(self, text=" Gait Parameters ", padding=(15, 15))
        frame_mid.pack(fill=tk.BOTH, expand=True, pady=(0, 15))
        
        # Grid layout for params
        row = 0
        def add_field(label_text, var, row_idx, widget_type="entry", values=None):
            ttk.Label(frame_mid, text=label_text, font=('Arial', 10, 'bold')).grid(row=row_idx, column=0, sticky=tk.W, pady=5)
            if widget_type == "combo":
                cb = ttk.Combobox(frame_mid, textvariable=var, values=values, state="readonly", width=18)
                cb.grid(row=row_idx, column=1, sticky=tk.W, pady=5, padx=10)
            else:
                ent = ttk.Entry(frame_mid, textvariable=var, width=20)
                ent.grid(row=row_idx, column=1, sticky=tk.W, pady=5, padx=10)

        add_field("Gait Type (-g):", self.var_gait, 0, "combo", ["Walk", "Trot", "Pace", "Bound", "Pronk"])
        add_field("Vx (m/s):", self.var_vx, 1)
        add_field("Vy (m/s):", self.var_vy, 2)
        add_field("Wz (rad/s):", self.var_wz, 3)
        add_field("Stand Height (m):", self.var_height, 4)
        add_field("Step Height (m):", self.var_step, 5)
        add_field("Period (s):", self.var_period, 6)
        
        ttk.Separator(frame_mid, orient='horizontal').grid(row=7, column=0, columnspan=2, sticky='ew', pady=10)
        
        add_field("Cycles (-c):", self.var_cycles, 8)
        add_field("Resolution / dt (s):", self.var_dt, 9)
        add_field("Output Dir (-o):", self.var_outdir, 10)

        # --- Launch Control Section ---
        frame_launch = ttk.LabelFrame(self, text=" Launch Control ", padding=(15, 10))
        frame_launch.pack(fill=tk.X, pady=(0, 12))

        ttk.Checkbutton(frame_launch, text="Enable launch ramp",
                        variable=self.var_launch,
                        command=self._toggle_launch_fields).grid(
            row=0, column=0, columnspan=2, sticky=tk.W, pady=(0, 6))

        # Ramp duration: choose cycles or seconds via radio buttons
        self.rb_cycles = ttk.Radiobutton(
            frame_launch, text="Ramp cycles:", variable=self.var_ramp_mode,
            value="cycles", command=self._toggle_launch_fields)
        self.rb_cycles.grid(row=1, column=0, sticky=tk.W, pady=3)
        self.ent_ramp_cycles = ttk.Entry(frame_launch, textvariable=self.var_ramp_cycles, width=8)
        self.ent_ramp_cycles.grid(row=1, column=1, sticky=tk.W, padx=10)

        self.rb_seconds = ttk.Radiobutton(
            frame_launch, text="Ramp seconds:", variable=self.var_ramp_mode,
            value="seconds", command=self._toggle_launch_fields)
        self.rb_seconds.grid(row=2, column=0, sticky=tk.W, pady=3)
        self.ent_ramp_seconds = ttk.Entry(frame_launch, textvariable=self.var_ramp_seconds, width=8)
        self.ent_ramp_seconds.grid(row=2, column=1, sticky=tk.W, padx=10)

        ttk.Label(frame_launch, text="Ramp floor (0–1):").grid(
            row=3, column=0, sticky=tk.W, pady=3)
        self.ent_ramp_floor = ttk.Entry(frame_launch, textvariable=self.var_ramp_floor, width=8)
        self.ent_ramp_floor.grid(row=3, column=1, sticky=tk.W, padx=10)

        self.lbl_launch_hint = ttk.Label(
            frame_launch,
            text="e.g. 3 cycles, floor=0.1 → v ramps 10%→55%→100% of target",
            foreground="gray")
        self.lbl_launch_hint.grid(row=4, column=0, columnspan=2, sticky=tk.W)

        self._toggle_launch_fields()   # set initial enabled state

        # --- Bottom Section: Action & Logs ---
        frame_bot = ttk.Frame(self)
        frame_bot.pack(fill=tk.BOTH, expand=True)

        frame_btns = ttk.Frame(frame_bot)
        frame_btns.pack(fill=tk.X, pady=(0, 10))

        self.btn_generate = ttk.Button(frame_btns, text="🚀 Generate CSV", command=self.generate_csv, style="Accent.TButton")
        self.btn_generate.pack(side=tk.LEFT, fill=tk.X, expand=True, ipady=5, padx=(0, 5))

        self.btn_copy = ttk.Button(frame_btns, text="📋 Copy Filename", command=self.copy_filename, state=tk.DISABLED)
        self.btn_copy.pack(side=tk.LEFT, fill=tk.X, expand=True, ipady=5, padx=(5, 0))

        self.progress = ttk.Progressbar(frame_bot, mode="indeterminate")
        self.progress.pack(fill=tk.X, pady=(0, 6))

        self.lbl_status = ttk.Label(frame_bot, textvariable=self.status_var, foreground="#1f6aa5")
        self.lbl_status.pack(anchor=tk.W, pady=(0, 8))

        # Log Text Box
        self.log_txt = tk.Text(frame_bot, height=12, bg="#1e1e1e", fg="#d4d4d4", font=("Consolas", 9))
        self.log_txt.pack(fill=tk.BOTH, expand=True)

    def _toggle_launch_fields(self):
        enabled = self.var_launch.get()
        if enabled:
            mode = self.var_ramp_mode.get()
            self.ent_ramp_cycles.config(state=tk.NORMAL if mode == "cycles" else tk.DISABLED)
            self.ent_ramp_seconds.config(state=tk.NORMAL if mode == "seconds" else tk.DISABLED)
            self.ent_ramp_floor.config(state=tk.NORMAL)
        else:
            self.ent_ramp_cycles.config(state=tk.DISABLED)
            self.ent_ramp_seconds.config(state=tk.DISABLED)
            self.ent_ramp_floor.config(state=tk.DISABLED)

    def log(self, text, clear=False):
        if clear:
            self.log_txt.delete("1.0", tk.END)
        self.log_txt.insert(tk.END, text + "\n")
        self.log_txt.see(tk.END)
        self.update_idletasks()

    def load_csv(self):
        filepath = filedialog.askopenfilename(
            initialdir=os.path.join(BASE_DIR, "outputs", "csv"),
            title="Select CSV Template",
            filetypes=(("CSV Files", "*.csv"), ("All Files", "*.*"))
        )
        if not filepath:
            return

        filename = os.path.basename(filepath)
        self.lbl_loaded.config(text=filename, foreground="blue")
        self.log(f"Loading template: {filename}", clear=True)

        # Pattern: Walk_Vx0.10_Vy0.00_Wz0.00_H0.25_S0.029_P1.0.csv
        # Pattern can handle negative numbers e.g. Vx-0.10
        pattern = r"(?P<gait>[A-Za-z]+)_Vx(?P<vx>[\d\.\-]+)_Vy(?P<vy>[\d\.\-]+)_Wz(?P<wz>[\d\.\-]+)_H(?P<h>[\d\.\-]+)_S(?P<s>[\d\.\-]+)_P(?P<p>[\d\.\-]+)\.csv$"
        match = re.match(pattern, filename)
        if match:
            d = match.groupdict()
            self.var_gait.set(d['gait'])
            self.var_vx.set(d['vx'])
            self.var_vy.set(d['vy'])
            self.var_wz.set(d['wz'])
            self.var_height.set(d['h'])
            # Note: The parsed 'S' is the scaled step height. The user might want the original, 
            # but setting it here is safer / expected.
            self.var_step.set(d['s'])
            self.var_period.set(d['p'])
            self.log("Parameters successfully parsed and updated.")
        else:
            self.log("Could NOT parse parameters from filename. File might not map to standard format.")

    def generate_csv(self):
        if self.is_running:
            self.log("A generation task is already running.")
            return

        # Build command
        cmd = [
            sys.executable, GENERATOR_SCRIPT,
            "-g", self.var_gait.get(),
            "-vx", self.var_vx.get(),
            "-vy", self.var_vy.get(),
            "-wz", self.var_wz.get(),
            "-z", self.var_height.get(),
            "-s", self.var_step.get(),
            "-p", self.var_period.get(),
            "-c", self.var_cycles.get(),
            "-dt", self.var_dt.get(),
            "-o", self.var_outdir.get(),
        ]
        if self.var_launch.get():
            if self.var_ramp_mode.get() == "seconds":
                try:
                    ramp_secs = float(self.var_ramp_seconds.get())
                    period = float(self.var_period.get())
                    n_ramp = max(1, math.ceil(ramp_secs / period))
                except ValueError:
                    n_ramp = 3
                ramp_cycles_str = str(n_ramp)
            else:
                ramp_cycles_str = self.var_ramp_cycles.get()
            cmd += [
                "--launch",
                "--ramp-cycles", ramp_cycles_str,
                "--ramp-floor",  self.var_ramp_floor.get(),
            ]

        self.is_running = True
        self.generation_started_at = time.time()
        self.btn_generate.config(state=tk.DISABLED)
        self.btn_copy.config(state=tk.DISABLED)
        self.status_var.set("Status: Running (0.0s)")
        self.progress.start(10)

        self.log(f"Executing: {' '.join(cmd)}", clear=True)
        self.log("-" * 50)

        worker = threading.Thread(target=self._run_generator, args=(cmd,), daemon=True)
        worker.start()
        self.after(120, self._poll_process_queue)

    def _run_generator(self, cmd):
        output_lines = []
        try:
            # Stream output from generator to keep UI informed while process runs.
            process = subprocess.Popen(
                cmd,
                cwd=BASE_DIR,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )

            if process.stdout is not None:
                for raw_line in process.stdout:
                    line = raw_line.rstrip("\n")
                    if line:
                        output_lines.append(line)
                        self.proc_queue.put({"type": "log", "text": line})
                process.stdout.close()

            returncode = process.wait()
            self.proc_queue.put(
                {
                    "type": "done",
                    "returncode": returncode,
                    "output": "\n".join(output_lines),
                }
            )
        except Exception as e:
            self.proc_queue.put({"type": "exception", "error": str(e)})

    def _poll_process_queue(self):
        while not self.proc_queue.empty():
            event = self.proc_queue.get()
            event_type = event.get("type")

            if event_type == "log":
                self.log(event.get("text", ""))
            elif event_type == "done":
                self._on_generation_done(event.get("returncode", 1), event.get("output", ""))
                return
            elif event_type == "exception":
                self._on_generation_exception(event.get("error", "Unknown error"))
                return

        if self.is_running:
            elapsed = time.time() - self.generation_started_at
            self.status_var.set(f"Status: Running ({elapsed:.1f}s)")
            self.after(120, self._poll_process_queue)

    def _finish_running_state(self):
        self.is_running = False
        self.progress.stop()
        self.btn_generate.config(state=tk.NORMAL)

    def _on_generation_done(self, returncode, output_text):
        self._finish_running_state()

        if returncode == 0:
            self.status_var.set("Status: Completed")
            self.log(f"Process completed successfully. Code {returncode}")
            m = re.search(r"Saved to:\s*(.*\.csv)", output_text)
            if m:
                filepath = m.group(1).strip()
                self.last_filename = os.path.basename(filepath)
                self.btn_copy.config(state=tk.NORMAL)
                self.log(f"Generated file: {self.last_filename}")

            messagebox.showinfo("Success", "CSV generated successfully!\nCheck the logs for details.")
        else:
            self.status_var.set("Status: Failed")
            self.log(f"Process failed with exit code: {returncode}")
            messagebox.showerror("Error", "Generation failed. Check the logs.")

    def _on_generation_exception(self, error_text):
        self._finish_running_state()
        self.status_var.set("Status: Exception")
        self.log(f"Exception occurred:\n{error_text}")
        messagebox.showerror("Exception", error_text)

    def copy_filename(self):
        if hasattr(self, 'last_filename') and self.last_filename:
            self.clipboard_clear()
            self.clipboard_append(self.last_filename)
            self.log(f"Copied to clipboard: {self.last_filename}")

if __name__ == "__main__":
    app = CSVGeneratorUI()
    app.mainloop()

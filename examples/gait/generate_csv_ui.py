"""
Interactive UI for CorgiRobot Hardware CSV Generator.

This is a standalone Tkinter application that acts as an interactive GUI
wrapper for `generate_hardware_csv.py`.
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import subprocess
import os
import re
import sys

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

        # --- Bottom Section: Action & Logs ---
        frame_bot = ttk.Frame(self)
        frame_bot.pack(fill=tk.BOTH, expand=True)

        frame_btns = ttk.Frame(frame_bot)
        frame_btns.pack(fill=tk.X, pady=(0, 10))

        btn_generate = ttk.Button(frame_btns, text="🚀 Generate CSV", command=self.generate_csv, style="Accent.TButton")
        btn_generate.pack(side=tk.LEFT, fill=tk.X, expand=True, ipady=5, padx=(0, 5))

        self.btn_copy = ttk.Button(frame_btns, text="📋 Copy Filename", command=self.copy_filename, state=tk.DISABLED)
        self.btn_copy.pack(side=tk.LEFT, fill=tk.X, expand=True, ipady=5, padx=(5, 0))

        # Log Text Box
        self.log_txt = tk.Text(frame_bot, height=12, bg="#1e1e1e", fg="#d4d4d4", font=("Consolas", 9))
        self.log_txt.pack(fill=tk.BOTH, expand=True)

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
            "-o", self.var_outdir.get()
        ]
        
        self.log(f"Executing: {' '.join(cmd)}", clear=True)
        self.log("-" * 50)
        
        try:
            # Run the command and capture output
            result = subprocess.run(cmd, cwd=BASE_DIR, capture_output=True, text=True)
            
            # Print stdout and stderr
            if result.stdout:
                self.log(result.stdout)
            if result.stderr:
                self.log(f"[ERROR or WARNING]\n{result.stderr}")
                
            if result.returncode == 0:
                self.log(f"Process completed successfully. Code {result.returncode}")
                # Try to extract the filename from stdout
                m = re.search(r"Saved to:\s*(.*\.csv)", result.stdout)
                if m:
                    filepath = m.group(1).strip()
                    self.last_filename = os.path.basename(filepath)
                    self.btn_copy.config(state=tk.NORMAL)

                messagebox.showinfo("Success", "CSV generated successfully!\nCheck the logs for details.")
            else:
                self.log(f"Process failed with exit code: {result.returncode}")
                messagebox.showerror("Error", "Generation failed. Check the logs.")
                
        except Exception as e:
            self.log(f"Exception occurred:\n{str(e)}")
            messagebox.showerror("Exception", str(e))

    def copy_filename(self):
        if hasattr(self, 'last_filename') and self.last_filename:
            self.clipboard_clear()
            self.clipboard_append(self.last_filename)
            self.log(f"Copied to clipboard: {self.last_filename}")

if __name__ == "__main__":
    app = CSVGeneratorUI()
    app.mainloop()

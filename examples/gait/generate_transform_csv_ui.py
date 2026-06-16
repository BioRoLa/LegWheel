"""
Interactive UI for CorgiRobot actuator transform CSV generation.

This standalone Tkinter tool generates a simple 12-DOF point-to-point actuator
positioning CSV. It does not invoke gait planning.
"""

import os
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from generate_transform_csv import generate_transform_csv


BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
LEG_NAMES = ("FL", "FR", "RR", "RL")
JOINT_NAMES = ("theta", "beta", "gamma")


class TransformCSVUI(tk.Tk):
    """Tkinter UI for actuator transform CSV generation."""

    def __init__(self):
        super().__init__()
        self.title("CorgiRobot Actuator Transform CSV")
        self.geometry("760x620")
        self.configure(padx=20, pady=20)

        style = ttk.Style(self)
        style.theme_use("clam")

        self.unit_var = tk.StringVar(value="deg")
        self.duration_var = tk.StringVar(value="5.0")
        self.hold_var = tk.StringVar(value="0.0")
        self.dt_var = tk.StringVar(value="0.001")
        self.output_var = tk.StringVar(
            value=os.path.join("outputs", "csv", "transform_pose.csv")
        )

        self.target_vars = self._create_pose_vars(default_theta="45.0")
        self.start_vars = self._create_pose_vars(default_theta="17.0")

        self.create_widgets()

    def _create_pose_vars(self, default_theta):
        """Create StringVars for a four-leg actuator pose.

        Args:
            default_theta (str): Default theta value for all four legs.

        Returns:
            dict: Nested dictionary keyed by joint name and leg name.
        """
        pose_vars = {}
        for joint_name in JOINT_NAMES:
            pose_vars[joint_name] = {}
            for leg_name in LEG_NAMES:
                default_value = default_theta if joint_name == "theta" else "0.0"
                pose_vars[joint_name][leg_name] = tk.StringVar(value=default_value)
        return pose_vars

    def create_widgets(self):
        """Build UI widgets."""
        title = ttk.Label(
            self,
            text="Actuator Transform CSV Generator",
            font=("Arial", 15, "bold"),
        )
        title.pack(anchor=tk.W, pady=(0, 12))

        self._add_pose_frame(" Target Pose ", self.target_vars).pack(fill=tk.X, pady=(0, 12))
        self._add_pose_frame(" Start Pose ", self.start_vars).pack(fill=tk.X, pady=(0, 12))
        self._add_options_frame().pack(fill=tk.X, pady=(0, 12))
        self._add_action_frame().pack(fill=tk.BOTH, expand=True)

    def _add_pose_frame(self, title, pose_vars):
        """Create one pose entry frame.

        Args:
            title (str): Frame title.
            pose_vars (dict): Nested StringVar dictionary.

        Returns:
            ttk.LabelFrame: Created frame.
        """
        frame = ttk.LabelFrame(self, text=title, padding=(12, 10))

        ttk.Label(frame, text="Joint / Leg", font=("Arial", 10, "bold")).grid(
            row=0, column=0, padx=6, pady=4
        )
        for col, leg_name in enumerate(LEG_NAMES, start=1):
            ttk.Label(frame, text=leg_name, font=("Arial", 10, "bold")).grid(
                row=0, column=col, padx=6, pady=4
            )

        for row, joint_name in enumerate(JOINT_NAMES, start=1):
            ttk.Label(frame, text=joint_name).grid(row=row, column=0, sticky=tk.W, padx=6, pady=4)
            for col, leg_name in enumerate(LEG_NAMES, start=1):
                entry = ttk.Entry(frame, textvariable=pose_vars[joint_name][leg_name], width=11)
                entry.grid(row=row, column=col, padx=6, pady=4)

        return frame

    def _add_options_frame(self):
        """Create generation option widgets.

        Returns:
            ttk.LabelFrame: Created frame.
        """
        frame = ttk.LabelFrame(self, text=" Options ", padding=(12, 10))

        ttk.Label(frame, text="Unit:").grid(row=0, column=0, sticky=tk.W, padx=6, pady=4)
        ttk.Combobox(
            frame,
            textvariable=self.unit_var,
            values=("deg", "rad"),
            state="readonly",
            width=8,
        ).grid(row=0, column=1, sticky=tk.W, padx=6, pady=4)

        self._add_option_entry(frame, "Duration (s):", self.duration_var, row=0, column=2)
        self._add_option_entry(frame, "Hold (s):", self.hold_var, row=0, column=4)
        self._add_option_entry(frame, "dt (s):", self.dt_var, row=1, column=0)

        ttk.Label(frame, text="Output:").grid(row=2, column=0, sticky=tk.W, padx=6, pady=4)
        ttk.Entry(frame, textvariable=self.output_var, width=52).grid(
            row=2, column=1, columnspan=4, sticky=tk.EW, padx=6, pady=4
        )
        ttk.Button(frame, text="Browse...", command=self.browse_output).grid(
            row=2, column=5, sticky=tk.W, padx=6, pady=4
        )
        frame.columnconfigure(4, weight=1)
        return frame

    def _add_option_entry(self, frame, label, variable, row, column):
        """Add one labeled option entry.

        Args:
            frame (ttk.Frame): Parent frame.
            label (str): Label text.
            variable (tk.StringVar): Entry variable.
            row (int): Grid row.
            column (int): Grid column.
        """
        ttk.Label(frame, text=label).grid(row=row, column=column, sticky=tk.W, padx=6, pady=4)
        ttk.Entry(frame, textvariable=variable, width=10).grid(
            row=row, column=column + 1, sticky=tk.W, padx=6, pady=4
        )

    def _add_action_frame(self):
        """Create action buttons and log box.

        Returns:
            ttk.Frame: Created frame.
        """
        frame = ttk.Frame(self)

        button_row = ttk.Frame(frame)
        button_row.pack(fill=tk.X, pady=(0, 8))
        ttk.Button(button_row, text="Generate Transform CSV", command=self.generate_csv).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5), ipady=5
        )
        ttk.Button(button_row, text="Reset Home Start", command=self.reset_start_pose).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(5, 0), ipady=5
        )

        self.log_txt = tk.Text(frame, height=10, bg="#1e1e1e", fg="#d4d4d4", font=("Consolas", 9))
        self.log_txt.pack(fill=tk.BOTH, expand=True)
        self.log("Ready. Inputs are in FL, FR, RR, RL order.")
        self.log("Safety limits: theta 17~160 deg, beta/gamma from RobotParams.")
        return frame

    def browse_output(self):
        """Open a save-file dialog for the output CSV."""
        filepath = filedialog.asksaveasfilename(
            initialdir=os.path.join(BASE_DIR, "outputs", "csv"),
            title="Save transform CSV",
            defaultextension=".csv",
            filetypes=(("CSV Files", "*.csv"), ("All Files", "*.*")),
        )
        if filepath:
            self.output_var.set(filepath)

    def reset_start_pose(self):
        """Reset the start pose to the hardware home pose."""
        for leg_name in LEG_NAMES:
            self.start_vars["theta"][leg_name].set("17.0")
            self.start_vars["beta"][leg_name].set("0.0")
            self.start_vars["gamma"][leg_name].set("0.0")
        self.log("Start pose reset to theta=17, beta=0, gamma=0.")

    def _read_pose(self, pose_vars):
        """Read one pose from UI fields.

        Args:
            pose_vars (dict): Nested StringVar dictionary.

        Returns:
            tuple: ``(theta, beta, gamma)`` lists in FL, FR, RR, RL order.
        """
        values = []
        for joint_name in JOINT_NAMES:
            joint_values = []
            for leg_name in LEG_NAMES:
                joint_values.append(float(pose_vars[joint_name][leg_name].get()))
            values.append(joint_values)
        return tuple(values)

    def generate_csv(self):
        """Generate a transform CSV from UI values."""
        try:
            target_theta, target_beta, target_gamma = self._read_pose(self.target_vars)
            start_theta, start_beta, start_gamma = self._read_pose(self.start_vars)
            filepath = generate_transform_csv(
                target_theta=target_theta,
                target_beta=target_beta,
                target_gamma=target_gamma,
                start_theta=start_theta,
                start_beta=start_beta,
                start_gamma=start_gamma,
                unit=self.unit_var.get(),
                duration=float(self.duration_var.get()),
                hold_time=float(self.hold_var.get()),
                dt=float(self.dt_var.get()),
                output_path=self.output_var.get(),
            )
        except Exception as exc:
            self.log("ERROR: {}".format(exc))
            messagebox.showerror("Generation failed", str(exc))
            return

        self.log("Generated: {}".format(filepath))
        messagebox.showinfo("Success", "CSV generated:\n{}".format(filepath))

    def log(self, text):
        """Append text to the UI log.

        Args:
            text (str): Log line.
        """
        self.log_txt.insert(tk.END, text + "\n")
        self.log_txt.see(tk.END)


if __name__ == "__main__":
    app = TransformCSVUI()
    app.mainloop()

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Circle, Wedge
from matplotlib.lines import Line2D
import time
from legwheel.models.leg_model import LegModel

class PlotLeg(LegModel):
    """
    Visualization tool for the Leg-Wheel mechanism.
    Inherits kinematics from LegModel and provides Matplotlib plotting.
    """
    def __init__(self):
        super().__init__()
        # Origin of leg in plot coordinate system
        self.O = np.array([0, 0])
        # Inner class to manage geometric shapes
        self.leg_shape = self.LegShape(self, self.O)

    class LegShape:
        def __init__(self, leg_model, O):
            self.leg_model = leg_model
            self.O = np.array(O)
            
            # Plot settings
            self.fig_size = 15
            self.mark_size = 2.0
            self.line_width = 1.0
            self.zorder = 1.0
            self.color = "black"
            self.link_alpha = 0.0
            self.Construction = False
            
            # Color palette
            self.color_set = [plt.get_cmap('Set1')(i) for i in range(plt.get_cmap('Set1').N)] + [plt.get_cmap('Set2')(i) for i in range(plt.get_cmap('Set2').N)]
            self.color_label = {
                'Actuating_Link': self.color_set[0],
                'Driven_Link'   : self.color_set[1],
                'Upper_Rim'     : self.color_set[2],
                'Lower_Rim'     : self.color_set[3],
                'Foot_Rim'      : self.color_set[4],
                'Upper_Tyre'    : self.color_set[6],
                'Construction_Line' : self.color_set[8],
                'Axis': self.color_set[9],
                'trajectory':self.color_set[10],
                'Joint': 'k'
            }
            self.get_shape(O)

        class RimObj:
            def __init__(self, arc_in, arc_out, start, fill):
                self.arc = [arc_in, arc_out]
                self.start = start
                self.arc_fill = fill

        def get_shape(self, O):
            """Updates all geometric primitives based on current leg state."""
            self.O = np.array(O)
            self._update_geometry()

        def _update_geometry(self):
            lm = self.leg_model
            # Arcs
            c = 0.0012 # Clearance
            self.upper_rim_r = self.RimObj(*self._make_arc(lm.F_r, lm.H_r, lm.U_r, lm.r-c, 'Upper_Rim'))
            self.upper_rim_l = self.RimObj(*self._make_arc(lm.H_l, lm.F_l, lm.U_l, lm.r-c, 'Upper_Rim'))
            self.lower_rim_r = self.RimObj(*self._make_arc(lm.G,   lm.F_r, lm.L_r, lm.r-c, 'Lower_Rim'))
            self.lower_rim_l = self.RimObj(*self._make_arc(lm.F_l, lm.G,   lm.L_l, lm.r-c, 'Lower_Rim'))
            self.foot_rim    = self.RimObj(*self._make_arc(lm.I_l, lm.I_r, lm.O_r, lm.tyre_thickness, 'Foot_Rim'))
            self.upper_rim_r_f = self.RimObj(*self._make_arc(lm.J_r, lm.H_extend_r, lm.U_r, lm.tyre_thickness-c, 'Upper_Tyre', z_off=0.0002))
            self.upper_rim_l_f = self.RimObj(*self._make_arc(lm.H_extend_l, lm.J_l, lm.U_l, lm.tyre_thickness-c, 'Upper_Tyre', z_off=0.0002))

            # Joints
            self.upper_joint_r = self._make_circle(lm.H_r, lm.r, 'Upper_Rim')
            self.upper_joint_l = self._make_circle(lm.H_l, lm.r, 'Upper_Rim')
            self.lower_joint_r = self._make_circle(lm.F_r, lm.r, 'Lower_Rim')
            self.lower_joint_l = self._make_circle(lm.F_l, lm.r, 'Lower_Rim')
            self.G_joint       = self._make_circle(lm.G,   lm.r, 'Foot_Rim')
            self.foot_joint    = self._make_circle(lm.G,   lm.foot_offset + lm.tyre_thickness, 'Foot_Rim')
            self.I_joint_l     = self._make_circle(lm.I_l, lm.tyre_thickness, 'Foot_Rim')
            self.I_joint_r     = self._make_circle(lm.I_r, lm.tyre_thickness, 'Foot_Rim')
            self.J_joint_l     = self._make_circle(lm.J_l, lm.tyre_thickness, 'Upper_Tyre', z_off=0.0002)
            self.J_joint_r     = self._make_circle(lm.J_r, lm.tyre_thickness, 'Upper_Tyre', z_off=0.0002)
            self.H_extend_joint_l = self._make_circle(lm.H_extend_l, lm.tyre_thickness, 'Upper_Tyre', z_off=0.0002)
            self.H_extend_joint_r = self._make_circle(lm.H_extend_r, lm.tyre_thickness, 'Upper_Tyre', z_off=0.0002)
            
            # Bars
            self.OB_bar_r = self._make_line([0,0], lm.B_r, 'Actuating_Link')
            self.OB_bar_l = self._make_line([0,0], lm.B_l, 'Actuating_Link')
            self.AE_bar_r = self._make_line(lm.A_r, lm.E, 'Driven_Link')
            self.AE_bar_l = self._make_line(lm.A_l, lm.E, 'Driven_Link')
            self.CD_bar_r = self._make_line(lm.C_r, lm.D_r, 'Driven_Link')
            self.CD_bar_l = self._make_line(lm.C_l, lm.D_l, 'Driven_Link')

        def _to_xy(self, p):
            """Ensures point is [x, y] regardless of complex/vector source."""
            if isinstance(p, (complex, np.complex128)):
                return np.array([p.real, p.imag])
            return np.array(p)

        def _make_arc(self, p1, p2, o, offset, color_key, z_off=0.0):
            p1, p2, o = self._to_xy(p1), self._to_xy(p2), self._to_xy(o)
            start = np.degrees(np.arctan2((p1-o)[1], (p1-o)[0]))
            end = np.degrees(np.arctan2((p2-o)[1], (p2-o)[0]))
            radius = np.linalg.norm(p1-o)
            color = self.color_label[color_key]
            
            # Standard matplotlib Arc/Wedge
            center = self.O + o
            arc_in = Arc(center, 2*(radius-offset), 2*(radius-offset), theta1=start, theta2=end, color=color, linewidth=self.line_width, zorder=self.zorder+z_off)
            arc_out = Arc(center, 2*(radius+offset), 2*(radius+offset), theta1=start, theta2=end, color=color, linewidth=self.line_width, zorder=self.zorder+z_off)
            fill = Wedge(center, radius+offset, start, end, width=2*offset, facecolor=(1,1,1,self.link_alpha), zorder=self.zorder+z_off-0.0001)
            return arc_in, arc_out, start, fill

        def _make_circle(self, o, r, color_key, z_off=0.0):
            o = self._to_xy(o)
            return Circle(self.O + o, radius=r, facecolor=(1,1,1,self.link_alpha), edgecolor=self.color_label[color_key], linewidth=self.line_width, zorder=self.zorder + z_off)

        def _make_line(self, p1, p2, color_key):
            p1, p2 = self._to_xy(p1), self._to_xy(p2)
            return Line2D(self.O[0] + [p1[0], p2[0]], self.O[1] + [p1[1], p2[1]], marker='o', markersize=self.mark_size, color=self.color_label[color_key], linewidth=self.line_width, zorder=self.zorder)

    def plot_leg(self, theta, beta, O, ax):
        """Updates kinematics and renders leg on axis."""
        self.forward(theta, beta, vector=False) # PlotLeg uses complex internals for consistency
        self.leg_shape.get_shape(O)
        
        for key, val in self.leg_shape.__dict__.items():
            if "rim" in key and hasattr(val, 'arc'):
                ax.add_patch(val.arc[0]); ax.add_patch(val.arc[1])
                if self.leg_shape.link_alpha > 0: ax.add_patch(val.arc_fill)
            elif "joint" in key:
                ax.add_patch(val)
            elif "bar" in key:
                ax.add_line(val)
        return ax

    def plot_by_angle(self, theta=None, beta=0.0, O=[0,0], ax=None):
        if theta is None: theta = self.theta0
        if ax is None: fig, ax = plt.subplots()
        ax.set_aspect('equal')
        return self.plot_leg(theta, beta, O, ax)

if __name__ == '__main__':
    pl = PlotLeg()
    pl.plot_by_angle()
    plt.show()
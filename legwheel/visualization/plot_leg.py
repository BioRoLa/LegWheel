import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc
from matplotlib.patches import Circle
from matplotlib.patches import Wedge
from matplotlib.lines import Line2D
import time
from legwheel.models import leg_model as LegModel
from legwheel.utils.fitted_coefficient import *

class PlotLeg(LegModel.LegModel):
    def __init__(self):
        super().__init__()
        self.forward(np.deg2rad(17), 0, vector=False)
        self.O = np.array([0, 0])   # origin of leg in world coordinate
        self.leg_shape = self.LegShape(self, self.O)   # initial pose of leg
        
    ## Shape Of Leg ##  
    class LegShape:
        def __init__(self, leg_model, O):
            self.O = O
            self.leg_model = leg_model
            self.R = leg_model.R
            self.r = leg_model.r
            self.radius = leg_model.radius
            self.foot_offset = leg_model.foot_offset
            self.tyre_thickness = leg_model.tyre_thickness
            # Plot setting 
            self.fig_size = 15
            self.mark_size = 2.0
            self.line_width = 1.0
            self.zorder = 1.0
            self.color = "black"
            self.link_alpha = 0.0
            self.Construction = None
            self.Colorful_Display = True
            # Color setting
            self.color_set = [plt.get_cmap('Set1')(i) for i in range(plt.get_cmap('Set1').N)] + [plt.get_cmap('Set2')(i) for i in range(plt.get_cmap('Set2').N)]
            self.color_label = {'Actuating_Link': self.color_set[0],
                                'Driven_Link'   : self.color_set[1],
                                'Upper_Rim'     : self.color_set[2],
                                'Lower_Rim'     : self.color_set[3],
                                'Foot_Rim'      : self.color_set[4],
                                'Upper_Tyre'    : self.color_set[6],    # skip Color No5('Y')
                                'Construction_Line' : self.color_set[8],
                                'Axis':self.color_set[9],
                                'trajectory':self.color_set[10],
                                'Joint'         : 'k'
                                }
            self.get_shape(O)

        class rim:
            def __init__(self, arc, arc_out, start, arc_fill):
                self.arc = [arc, arc_out]   # inner & outer arcs
                self.start = start          # start angle 
                self.arc_fill = arc_fill    # fill white between inner & outer arcs
                
        def plot_arcs(self):
            # four rims (inner arc, outer arc, start angle)
            clearance = 0.0012
            self.upper_rim_r = self.rim( *self.get_arc(p1 = self.leg_model.F_r,
                                                       p2 = self.leg_model.H_r,
                                                       o  = self.leg_model.U_r,
                                                       offset = self.r-clearance,
                                                       edge_color_overwrite=(True, (self.color_label['Upper_Rim'], 0.8) )))

            self.upper_rim_l = self.rim( *self.get_arc(p1 = self.leg_model.H_l,
                                                       p2 = self.leg_model.F_l,
                                                       o  = self.leg_model.U_l,
                                                       offset = self.r-clearance,
                                                       edge_color_overwrite=(True, (self.color_label['Upper_Rim'], 0.8) )))

            self.lower_rim_r = self.rim( *self.get_arc(p1 = self.leg_model.G,
                                                       p2 = self.leg_model.F_r,
                                                       o  = self.leg_model.L_r,
                                                       offset = self.r-clearance,
                                                       edge_color_overwrite=(True, (self.color_label['Lower_Rim'], 0.8) )))

            self.lower_rim_l = self.rim( *self.get_arc(p1 = self.leg_model.F_l,
                                                       p2 = self.leg_model.G,
                                                       o  = self.leg_model.L_l,
                                                       offset = self.r-clearance,
                                                       edge_color_overwrite=(True, (self.color_label['Lower_Rim'], 0.8) )))

            self.foot_rim    = self.rim( *self.get_arc(p1 = self.leg_model.I_l,
                                                       p2 = self.leg_model.I_r,
                                                       o  = self.leg_model.O_r,
                                                       offset = self.tyre_thickness,
                                                       edge_color_overwrite=(True, (self.color_label['Foot_Rim'], 0.8) )))

            self.upper_rim_r_foot = self.rim( *self.get_arc(p1 = self.leg_model.J_r,
                                                       p2 = self.leg_model.H_extend_r,
                                                       o  = self.leg_model.U_r,
                                                       offset = self.tyre_thickness-clearance,
                                                       edge_color_overwrite=(True, (self.color_label['Upper_Tyre'], 0.8) ),
                                                       zorder=self.zorder+0.0002))

            self.upper_rim_l_foot = self.rim( *self.get_arc(p1 = self.leg_model.H_extend_l,
                                                       p2 = self.leg_model.J_l,
                                                       o  = self.leg_model.U_l,
                                                       offset = self.tyre_thickness-clearance,
                                                       edge_color_overwrite=(True, (self.color_label['Upper_Tyre'], 0.8) ),
                                                       zorder=self.zorder+0.0002))

        def plot_joints(self):
            # joints on the rims   (center, radius)
            self.upper_joint_r = self.get_circle(self.leg_model.H_r, self.r, edge_color_overwrite=(True, (self.color_label['Upper_Rim'], 0.8)))
            self.upper_joint_l = self.get_circle(self.leg_model.H_l, self.r, edge_color_overwrite=(True, (self.color_label['Upper_Rim'], 0.8)))
            self.lower_joint_r = self.get_circle(self.leg_model.F_r, self.r, edge_color_overwrite=(True, (self.color_label['Lower_Rim'], 0.8)))
            self.lower_joint_l = self.get_circle(self.leg_model.F_l, self.r, edge_color_overwrite=(True, (self.color_label['Lower_Rim'], 0.8)))
            self.G_joint       = self.get_circle(self.leg_model.G,   self.r, edge_color_overwrite=(True, (self.color_label['Foot_Rim'], 0.8)))
            self.foot_joint       = self.get_circle(self.leg_model.G,  self.foot_offset+self.tyre_thickness, edge_color_overwrite=(True, (self.color_label['Foot_Rim'], 0.8)))
            self.I_joint_l       = self.get_circle(self.leg_model.I_l, self.tyre_thickness, edge_color_overwrite=(True, (self.color_label['Foot_Rim'], 0.8)))
            self.I_joint_r       = self.get_circle(self.leg_model.I_r, self.tyre_thickness, edge_color_overwrite=(True, (self.color_label['Foot_Rim'], 0.8)))
            self.H_extend_joint_l = self.get_circle(self.leg_model.H_extend_l, self.tyre_thickness, edge_color_overwrite=(True, (self.color_label['Upper_Tyre'], 0.8)), zorder=self.zorder+0.0002)
            self.H_extend_joint_r = self.get_circle(self.leg_model.H_extend_r, self.tyre_thickness, edge_color_overwrite=(True, (self.color_label['Upper_Tyre'], 0.8)), zorder=self.zorder+0.0002)
            self.J_joint_l        = self.get_circle(self.leg_model.J_l, self.tyre_thickness, edge_color_overwrite=(True, (self.color_label['Upper_Tyre'], 0.8)), zorder=self.zorder+0.0002)
            self.J_joint_r        = self.get_circle(self.leg_model.J_r, self.tyre_thickness, edge_color_overwrite=(True, (self.color_label['Upper_Tyre'], 0.8)), zorder=self.zorder+0.0002)

        def gen_construction_lines(self, p1, p2):
            return self.get_line(p1, p2, color_overwrite=(True, self.color_label['Construction_Line']), line_style_overwrite=(True, '--'), line_width_overwrite=(True, self.line_width*0.8), zorder=self.zorder+1)
            
        def plot_bars(self):
            # six bars  (point1, point2)
            self.OB_bar_r = self.get_line(0, self.leg_model.B_r, color_overwrite=(True, self.color_label['Actuating_Link'])) 
            self.OB_bar_l = self.get_line(0, self.leg_model.B_l, color_overwrite=(True, self.color_label['Actuating_Link'])) 
            self.AE_bar_r = self.get_line(self.leg_model.A_r, self.leg_model.E  , color_overwrite=(True, self.color_label['Driven_Link']))
            self.AE_bar_l = self.get_line(self.leg_model.A_l, self.leg_model.E  , color_overwrite=(True, self.color_label['Driven_Link']))
            self.CD_bar_r = self.get_line(self.leg_model.C_r, self.leg_model.D_r, color_overwrite=(True, self.color_label['Driven_Link']))
            self.CD_bar_l = self.get_line(self.leg_model.C_l, self.leg_model.D_l, color_overwrite=(True, self.color_label['Driven_Link']))
        
        def plot_construction_lines(self):
            # construction lines
            if self.Construction:
                self.U_L_Construction_bar_1 = self.gen_construction_lines(self.leg_model.U_l, self.leg_model.H_extend_l)
                self.U_L_Construction_bar_2 = self.gen_construction_lines(self.leg_model.U_l, self.leg_model.J_l)
                self.U_R_Construction_bar_1 = self.gen_construction_lines(self.leg_model.U_r, self.leg_model.H_extend_r)
                self.U_R_Construction_bar_2 = self.gen_construction_lines(self.leg_model.U_r, self.leg_model.J_r)
                self.foot_Construction_bar_1 = self.gen_construction_lines(self.leg_model.I_l, self.leg_model.O_r)
                self.foot_Construction_bar_2 = self.gen_construction_lines(self.leg_model.I_r, self.leg_model.O_r)
                self.center_Construction_axis_bar = self.get_line(self.leg_model.G*1.3, self.leg_model.G*1.1*np.exp(1j*np.pi), color_overwrite=(True, self.color_label['Axis']), line_style_overwrite=(True, 'dashdot'), line_width_overwrite=(True, self.line_width*0.8), zorder=self.zorder+1)
                
        def get_shape(self, O):
            self.O = np.array(O)    # origin of leg in world coordinate
            self.plot_arcs()
            self.plot_joints()
            self.plot_bars()
            if self.Construction == True:
                self.plot_construction_lines()
        
        def set_variable(self, var_name, value):
            setattr(self, var_name, value)

        def set_color_type(self, color_type='Colorful'):
                if color_type == 'Colorful':
                    self.color_label = {'Actuating_Link': self.color_set[0],
                    'Driven_Link'   : self.color_set[1],
                    'Upper_Rim'     : self.color_set[2],
                    'Lower_Rim'     : self.color_set[3],
                    'Foot_Rim'      : self.color_set[4],
                    'Upper_Tyre'    : self.color_set[6],    # skip Color No5('Y')
                    'Construction_Line' : self.color_set[8],
                    'Axis':self.color_set[9],
                    'Joint'         : 'k'
                    }
                elif color_type == 'Black':
                    for i in self.color_label.keys():
                        self.color_label[i] = 'k'

        def get_arc(self, p1, p2, o, offset=0.01,
                    edge_color_overwrite = (False, None),
                    infill_color_overwrite = (False, None),
                    zorder = None):
            start = np.angle(p1-o, deg=True)
            end = np.angle(p2-o, deg=True)
            radius = np.abs(p1-o)
            arc = Arc(self.O+[o.real, o.imag],
                      2*(radius-offset), 2*(radius-offset),
                      angle=0.0, theta1=start, theta2=end,
                      color=self.color  if not edge_color_overwrite[0]
                                        else edge_color_overwrite[1],
                      linewidth=self.line_width,
                      zorder=self.zorder if zorder is None else zorder)

            arc_out = Arc(self.O+[o.real, o.imag],
                          2*(radius+offset), 2*(radius+offset),
                          angle=0.0, theta1=start, theta2=end,
                          color=self.color  if not edge_color_overwrite[0]
                                            else edge_color_overwrite[1],
                          linewidth=self.line_width,
                          zorder=self.zorder if zorder is None else zorder)

            arc_fill = Wedge(center=self.O+[o.real, o.imag],
                             r=(radius+offset), width=2*offset,
                             theta1=start, theta2=end,
                             facecolor=(1, 1, 1, self.link_alpha)   if not infill_color_overwrite[0]
                                                                    else infill_color_overwrite[1],
                             zorder=(self.zorder-0.00001) if zorder is None else zorder-0.00001)
            return arc, arc_out, start, arc_fill

        def get_circle(self, o, r, edge_color_overwrite = (False, None), zorder=None):
            circle = Circle(self.O+[o.real, o.imag],
                            radius=r, facecolor=(1, 1, 1, self.link_alpha),
                            edgecolor=self.color if not edge_color_overwrite[0] else edge_color_overwrite[1],
                            linewidth=self.line_width,
                            zorder=self.zorder if zorder is None else zorder)
            # circle = Arc([o.real, o.imag], 2*r, 2*r, angle=0.0, theta1=0, theta2=360, color=self.color, linewidth=self.line_width, zorder=self.zorder)
            return circle

        def get_line(self, p1, p2, color_overwrite = (False, None), line_style_overwrite = (False, None), line_width_overwrite = (False, None), zorder=None):
            line = Line2D(self.O[0]+[p1.real, p2.real],
                          self.O[1]+[p1.imag, p2.imag],
                          marker='o', markersize=self.mark_size, linestyle='-' if not line_style_overwrite[0] else line_style_overwrite[1],
                          color= self.color if not color_overwrite[0] else color_overwrite[1],
                          linewidth=self.line_width if not line_width_overwrite[0] else line_width_overwrite[1],
                          zorder=self.zorder if zorder is None else zorder)
            return line
        
        
    ## Parameters Setting ##
    def setting(self, fig_size=-1, mark_size=-1, line_width=-1, link_alpha=-1, color=None, zorder=None):
        if fig_size != -1:
            self.leg_shape.fig_size = fig_size
        if mark_size != -1:
            self.leg_shape.mark_size = mark_size
        if line_width != -1:
            self.leg_shape.line_width = line_width
        if link_alpha != -1:
            self.leg_shape.link_alpha = link_alpha
        if color != None:
            self.leg_shape.color = color
        if zorder != None:
            self.leg_shape.zorder = zorder

            
    #### Plot leg with current shape ####
    def plot_leg(self, theta, beta, O, ax):
        # Initialize all graphics 
        self.forward(theta, beta, vector=False)  # update to apply displacement of origin of leg.
        self.leg_shape.get_shape(O)
        # self.center_line, = ax.plot([], [], linestyle='--', color='blue', linewidth=1)   # center line (Axis of Symmetry)
        # Add leg part to the plot
        for key, value in self.leg_shape.__dict__.items():
            if "rim" in key:
                ax.add_patch(value.arc[0])
                ax.add_patch(value.arc[1])
                if self.leg_shape.link_alpha > 0:
                    ax.add_patch(value.arc_fill)
            elif "joint" in key:
                ax.add_patch(value)
            elif "bar" in key:
                if 'construction' in key:
                    if self.leg_shape.Construction:
                        ax.add_line(value)
                else:
                    ax.add_line(value)

        # Joint points
        self.joint_points = [ ax.plot([], [], marker='o',
                                      color=self.leg_shape.color,
                                      markersize=self.leg_shape.mark_size,
                                      zorder=self.leg_shape.zorder+0.00001)[0] for _ in range(5) ]   # five dots at the center of joints
        for i, circle in enumerate([self.leg_shape.upper_joint_r, self.leg_shape.upper_joint_l, self.leg_shape.lower_joint_r, self.leg_shape.lower_joint_l, self.leg_shape.G_joint]):
            center = circle.get_center()
            self.joint_points[i].set_data([center[0]], [center[1]])
            
        return ax  
    
    #### Plot leg on one fig given from user ####
    def plot_by_angle(self, theta=np.deg2rad(17.0), beta=0, O=np.array([0, 0]), ax=None): 
        O = np.array(O)
        if ax is None:
            fig, ax = plt.subplots()
        # Plot setting
        ax.set_aspect('equal')  # 座標比例相同
        if ax is None:
            ax = self.plot_leg(theta, beta, O, ax)
        else:
            self.plot_leg(theta, beta, O, ax)
        
        return ax

    #### Plot leg by given foothold of G, lower rim, or upper rim ####
    def plot_by_rim(self, foothold=np.array([0, 0]), O=np.array([0, 0]), rim='G', ax=None): 
        O = np.array(O)
        foothold = np.array(foothold)
        if rim == 'G':
            theta, beta = self.inverse(foothold - O + np.array([0, self.r]), 'G')
        elif rim == 'lower':
            if foothold[0] > O[0]:  # left lower rim
                theta, beta = self.inverse(foothold - O + np.array([0, self.radius]), 'L_l')
            else:                   # right lower rim
                theta, beta = self.inverse(foothold - O + np.array([0, self.radius]), 'L_r')
        elif rim == 'upper':
            if foothold[0] > O[0]:  # left lower rim
                theta, beta = self.inverse(foothold - O + np.array([0, self.radius]), 'U_l')
            else:                   # right lower rim
                theta, beta = self.inverse(foothold - O + np.array([0, self.radius]), 'U_r')
            
        if ax is None:
            fig, ax = plt.subplots()
        # plot setting
        ax.set_aspect('equal')  # 座標比例相同
        ax = self.plot_leg(theta, beta, O, ax)
        return ax
    

if __name__ == '__main__':
    file_name = 'plot_leg_example'
    start_time = time.time()  # end time
    
    plot_leg = PlotLeg()  # rad
    ax = plot_leg.plot_by_angle()
    ax = plot_leg.plot_by_angle(np.deg2rad(130), np.deg2rad(-45), [0., 0.3], ax=ax)
    ax = plot_leg.plot_by_rim([0.2, 0.0], [0.1, 0.3], rim='G', ax=ax)
    ax = plot_leg.plot_by_rim([0.6, 0.1], [0.5, 0.2], rim='lower', ax=ax)
    ax = plot_leg.plot_by_rim([0.3, 0.1], [0.4, 0.2], rim='lower', ax=ax)
    plot_leg.setting(mark_size=10, line_width=3, color='red')
    ax = plot_leg.plot_by_rim([0.8, 0.0], [0.9, 0.12], rim='upper', ax=ax)
    ax = plot_leg.plot_by_rim([1.0, 0.0], [0.9, 0.12], rim='upper', ax=ax)
    ax.grid()
    
    plt.savefig(file_name + '.png')
    
    end_time = time.time()  # end time
    print("\nExecution Time:", end_time - start_time, "seconds")
    
    plt.show()
    plt.close()

    

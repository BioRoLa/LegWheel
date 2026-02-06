import numpy as np
import matplotlib.pyplot as plt
from legwheel.models.leg_model import LegModel
from legwheel.visualization.plot_leg import PlotLeg
from legwheel.utils.solver import Solver
from legwheel.utils.fitted_coefficient import inv_G_dist_poly
from legwheel.bezier import swing
import pandas as pd


class TrajectoryPlanner:
    """
    Plan the trajectory of the leg during walking or any other movement.
    Attributes:
        stand_height (float): The height of the leg when standing.
        step_length (float): The length of each step.
        leg (LegModel): An instance of the LegModel class representing the leg.
        step_height (float): The height of each step.
        period (float): The duration of each step.
        H (float): The height from the ground to the hip joint.
        theta0 (float): The initial angle of the upper leg.
        beta0 (float): The initial angle of the lower leg.
        D (float): The horizontal distance moved by the hip during one step.
        L (float): The horizontal distance from the ground contact point to the hip joint.
        ground_contact_point0 (np.array): The initial ground contact point of the foot.
        dt: (float): The time step for the simulation.
        duty: (float): The duty of swing phase in one step.
        overlap: (float): The overlap between stance and swing phases overlap time = duty*overlap
    """
    def __init__(self, stand_height=0.3,
                 step_length=0.4,
                 leg=LegModel(),
                 step_height=0.04,
                 period=2.4,
                 dt=0.01,
                 duty=1/4,
                 overlap=0):
        self.stand_height = stand_height                                    # Initialize stand height
        self.step_length = step_length                                      # Initialize step length
        self.leg = leg                                                      # Initialize leg model
        self.H = stand_height - leg["foot_radius"]                          # Calculate height from ground to hip joint
        self.L = 0                                                          # Distance from ground contact point to hip joint in X axis
        self.D = 0                                                          # Distance from hip joint to ground contact point in X axis
        self.theta0 = np.deg2rad(17)                                        # initial theta angle
        self.beta0 = 0                                                      # initial beta angle
        self.get_init_tb()                                                  # Get initial joint angles
        self.step_height = step_height                                      # Initialize step height
        self.T = period                                                     # Initialize period
        self.dt = dt                                                        # Initialize time step
        self.duty = duty                                                    # Initialize duty of swing phase in one step
        self.overlap = overlap                                              # Initialize overlap between stance and swing phases
        leg.forward(theta=self.theta0, beta=self.beta0)
        self.ground_contact_point0 = leg.rim_point(np.rad2deg(-self.beta0))
        self.V = self.D/self.T*4                                            # velocity of hip joint

        self.leg.forward(theta=self.theta0, beta=-self.beta0)
        self.G_lo = self.leg.G.copy()   # get initial foot position
        self.leg.forward(theta=self.theta0, beta=self.beta0)
        self.G_td = self.leg.G.copy()   # get target foot position
        self.v_ = np.array([-self.L / self.T, 0]) #lift and touch down velocity
        self.v_lo, self.v_td = self.v_, self.v_
        self.swing = swing.SwingLegPlanner(dt=dt, T_sw=self.T * self.duty*(1-self.overlap), T_st=self.T * (1 - self.duty*(1-self.overlap)))         # Initialize swing leg planner
        self.generate_swing_trajectory()                                      # Generate swing trajectory
        
        
    def set_speed(self, speed):
        """
        Set the speed of the leg movement.

        Args:
            speed (float): unit: m/s. The desired speed of the leg movement.

        Raises:
            KeyError: If the speed is not valid.

        Returns:
            float: The updated speed of the leg movement.

            KeyError: If the speed is not valid.

        Returns:
            float: The updated speed of the leg movement.       
        """
        if speed <= 0:
            raise KeyError("Invalid speed. Speed must be positive.")
        if self.V == 0:
            raise KeyError("Current speed is zero, cannot scale to new speed.")
        scale = speed / self.V
        self.T = self.T / scale
        self.reinitialize()
        
    def reinitialize(self):
        self.__init__(stand_height=self.stand_height,
                      step_length=self.step_length,
                      leg=self.leg,
                      step_height=self.step_height,
                      period=self.T,
                      dt=self.dt,
                      duty=self.duty,
                      overlap=self.overlap)

    # ====================< function to joint angles >===================
    # function to transfer stand height and step length to joint angles
    # ===================================================================
    def get_init_tb(self):
        """Get initial joint angles based on the standing height and step length.
        Args:
            None
        Returns:
            theta0 (float): The initial angle of the upper leg.
            beta0 (float): The initial angle of the lower leg.
            D (float): The horizontal distance moved by the hip during one step.
        """
        func = lambda x: (self.H)*np.tan(x) + self.leg["foot_radius"]*x - 3*self.step_length/8
        # solver to solve beta
        solver = Solver(
            method="Secant",
            tol=1e-6,
            max_iter=100,
            function= func,
            derivative= lambda x: (self.H)*np.sec(x)**2 + self.leg["foot_radius"] - 3*self.step_length/8
        )
        self.beta0 = solver.solve(0, np.deg2rad(40))

        # calculate foot position
        G_dist = (self.H)/np.cos(self.beta0) + self.leg["R"]
        OO_r_Dist = G_dist - self.leg["R"]
        self.theta0 = inv_G_dist_poly(G_dist)

        self.L = 2*OO_r_Dist*np.sin(self.beta0)                  # distance of ground contact to hip joint in X axis
        self.D = (self.L+self.leg.foot_radius*2*self.beta0)/3    # movement of hip joint in X axis during one step
        return self.theta0, self.beta0, self.D

    # solve theta angle based on beta angle
    def solve_theta(self, beta):
        G_dist = (self.H)/np.cos(beta) + self.leg["R"]
        theta = inv_G_dist_poly(G_dist)
        return theta

    def generate_swing_trajectory(self):
        """Plan the swing trajectory for the leg.
        Args:
            G_lo (np.ndarray): The initial position of the foot in leg coordinates.
            G_td (np.ndarray): The target position of the foot in leg coordinates.
            step_height (float): The height of the step (maximum height from ground ).
            v_ (np.ndarray): The initial velocity of the foot.
            v (np.ndarray): The target velocity of the foot.
        Returns:
            swing_trajectory (SwingTrajectory): The planned swing trajectory.
        """
        if self.step_height > self.H:
            self.step_height = self.H
        self.swing_trajectory = self.swing.solveSwingTrajectory(self.G_lo, self.G_td, self.step_height, self.v_lo, self.v_td)
        return self.swing_trajectory

    def cartesian_to_polar(self, point):
        """Convert a point from Cartesian to polar coordinates.

        Args:
            point (tuple): The (x, y) coordinates in Cartesian space.

        Returns:
            tuple: The (r, beta) coordinates in polar space.
        """
        x, y = point
        r = np.sqrt(x**2 + y**2)
        beta_ = (np.arctan2(y, x))    # Angle in polar coordinates
        return r, beta_

    def polar_to_cartesian(self, polar):
        """Convert a point from polar to Cartesian coordinates.

        Args:
            polar (tuple): The (r, theta) coordinates in polar space.

        Returns:
            tuple: The (x, y) coordinates in Cartesian space.
        """
        r, theta = polar
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return x, y

    def move(self):
        self.cmd = []   # command list [theta, beta]
        self.phase = [] # phase list [stance=0, swing=1]
        
        # rolling phase planning
        for t in np.arange(0, self.T * (1 - self.duty*(1-self.overlap)), self.dt):
            if t < self.T/4*3:
                # stand phase
                solver = Solver(
                    method="Newton",
                    tol=1e-9,
                    max_iter=100,
                    function= lambda beta: self.H*(np.sin(self.beta0) - np.sin(beta)) + self.leg["foot_radius"]*(self.beta0 - beta) - self.V*t,
                    derivative= lambda beta: -self.H*np.cos(beta) - self.leg["foot_radius"]
                )
                beta = solver.solve(self.cmd[-1][1] if len(self.cmd)>1 else self.beta0)
                if abs(beta) > np.deg2rad(40):
                    break
                self.cmd.append([self.solve_theta(beta), beta])
        self.phase += [0]*len(self.cmd)
        
        # lift off velocity setting
        self.leg.forward(theta=self.cmd[-1][0], beta=self.cmd[-1][1])
        self.G_lo = self.leg["G"].copy()
        self.leg.forward(theta=self.cmd[-2][0], beta=self.cmd[-2][1])
        self.G_lo_prev = self.leg["G"].copy()
        # self.v_lo = self.L/(self.T) * (self.G_lo_prev - self.G_lo) / np.linalg.norm(self.G_lo_prev - self.G_lo)
        # self.v_lo = -self.V * (self.G_lo_prev - self.G_lo) / np.linalg.norm(self.G_lo_prev - self.G_lo)
        self.v_lo = np.array([0, self.V])
        
        # touch down velocity setting
        self.leg.forward(theta=self.cmd[0][0], beta=self.cmd[0][1])
        self.G_td = self.leg["G"].copy()
        self.leg.forward(theta=self.cmd[1][0], beta=self.cmd[1][1])
        self.G_td_prev = self.leg["G"].copy()
        # self.v_td = self.L/(self.T) * (self.G_td_prev - self.G_td) / np.linalg.norm(self.G_td_prev - self.G_td)
        # self.v_td = self.V * (self.G_td_prev - self.G_td) / np.linalg.norm(self.G_td_prev - self.G_td)
        self.v_td = np.array([0,-self.V/10])

        self.generate_swing_trajectory()
        # swing phase
        self.swing_points = [self.swing_trajectory.getFootendPoint(_) 
                             for _ in np.linspace(0, 1, int(self.T * self.duty * (1-self.overlap) / self.dt))]  # get points in swing trajectory [x,y]
        # transfer to polar coordinates
        self.swing_points_polar = list(map(self.cartesian_to_polar, self.swing_points)) # transform to polar coordinates then transfer to command space
        ang_offset = np.pi/2        # angle offset for swing trajectory
        
        # transform to command space
        self.swing_points_cmd = list(map(lambda x: [inv_G_dist_poly(x[0]), x[1] + ang_offset] , self.swing_points_polar))  # transform to command space
        
        self.swing_points_cmd = list(map(lambda x: [x[0] if x[0] <= np.deg2rad(160) else np.deg2rad(160), x[1]] ,
                                         self.swing_points_cmd))  # transform to command space
        self.swing_points_cmd = list(map(lambda x: [x[0] if x[0] >= np.deg2rad(17) else np.deg2rad(17), x[1]] ,
                                         self.swing_points_cmd))  # transform to command space


        self.cmd += self.swing_points_cmd
        self.phase += [1]*len(self.swing_points_cmd)

    # index operator
    def __getitem__(self, key):
        """
            Allows access to joint positions using [] operator.
            Example: legmodel['G'] returns the position of joint G.
        """
        if key not in self.__dict__:
            raise KeyError(f"Var {key}' not found.")
        return self.__dict__[key]
        

if __name__ == "__main__":
    def plot_point(ax, point, color='red',plot_leg=PlotLeg()):
        point = ax.plot(point[0], point[1], marker='o', color=color, markersize=plot_leg.leg_shape.mark_size, zorder=plot_leg.leg_shape.zorder+0.00001)[0]
        return point
    animate = True
    save_data = True
    import time
    if animate:
        from matplotlib.animation import FuncAnimation
    
    # ====================< initialization >===================
    # Initialize parameters for leg model
    # =========================================================
    leg = LegModel()
    traj_planner = TrajectoryPlanner(stand_height = 0.3,
                                     step_length = 0.4,
                                     leg = leg,
                                     step_height=0.02,
                                     period=2.4,
                                     dt=0.001,
                                     duty=1/4,
                                     overlap=0.2)
    theta0, beta0, D = traj_planner.get_init_tb()
    leg.forward(theta=theta0, beta=beta0)
    ground_contact_point0 =  leg.rim_point(np.rad2deg(-beta0))
    start_time = time.time()
    traj_planner.move()
    end_time = time.time()
    print(f"Trajectory planning took {end_time - start_time:.4f} seconds.")
    # ==================< Animation >==================
    # Animate the leg movement
    # =================================================
    beta_list = np.linspace(beta0, -beta0, num=100) # list of beta angles from touch to leave
    fig, ax = plt.subplots(figsize=(8, 5))
    # fig.set_size_inches(8, 6)
    plot_leg = PlotLeg()
    point_G_list = []
    
    if animate:
        t = 0
        def update(frame):
            global t, ax
            # Clear the axis
            ax.clear()
            # fix the frame
            ax.set_xlim(-D-leg.foot_radius, 1)
            ax.set_ylim(-traj_planner.stand_height-0.1, leg.foot_radius)
            ax.set_aspect('equal')

            # beta = beta_list[frame]             # update beta
            # OO_r_Dist = H/np.cos(beta)          # update OO_r_Dist
            # theta=inv_G_dist_poly(OO_r_Dist+leg["R"])    # update theta
            theta, beta = traj_planner.cmd[frame]   # update theta and beta from command list
            hip_movement = traj_planner.H*(np.sin(beta0)-np.sin(beta))+leg.foot_radius*( beta0-beta )            # calculate hip_movement
            # hip_movement = traj_planner.V*t
            # hip_movement = 0
            ground_contact_point =  plot_leg.rim_point(np.rad2deg(-beta))+np.array([hip_movement,0])
            leg.forward(theta, beta)
            point_G_list.append(leg.G+np.array([hip_movement,0]))
            for g in point_G_list:
                plot_point(ax, g, color='green', plot_leg=plot_leg)
            plot_leg.plot_by_angle(theta, beta, O=np.array([hip_movement,0]), ax=ax)                # plot the leg
            plot_point(ax, ground_contact_point, color='red', plot_leg=plot_leg)                    # plot the ground contact point
            plot_point(ax, np.array([[0, hip_movement],[0, 0]]), color='blue', plot_leg=plot_leg)   # plot the hip joint trajectory
            # plot_point(ax, np.array([[ground_contact_point0[0], ground_contact_point[0]],[ground_contact_point0[1], ground_contact_point[1]]]), color='red', plot_leg=plot_leg)             # plot the hip joint position
            ax.grid()
            t += traj_planner.dt
            ax.set_title(f'Time: {t:.2f}s')

        # plot_point(ax, plot_leg.rim_point(np.rad2deg(-beta0)), color='red', plot_leg=plot_leg)
        # plot_leg.plot_by_angle(theta0, beta0, O=np.array([D*3,0]), ax=ax)
        func_animation = FuncAnimation(fig, update, frames=int(len(traj_planner.cmd)), interval=50)
        if save_data:
            # plt.show()
            # Save the animation as a video file
            output_file_name = "".join(["Output_datas/Single_leg_trajectory",
                                        "_Hip_height"+str(traj_planner.stand_height),
                                        "_Step_Length"+str(traj_planner.step_length),
                                        "_Step_Height"+str(traj_planner.step_height),
                                        "_Period"+str(traj_planner.T ),
                                        "_Velocity"+str(round(traj_planner.V, 2))])
            func_animation.save(output_file_name + ".mp4", fps=10)
            pd.DataFrame(traj_planner.cmd, columns=["theta", "beta"]).to_csv(output_file_name + ".csv")
        else:
            plt.show()


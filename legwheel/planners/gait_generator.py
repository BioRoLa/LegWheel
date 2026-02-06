import legwheel.planners.trajectory_planning as TJ
import legwheel.models.leg_model as LegModel
import numpy as np
import matplotlib.pyplot as plt
import legwheel.visualization.plot_leg as PlotLeg
from matplotlib.animation import FuncAnimation
import pandas as pd


# create leg object for planning
class leg_object:
    def __init__(self, curve = None, start_point = 0, inverse = False, phase = []):
        self.leg = PlotLeg.PlotLeg()    # leg object
        self.initial_curve = curve      # initial curve, starts at the touch down point
        self.start_point = start_point  # start point ratio at partial of initial curve
        self.phase = phase              # phase list [stance=0, swing=1]
        self.traj_curve = curve         # real curve, the actual curve this leg is going
        self.inverse = inverse          # inverse beta value for LHS leg
        if curve != None:
            clip_index = int(len(self.initial_curve)*self.start_point)
            self.traj_curve = self.initial_curve[clip_index::]  + self.initial_curve[:clip_index:]
            self.phase      = self.phase[clip_index::]          + self.phase[:clip_index:]
            if inverse:
                self.traj_curve = list(map(lambda x: [x[0], -x[1]], self.traj_curve))
        self.transform = []
        self.transformed = False
        self.output_curve = []
        self.local_Motion = []

    def transform_generation(self):
        """
        generate transform command, data length:5000
        """
        if self.traj_curve != None:
            theta = self.traj_curve[0][0]
            beta  = self.traj_curve[0][1]
            T = np.linspace(np.deg2rad(17), theta, 5000).tolist()
            B = np.linspace(0, beta, 5000).tolist()
            self.transform = [[T[i],B[i]] for i in range(5000)]
            self.transformed = True
        else:
            raise("trajectory curve is empty")
    
    def extend(self, target_length = 0, transform_included = False):
        """
        input target run time, this function will extend it and return a list of cmds
        Args:
            target_length (int, optional): target length want to extend to. Defaults to 0.
            transform_included (bool, optional): is transformation time included inside?. Defaults to False.
        """
        if transform_included:
            if target_length <  5000:
                raise("Target length must be longer then transform time")
            target_length -= 5000
        
        scale = target_length//len(self.traj_curve)
        if not self.transformed:
            self.transform_generation()
        self.local_Motion = self.traj_curve*scale + self.traj_curve[:target_length % len(self.traj_curve)]
        self.phase = self.phase*scale + self.phase[:target_length % len(self.traj_curve)]
        self.output_curve = self.transform + self.local_Motion
        return self.output_curve.copy()
            
    
    
    def get_point_at_time(self, time):
        """
        return [theta, beta] at certain time

        Args:
            time (_type_): certain time, unit: index aka dt*iteration
        """
        return self.output_curve[time]
    
    def __getitem__(self,key):
        if key not in self.__dict__:
            raise KeyError(f"Var {key}' not found.")
        return self.__dict__[key]

class Gait_Generator:
    """_summary_
    """
    def __init__(self,  stand_height=0.25,  step_length=0.4,
                        leg=LegModel.LegModel(),
                        step_height=0.08,   period=10, distance = 3,
                        dt=0.001,   duty=1 / 8, overlap=0.2, velocity = 0.3,gait_select = "walk"):
        # initialize
        self.Traj = TJ.TrajectoryPlanner(stand_height=stand_height,
                                    step_length=step_length,
                                    leg=leg,
                                    step_height=step_height,
                                    period=period,
                                    dt=dt,
                                    duty=duty,
                                    overlap=overlap)
        
        self.Body_Information()
        self.Traj.set_speed(velocity) # set speed to 0.1 m/s or 10cm /s
        self.Traj.move()
        self.Traj_Curve = self.Traj["cmd"]
        self.Target_dist = distance     # target distance to walk, meter
        self.total_time = self.Target_dist / self.Traj.V        # moving time, unit: second
        self.data_len = int(self.total_time / self.Traj.dt)     # target cmd data length must output
        # self.gait_select = "trot"
        self.gait_select = "walk"
        self.select_gait()
        self.legs = [leg_object( curve=self.Traj["cmd"],
                                 start_point=(self.gait[i]-1)/len(self.gait),
                                 inverse= i in [0,3],
                                 phase=self.Traj["phase"] )
                                    for i in range(len(self.gait)) ]
        # self.gait_select = "walk"
        
    # setup robot body infornmation
    def Body_Information(self, BL = 0.444, BH = 0.2, BW = 0.33, CoM_Bias = 0.0):
        self.BL = BL                # body length, 44.4 cm
        self.BH = BH                # body height, 20 cm
        self.BW = BW                # body width, 33 cm
        self.CoM_bias = CoM_Bias    # x bias of center of mass
        self.offset_x = [BL/2, BL/2, -BL/2, -BL/2]
        self.offset_z = [-BW/2, BW/2, BW/2, -BW/2]
    
    # set up gait type
    def select_gait(self, gait = None):
        # ==<leg Number>
        #       1 | 2 
        #       ----- 
        #       4 | 3
        # ==============
        if gait != None:
            self.gait_select = gait

        # larger number means lift ooff earlier
        if self.gait_select == "walk":
            # walking gait
            self.gait = [4,2,3,1]
        elif self.gait_select == "trot":
            # trotting gait
            self.gait = [1,3,1,3]
        else:
            self.gait = [4,2,3,1]
    
    def generate_gait(self):
        self.CMDS = [[] for i in range(len(self.legs)*2)]   # generate CMD lists
        self.phase = pd.DataFrame()   # generate phase lists
        # get cmd list from each legs
        for i, leg in enumerate(self.legs):
            cmd_temp = leg.extend(self.data_len)
            # series = pd.Series(data= leg.phase , name= f'Leg_{i+1}')
            self.phase[f'Leg_{i+1}'] = leg.phase
            # separate theta and beta
            self.CMDS[i*2] = list(map(lambda x: x[0]    , cmd_temp))                                # theta
            self.CMDS[i*2+1] = list(map(lambda x: x[1]  , cmd_temp))   # beta
    
    # plot legs on figure
    def plot(self, iteration, fig, ax):
        for i, leg in enumerate(self.legs):
            pos = leg.get_point_at_time(int(iteration))
            leg.leg.plot_by_angle(pos[0], pos[1],
                            O=np.array([self.offset_x[i]+self.BL/2 + (self.Traj.V * ((iteration-5000)/1000 if iteration >5000 else 0)),
                                        self.Traj.stand_height]),
                            ax=ax)
    
    # save smd datas
    def save(self, Dir = '', output_file_name = ''):
        if output_file_name == '':
            self.output_file_name = "".join([f"{self.gait_select}_Traj_",
                                        "_Hip_height"   +str(self.Traj.stand_height),
                                        "_Step_Length"  +str(self.Traj.step_length),
                                        "_Step_Height"  +str(self.Traj.step_height),
                                        "_Velocity"     +str(round(self.Traj.V, 2)),
                                        "_Distance"     +str(self.Target_dist)])
        else:
            self.output_file_name = output_file_name
        # save command into csv
        print("Saving to", Dir+"CSV/"+self.output_file_name+".csv")
        
        df = pd.DataFrame(np.array(self.CMDS ).T)
        df.to_csv(Dir+"CSV/"+self.output_file_name+".csv", index=False, header= False)
        
        print("Saving Phase to", Dir+"Phase/"+self.output_file_name+"_Phase.csv")
        # df_phase = pd.DataFrame(np.array(self.phase).T)
        self.phase.to_csv(Dir+"Phase/"+self.output_file_name+"_Phase.csv", index=False)
        
    
    def __getitem__(self, key):
        """
            Allows access to joint positions using [] operator.
            Example: legmodel['G'] returns the position of joint G.
        """
        if key not in self.__dict__:
            raise KeyError(f"Var {key}' not found.")
        return self.__dict__[key]    
    
if __name__ == "__main__":
    Gait = Gait_Generator()
    Gait.generate_gait()
    t = 0                                           # Initialize time
    cmd_record = [[] for _ in range(8)]             # Initialize command record for each leg

    # animation settings
    fig,ax = plt.subplots(figsize=(14, 4))  
    ani_interval = 100                              # Animation interval in milliseconds
    speed = ani_interval /( Gait.Traj.dt *1000)          # Frame rate in milliseconds
    frame_len = int((Gait.data_len+5000)/speed)               # Total frame length

    # animation function
    def update(frame):
        global t, ax, init_index, offset_x, offset_z, gait, cmd_record, speed, traj_quad_point
        # fig setting
        ax.clear()
        ax.set_xlim(-Gait.BL/2, Gait.Target_dist+Gait.BL)
        ax.set_ylim(0, Gait.BH*2)
        ax.set_aspect('equal')
        ax.set_title(Gait.gait_select+'Animation'+f'T = {t:.2f}s'+
                    '\n' + f'Stand Height: {Gait.Traj.stand_height}m,'+
                    f' Step Length: {Gait.Traj.step_length}m,'+
                    f' Step Height: {Gait.Traj.step_height}m\n'+
                    f'velocity: {Gait.Traj.V:.2f}m/s')
        ax.grid()
        Gait.plot(fig=fig, ax=ax, iteration= frame*speed)
        t += Gait.Traj.dt*speed
    # func_animation = FuncAnimation(fig, update, frames=len(Traj_Curve)*5, interval=Traj["dt"]*1000, repeat=False)
    func_animation = FuncAnimation(fig, update, frames=frame_len, interval=ani_interval, repeat=False)

    save_data = True
    # save_data = False

    # create output file name
    path = "Output_datas/"
    if save_data:
        Gait.save(Dir=path)
        output_file_name = Gait.output_file_name
        # # # saving animation
        # print("Saving animation to", path+'Videos/'+output_file_name+".mp4")
        # func_animation.save(path+'Videos/'+output_file_name + ".mp4", fps=10, writer='ffmpeg')
        
    else:
        plt.show()
    
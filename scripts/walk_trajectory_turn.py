import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.animation import FuncAnimation
from legwheel.models import leg_model as LegModel
from legwheel.visualization import plot_leg as PlotLeg
from legwheel.utils.utils import *
from legwheel.bezier import swing

sim = False
leg_model = LegModel.LegModel(sim=sim)

#### User-defined parameters ####
animate = True  # create animate file
output_file_name = 'walk_turn'
transform = True   # tramsform to initial configuration before first command
BL = 0.444  # body length, 44.4 cm
BH = 0.2     # body height, 20 cm
CoM_bias = 0.0    # x bias of center of mass
velocity = 0.1     # velocity of hip, meter per second
sampling = 1000    # sampling rate, how many commands to one motor per second.
stand_height = 0.25
step_length = np.array([0.3, 0.2, 0.2, 0.3])
step_height = 0.04
forward_distance = 5.0  # distance to walk


# Use self-defined initial configuration
use_init_conf = False
init_eta = np.array([1.7908786895256839,0.7368824288764617,1.1794001564068406,-0.07401410141135822,1.1744876957173913,-1.8344700758454735e-15,1.790992783013031,5.5466991499313485])
init_theta = init_eta[[0,2,4,6]]
init_beta  = init_eta[[1,3,5,7]]
init_beta[[0, 3]] *= -1


#### Dependent parameters ####
swing_time = 0.15    # duty: 0.8~1.0
# Get foothold in hip coordinate from initial configuration
relative_foothold = np.zeros((4, 2))
for i in range(4):
    leg_model.contact_map(init_theta[i], init_beta[i])
    leg_model.forward(init_theta[i], init_beta[i])
    if leg_model.rim == 1:
        relative_foothold[i, 0] = leg_model.U_l[0]
    elif leg_model.rim == 2:
        relative_foothold[i, 0] = leg_model.L_l[0]
    elif leg_model.rim == 3:
        relative_foothold[i, 0] = leg_model.G[0]
    elif leg_model.rim == 4:
        relative_foothold[i, 0] = leg_model.L_r[0]
    elif leg_model.rim == 5:
        relative_foothold[i, 0] = leg_model.U_r[0]
    else: 
        print("Leg cannot contact ground.")
    relative_foothold[i, 1] = -stand_height
# Get initial leg duty  
first_swing_leg = np.argmin(relative_foothold[:, 0])
if (not use_init_conf) or (first_swing_leg==0):
    duty = np.array([1-swing_time, 0.5-swing_time, 0.5, 0.0])   # initial duty, left front leg first swing
elif first_swing_leg==1:
    duty = np.array([0.5-swing_time, 1-swing_time, 0.0, 0.5])   # initial duty, right front leg first swing
elif first_swing_leg==2:
    duty = np.array([0.5-2*swing_time, 1-2*swing_time, 1-swing_time, 0.5-swing_time]) # initial duty, right hind leg first swing
elif first_swing_leg==3:
    duty = np.array([1-2*swing_time, 0.5-2*swing_time, 0.5-swing_time, 1-swing_time]) # initial duty, left hind leg first swing
swing_phase = np.array([0, 0, 0, 0]) # initial phase, 0:stance, 1:swing
# Get foothold in world coordinate
hip = np.array([[BL/2, stand_height],
                [BL/2, stand_height],
                [-BL/2, stand_height],
                [-BL/2, stand_height]])
if use_init_conf:
    foothold = np.array([hip[0] + relative_foothold[0],   # initial leg configuration
                        hip[0] + relative_foothold[1],
                        hip[2] + relative_foothold[2],
                        hip[2] + relative_foothold[3]])
else:
    foothold = np.array([hip[0] + [-step_length[0]/2*(1-swing_time), -stand_height],   # initial leg configuration, left front leg first swing
                        hip[0] + [step_length[1]/8*(1-swing_time), -stand_height],
                        hip[2] + [-step_length[2]/8*(1-swing_time), -stand_height],
                        hip[2] + [step_length[3]/2*(1-swing_time), -stand_height]])
foothold += np.array([[CoM_bias, 0]])
# Increment per one sample
dS = velocity/sampling * step_length / np.average(step_length)    # hip traveling distance per one sample
incre_duty = dS / step_length   # duty increment per one sample
print(dS)
print(incre_duty)

#### Walk ####
# Initial stored data
theta_list = [[] for _ in range(4)]
beta_list = [[] for _ in range(4)]
hip_list = [[] for _ in range(4)]
swing_length_arr = [[] for _ in range(4)]
swing_angle_arr = [[] for _ in range(4)]
sp = [[] for _ in range(4)]
traveled_distance = 0

# Initial teata, beta
contact_rim = ["G", "L_l", "L_r", "U_l", "U_r"]
rim_idx = [3, 2, 4, 1, 5]
contact_hieght = [leg_model.r, leg_model.radius, leg_model.radius, leg_model.radius, leg_model.radius]
for i in range(4):
    # calculate contact rim of initial pose
    for j in range(5):
        theta, beta = leg_model.inverse(foothold[i]+np.array([0, contact_hieght[j]]) - hip[i], contact_rim[j])
        leg_model.contact_map(theta, beta)
        if leg_model.rim == rim_idx[j]:
            break
    theta_list[i].append(theta)
    beta_list[i].append(beta)
    hip_list[i].append(hip[i].copy())
    
# Start walking
while traveled_distance <= forward_distance:
    for i in range(4):
        if swing_phase[i] == 0: # stance phase     
            theta, beta = leg_model.move(theta_list[i][-1], beta_list[i][-1], hip[i] - hip_list[i][-1])
        else:   # swing phase     
            swing_phase_ratio = (duty[i]-(1-swing_time))/(swing_time)
            curve_point = sp[i].getFootendPoint(swing_phase_ratio) # G position in world coordinate
            theta, beta = leg_model.inverse(curve_point - hip[i], 'G')
        theta_list[i].append(theta)
        beta_list[i].append(beta)
        hip_list[i].append(hip[i].copy())
        
        duty[i] += incre_duty[i]
        if duty[i] >= (1-swing_time) and swing_phase[i] == 0:
            swing_phase[i] = 1
            foothold[i] = [hip[i][0], 0] + ((1-swing_time)/2+swing_time)*np.array([step_length[i], 0])
            # Bezier curve for swing phase
            leg_model.forward(theta_list[i][-1], beta_list[i][-1])
            p_lo = hip[i] + leg_model.G # G position when leave ground
            # calculate contact rim when touch ground
            for j in [0, 1, 3]: # G, L_l, R_l
                theta, beta = leg_model.inverse( np.array([step_length[i]/2*(1-swing_time), -stand_height+contact_hieght[j]]), contact_rim[j])
                leg_model.contact_map(theta, beta)  # also get joint positions when touch ground, in polar coordinate (x+jy).
                if leg_model.rim == rim_idx[j]:
                    touch_rim = leg_model.rim
                    break
            # G position when touch ground
            if touch_rim == 3:  # G
                p_td = foothold[i] + np.array([0, leg_model.r])
            elif touch_rim == 2:  # L_l
                p_td = foothold[i] + np.array([0, leg_model.radius]) + [leg_model.G.real-leg_model.L_l.real, leg_model.G.imag-leg_model.L_l.imag]
            elif touch_rim == 1:  # U_l
                p_td = foothold[i] + np.array([0, leg_model.radius]) + [leg_model.G.real-leg_model.U_l.real, leg_model.G.imag-leg_model.U_l.imag]
            sp[i] = swing.SwingProfile(p_td[0] - p_lo[0], step_height, 0.0, 0.0, 0.0, 0.0, 0.0, p_lo[0], p_lo[1], p_td[1] - p_lo[1])
        elif duty[i] >= 1.0:
            swing_phase[i] = 0
            duty[i] -= 1.0

        hip[i] += [dS[i], 0]
    traveled_distance += dS[1]

theta_list = np.array(theta_list)
beta_list = np.array(beta_list)
hip_list = np.array(hip_list)
create_command_csv(theta_list, beta_list, output_file_name, transform=transform)

# Check for theta range
max_theta = np.deg2rad(160)
min_theta = np.deg2rad(17)
limit_u = theta_list > max_theta   # theta exceeding upper bound set to upper bound
limit_l = theta_list < min_theta   # theta below lower bound set to lower bound
print("Total limit upper bound:", np.sum(limit_u))
print("Total limit lower bound:", np.sum(limit_l))

# Animation
if animate:
    fps = 10
    divide = sampling//fps
    fig, ax = plt.subplots( figsize=(10, 5) )

    plot_leg = PlotLeg.PlotLeg(sim=sim)
    plot_leg.setting()
        
    number_command = theta_list.shape[1]
    def plot_update(frame):
        global ax
        ax.clear()  # clear plot
        
        #### Plot ####
        ax.set_aspect('equal')  # 座標比例相同
        ax.set_xlim(-0.4, 1.1)
        ax.set_ylim(-0.1, 0.5)
        
        # Ground
        # Whole Terrain
        plt.plot([-0.4, 1.1], [0, 0], 'g-') # hip trajectory on the stair
        plt.grid(True)
        
        
        plt.plot(*(( hip_list[0, frame*divide]+ hip_list[2, frame*divide])/2), 'P', color="orange", ms=10, mec='k') # center of mass    
        for i in range(4):
            ax = plot_leg.plot_leg(theta_list[i, frame*divide], beta_list[i, frame*divide], hip_list[i, frame*divide, :], ax)

    ani = FuncAnimation(fig, plot_update, frames=number_command//divide)
    ani.save(output_file_name + ".mp4", fps=fps)

    
    
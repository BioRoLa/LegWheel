import pandas as pd
from legwheel.planners.gait_generator import Gait_Generator

# path = 'docs/notes/design_48_runs.csv'
path = 'docs/notes/Exp index 0904.csv'
exp_datas = pd.read_csv(path)

for i in exp_datas.index:
    stand_H = exp_datas['body_h_cm'][i]/100
    step_L = exp_datas['step_len_cm'][i]/100
    velocity = exp_datas['speed_m_per_s'][i]
    try:
        gait = Gait_Generator(stand_height=stand_H,
                                            step_length=step_L,
                                            velocity=velocity,
                                            distance= 2
                                            )
        gait.generate_gait()
        gait.save(Dir="Output_datas/0904/", output_file_name='COT_Exp_Index_'+str(exp_datas['run_index'][i]))
    except Exception as e:
        print(f'error occur while generating file COT_Exp_Index_{exp_datas["run_index"][i]}: {e}')
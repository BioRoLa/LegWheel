import sys
import pandas as pd
import numpy as np
from legwheel.models import leg_model as LegModel

leg = LegModel.LegModel()
Dir = 'Output_datas/0904/CSV/'
# path = 'COT_Exp_index_49.csv'

# df = pd.read_csv(Dir+path, header = None)

for i in range(49,145):
    try:
        path = f'COT_Exp_index_{i}.csv'
        df = pd.read_csv(Dir+path, header=None)
        df['exceed'] = sum(df[j*2] > np.deg2rad(160) for j in range(4) )
        df['Under'] = sum(df[j*2] < np.deg2rad(16.9) for j in range(4) )
        
        print(f'Limit exceed count @ index_{i}:',sum(df['exceed']), sep='\t')
        print(f'Limit Under count @ index_{i}:',sum(df['Under']), sep='\t')
        
        print()
    except:
        pass
        # for j in range(4):
        #     leg.forward(df[ j*2 ], df[ j*2+1 ])
        # try:
            
        # except:
        #     print(f'Fail while reading index{i}')

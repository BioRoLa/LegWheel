"""
Gait Command Generation Example.
Derived from docs/notes/Note_While_ICRA.ipynb.

This script demonstrates how to generate coordinated gait commands 
for the four legs of the Corgi robot and view the resulting 
joint angle sequences.
"""

import numpy as np
import pandas as pd
from legwheel.planners.gait_generator import Gait_Generator

def main():
    # 1. Initialize Gait Generator
    # Uses default WALK_GAIT [4, 2, 3, 1] if not specified
    gait = Gait_Generator()
    
    # 2. Generate Commands
    print("Generating gait commands...")
    gait.generate_gait()
    
    # 3. Export to Pandas for easy analysis
    # CMDS structure is usually [Theta_L1, Beta_L1, Theta_L2, Beta_L2, ...]
    data = np.array(gait.CMDS).T
    df = pd.DataFrame(data)
    
    # Labeling columns for clarity (assuming 2 params per leg)
    columns = []
    for i in range(4):
        columns.extend([f'L{i+1}_Theta', f'L{i+1}_Beta'])
    df.columns = columns
    
    # 4. Show results
    print("\nGait Command Data (First 10 steps):")
    print(df.head(10))
    
    print(f"\nTotal steps generated: {len(df)}")
    print(f"Sampling rate: {1/gait.Traj.dt} Hz")
    
    # Optional: Save to CSV
    # df.to_csv('gait_commands.csv', index=False)
    # print("Commands saved to gait_commands.csv")

if __name__ == "__main__":
    main()

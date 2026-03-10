# plot_comparison.py

import pandas as pd
import matplotlib
matplotlib.use('TkAgg') # Forza l'uso del backend grafico Tkinter
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
import os

def main():
    sim_file = "sim_step_response.csv"
    real_file = "real_step_response.csv"

    # Check if files exist
    if not os.path.exists(sim_file):
        print(f"[ERROR] Could not find {sim_file}")
        return
    if not os.path.exists(real_file):
        print(f"[ERROR] Could not find {real_file}")
        return

    # Load data
    df_sim = pd.read_csv(sim_file)
    df_real = pd.read_csv(real_file)

    # Extract target (assuming target is constant, we take the first value)
    target_rad = df_sim['target_rad'].iloc[0]

    # Create the plot
    plt.figure(figsize=(10, 6))
    
    # Plot Simulated Data (Blue dashed line)
    plt.plot(df_sim['time_s'], df_sim['position_rad'], 
             label='Simulated (Isaac Lab)', color='blue', linestyle='--', linewidth=2)
    
    # Plot Real Data (Red solid line)
    plt.plot(df_real['time_s'], df_real['position_rad'], 
             label='Real (AgileX Piper)', color='red', linewidth=2)
    
    # Plot Target Line (Green dotted line)
    plt.axhline(y=target_rad, color='green', linestyle=':', linewidth=2, label=f'Target ({target_rad:.3f} rad)')

    # Formatting the chart
    plt.title('Step Response Comparison: Sim-to-Real', fontsize=16)
    plt.xlabel('Time (Seconds)', fontsize=12)
    plt.ylabel('Joint Position (Radians)', fontsize=12)
    plt.grid(True, which='both', linestyle='--', alpha=0.7)
    plt.legend(fontsize=12)
    plt.tight_layout()

    # Show the magic
    print("[INFO] Displaying plot. Close the window to exit.")
    plt.show()

if __name__ == "__main__":
    main()

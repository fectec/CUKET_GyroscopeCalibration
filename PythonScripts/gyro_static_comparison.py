import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import sys
import os

# ================= USER CONFIGURATION =================

# List the filenames of the static test logs you want to compare
# UPDATE THIS LIST with your actual file names
LOG_FILES = [
    "TEST1_STATIC.txt",
    "TEST2_STATIC.txt",
    "TEST3_STATIC.txt",
    "TEST4_STATIC.txt"
]

# ======================================================

def parse_static_logs(file_list):
    """
    Parses the header section of the static test log files.
    Returns a list of dictionaries containing the extracted data.
    """
    parsed_data = []

    for idx, filename in enumerate(file_list):
        if not os.path.exists(filename):
            print(f"Warning: File '{filename}' not found. Skipping.")
            continue

        print(f"Parsing file {idx + 1}: {filename}...")
        
        entry = {
            'id': idx + 1,
            'filename': filename,
            'x': {},
            'y': {},
            'z': {}
        }

        try:
            with open(filename, 'r') as f:
                # Read only the top portion (header)
                # The header ends when "RAW LOG DATA STARTS BELOW" is found
                for line in f:
                    line = line.strip()
                    
                    if "RAW LOG DATA" in line:
                        break
                    
                    # Parse CSV lines: Axis,Omega_Up,Omega_Down,Bias
                    parts = line.split(',')
                    if len(parts) == 4:
                        axis = parts[0].upper()
                        if axis in ['X', 'Y', 'Z']:
                            try:
                                omega_up = float(parts[1])
                                omega_down = float(parts[2])
                                bias = float(parts[3])
                                
                                entry[axis.lower()] = {
                                    'up': omega_up,
                                    'down': omega_down,
                                    'bias': bias
                                }
                            except ValueError:
                                continue
            
            # Verify we got all axes
            if entry['x'] and entry['y'] and entry['z']:
                parsed_data.append(entry)
            else:
                print(f"Warning: Incomplete header data in '{filename}'. Skipping.")

        except Exception as e:
            print(f"Error reading '{filename}': {e}")

    return parsed_data

def plot_comparison(data):
    if not data:
        print("No valid data to plot.")
        return

    # Generate distinct colors for each log file
    num_files = len(data)
    colors = cm.rainbow(np.linspace(0, 1, num_files))
    
    # Create a 3x3 subplot grid
    # Rows: Axes (X, Y, Z)
    # Cols: Metrics (Omega Up, Omega Down, Bias)
    fig, axs = plt.subplots(3, 3, figsize=(16, 12))
    fig.suptitle('Comparison of Gyroscope Static Parameters Across Multiple Tests', fontsize=16, fontweight='bold')
    
    # Define column titles - Using raw strings (r'') to fix SyntaxWarning
    cols = [r'$\bar{\omega}_{up}$ [dps]', r'$\bar{\omega}_{down}$ [dps]', r'Bias $b$ [dps]']
    rows = ['X-Axis', 'Y-Axis', 'Z-Axis']

    # Set headers for columns and rows
    for ax, col in zip(axs[0], cols):
        ax.set_title(col, fontsize=12, fontweight='bold')

    for ax, row in zip(axs[:,0], rows):
        # FIXED: Removed size='large' to avoid conflict with fontsize
        ax.set_ylabel(row, fontsize=12, fontweight='bold', rotation=90)

    # Mapping keys to subplot columns
    metrics = ['up', 'down', 'bias']
    axes_keys = ['x', 'y', 'z']

    # --- PLOTTING LOOP ---
    for i, axis_key in enumerate(axes_keys):       # Row index (0=X, 1=Y, 2=Z)
        for j, metric_key in enumerate(metrics):   # Col index (0=Up, 1=Down, 2=Bias)
            
            current_ax = axs[i, j]
            current_ax.grid(True, linestyle='--', alpha=0.6)
            
            # Plot each file as a single point
            for idx, entry in enumerate(data):
                val = entry[axis_key][metric_key]
                log_num = entry['id']
                
                # Scatter point
                current_ax.scatter(log_num, val, color=colors[idx], s=100, label=f"Test {log_num}" if (i==0 and j==0) else "")
                
            # Set X-axis integer ticks
            current_ax.set_xticks([d['id'] for d in data])
            current_ax.set_xlabel('Test Log Number')

    # --- LEGEND ---
    # Create a custom legend for the files
    legend_elements = []
    for idx, entry in enumerate(data):
        legend_elements.append(plt.Line2D([0], [0], marker='o', color='w', label=f"Test {entry['id']}: {entry['filename']}", 
                          markerfacecolor=colors[idx], markersize=10))

    fig.legend(handles=legend_elements, loc='lower center', ncol=3, bbox_to_anchor=(0.5, 0.01), fontsize=10)
    
    # Adjust layout to make room for legend
    plt.subplots_adjust(bottom=0.15, hspace=0.4, wspace=0.3)
    
    plt.show()

if __name__ == "__main__":
    print("--- Gyroscope Biases Comparison Tool ---")
    
    # 1. Parse Data
    parsed_results = parse_static_logs(LOG_FILES)
    
    print(f"Successfully parsed {len(parsed_results)} logs.")
    
    # 2. Plot
    plot_comparison(parsed_results)

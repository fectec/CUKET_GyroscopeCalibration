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
            # Store the color index to link back to the legend
            'color_idx': idx, 
            'x': {},
            'y': {},
            'z': {}
        }

        try:
            with open(filename, 'r') as f:
                # Read only the top portion (header)
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
                                # Skip header line
                                if axis == 'AXIS': continue
                                
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

def plot_normal(data):
    """Plots data based on the Test ID (Original Order)."""
    if not data:
        print("No valid data to plot.")
        return

    num_files = len(data)
    colors = cm.rainbow(np.linspace(0, 1, num_files))
    
    fig, axs = plt.subplots(3, 3, figsize=(16, 12))
    fig.suptitle('Comparison of Gyroscope Static Parameters Across Tests', fontsize=16, fontweight='bold')
    
    cols = [r'$\bar{\omega}_{up}$ [dps]', r'$\bar{\omega}_{down}$ [dps]', r'Bias $b$ [dps]']
    rows = ['X-Axis', 'Y-Axis', 'Z-Axis']
    metrics = ['up', 'down', 'bias']
    axes_keys = ['x', 'y', 'z']

    # Set Headers
    for ax, col in zip(axs[0], cols):
        ax.set_title(col, fontsize=12, fontweight='bold')
    for ax, row in zip(axs[:,0], rows):
        ax.set_ylabel(row, fontsize=12, fontweight='bold', rotation=90)

    # Plotting Loop
    for i, axis_key in enumerate(axes_keys):       
        for j, metric_key in enumerate(metrics):   
            current_ax = axs[i, j]
            current_ax.grid(True, linestyle='--', alpha=0.6)
            
            for entry in data:
                val = entry[axis_key][metric_key]
                log_num = entry['id']
                color = colors[entry['color_idx']]
                
                current_ax.scatter(log_num, val, color=color, s=100)
                
            current_ax.set_xticks([d['id'] for d in data])
            current_ax.set_xlabel('Test Log Number')

    # Legend
    _add_legend(fig, data, colors)
    plt.subplots_adjust(bottom=0.15, hspace=0.4, wspace=0.3)
    plt.show()

def plot_sorted(data):
    """Plots data sorted by the value of the metric (Lowest to Highest)."""
    if not data:
        print("No valid data to plot.")
        return

    num_files = len(data)
    colors = cm.rainbow(np.linspace(0, 1, num_files))
    
    fig, axs = plt.subplots(3, 3, figsize=(16, 12))
    fig.suptitle('Comparison of Gyroscope Static Parameters Across Tests (Sorted by Value)', fontsize=16, fontweight='bold')
    
    cols = [r'$\bar{\omega}_{up}$ [dps]', r'$\bar{\omega}_{down}$ [dps]', r'Bias $b$ [dps]']
    rows = ['X-Axis', 'Y-Axis', 'Z-Axis']
    metrics = ['up', 'down', 'bias']
    axes_keys = ['x', 'y', 'z']

    # Set Headers
    for ax, col in zip(axs[0], cols):
        ax.set_title(col, fontsize=12, fontweight='bold')
    for ax, row in zip(axs[:,0], rows):
        ax.set_ylabel(row, fontsize=12, fontweight='bold', rotation=90)

    # Plotting Loop
    for i, axis_key in enumerate(axes_keys):       
        for j, metric_key in enumerate(metrics):   
            current_ax = axs[i, j]
            current_ax.grid(True, linestyle='--', alpha=0.6)
            
            # Collect data for this subplot
            metric_data = []
            for entry in data:
                metric_data.append({
                    'val': entry[axis_key][metric_key],
                    'color': colors[entry['color_idx']]
                })

            # Sort by Value
            metric_data_sorted = sorted(metric_data, key=lambda x: x['val'])
            x_ranks = list(range(1, len(metric_data_sorted) + 1))
            
            # Plot
            for rank, d in enumerate(metric_data_sorted):
                 current_ax.scatter(rank + 1, d['val'], color=d['color'], s=100)

            current_ax.set_xticks(x_ranks)
            current_ax.set_xlabel('Sorted Rank (Low -> High)')

    # Legend
    _add_legend(fig, data, colors)
    plt.subplots_adjust(bottom=0.15, hspace=0.4, wspace=0.3)
    plt.show()

def _add_legend(fig, data, colors):
    legend_elements = []
    # Sort legend by ID for consistency
    sorted_data_for_legend = sorted(data, key=lambda x: x['id']) 
    
    for entry in sorted_data_for_legend:
        idx = entry['color_idx']
        legend_elements.append(plt.Line2D([0], [0], marker='o', color='w', 
                          label=f"Test {entry['id']}: {entry['filename']}", 
                          markerfacecolor=colors[idx], markersize=10))

    fig.legend(handles=legend_elements, loc='lower center', ncol=3, bbox_to_anchor=(0.5, 0.01), fontsize=10)

def main():
    print("--- Gyroscope Biases Comparison Tool ---")
    
    # 1. Parse Data once
    parsed_results = parse_static_logs(LOG_FILES)
    
    if not parsed_results:
        print("No valid logs found. Exiting.")
        return

    print(f"Successfully parsed {len(parsed_results)} logs.")

    while True:
        print("\nSelect Plot Mode:")
        print("1. Normal Order (X-axis = Test ID)")
        print("2. Sorted by Value (X-axis = Rank Low->High)")
        print("Q. Quit")
        
        choice = input("Enter choice (1/2/Q): ").strip().upper()

        if choice == '1':
            plot_normal(parsed_results)
        elif choice == '2':
            plot_sorted(parsed_results)
        elif choice == 'Q':
            print("Exiting.")
            break
        else:
            print("Invalid choice. Please try again.")

if __name__ == "__main__":
    main()